#include "interactive_simulator.hpp"

#include <chrono>
#include <cstring>
#include <thread>

#include "simulate_src/glfw_adapter.h"
#include "mia_hand_mujoco/plugin/index_thumb_actuator.h"

namespace mia_hand_mujoco
{

namespace
{
using Seconds = std::chrono::duration<double>;

// Matches the divergence check in MuJoCo simulate/main.cc
const char* check_diverged(const mjData* d)
{
  if (d->warning[mjWARN_BADQPOS].number ||
      d->warning[mjWARN_BADQVEL].number ||
      d->warning[mjWARN_BADQACC].number ||
      d->warning[mjWARN_BADCTRL].number) {
    return "Simulation is diverging; pausing.";
  }
  return nullptr;
}
}  // namespace

InteractiveSimulator& InteractiveSimulator::get_instance()
{
  static InteractiveSimulator instance;
  return instance;
}

const char* InteractiveSimulator::get_error_msg()
{
  return err_msg_;
}

bool InteractiveSimulator::simulate(const char* model, std::promise<bool>&& sim_start_ok)
{
  if (mjVERSION_HEADER != mj_version()) {
    mju_warning("MuJoCo API and library version do not match.");
  }
  return get_instance().simulate_impl(model, std::move(sim_start_ok));
}

void InteractiveSimulator::stop_simulation()
{
  if (sim_) {
    sim_->exitrequest.store(1);
  }
}

void InteractiveSimulator::read_jnt_pos(
  double& thumb_pos, double& index_pos, double& mrl_pos)
{
  std::lock_guard<std::mutex> lock(sim_mtx_);

  mjtByte index_reversed =
    reinterpret_cast<plugin::IndexThumbActuator*>(
      mj_data_->plugin_data[plugin_instance_])->is_index_reversed();

  thumb_pos = jnt_pos_state_[0];
  index_pos = index_reversed ? -jnt_pos_state_[1] : jnt_pos_state_[1];
  mrl_pos = jnt_pos_state_[2];
}

void InteractiveSimulator::read_jnt_vel(
  double& thumb_vel, double& index_vel, double& mrl_vel)
{
  std::lock_guard<std::mutex> lock(sim_mtx_);

  mjtByte index_reversed =
    reinterpret_cast<plugin::IndexThumbActuator*>(
      mj_data_->plugin_data[plugin_instance_])->is_index_reversed();

  thumb_vel = jnt_vel_state_[0];
  index_vel = index_reversed ? -jnt_vel_state_[1] : jnt_vel_state_[1];
  mrl_vel = jnt_vel_state_[2];
}

void InteractiveSimulator::set_jnt_pos(uint_fast8_t jnt, double pos)
{
  std::lock_guard<std::mutex> lock(sim_mtx_);
  jnt_pos_cmd_[jnt] = pos;
}

void InteractiveSimulator::set_jnt_vel(uint_fast8_t jnt, double vel)
{
  std::lock_guard<std::mutex> lock(sim_mtx_);
  jnt_vel_cmd_[jnt] = vel;
}

void InteractiveSimulator::stop_jnt(uint_fast8_t jnt)
{
  std::lock_guard<std::mutex> lock(sim_mtx_);
  // hold current position as the new command
  jnt_pos_cmd_[jnt] = jnt_pos_state_[jnt];
}

InteractiveSimulator::InteractiveSimulator()
: mj_model_(nullptr),
  mj_data_(nullptr),
  plugin_instance_(-1)
{
  err_msg_[0] = '\0';

  mjv_defaultCamera(&mjv_camera_);
  mjv_defaultOption(&mjv_options_);
  mjv_defaultPerturb(&mjv_pert_);

  for (double& v : jnt_vel_state_) { v = 0.0; }
  for (double& v : jnt_vel_cmd_)   { v = 0.0; }
  for (double& v : jnt_pos_state_) { v = 0.0; }
  for (double& v : jnt_pos_cmd_)   { v = 0.0; }
}

void InteractiveSimulator::control_cb(const mjModel* model, mjData* data)
{
  get_instance().control_cb_impl(model, data);
}

void InteractiveSimulator::control_cb_impl(const mjModel* /* model */, mjData* data)
{
  std::lock_guard<std::mutex> lock(sim_mtx_);
  data->ctrl[0] = jnt_pos_cmd_[0];
  data->ctrl[1] = jnt_pos_cmd_[1];
  data->ctrl[2] = jnt_pos_cmd_[2];
}

bool InteractiveSimulator::get_plugin_instance(const mjModel* p_mjm)
{
  for (int i = 0; i < p_mjm->nu; ++i) {
    if (p_mjm->actuator_plugin[i] != -1) {
      plugin_instance_ = p_mjm->actuator_plugin[i];
      return true;
    }
  }
  return false;
}

// Physics thread: loads model into Simulate, runs physics loop.
// Mirrors PhysicsThread + PhysicsLoop from MuJoCo simulate/main.cc.
void InteractiveSimulator::physics_thread_fn(
  const char* model_path, std::promise<bool>&& sim_start_ok)
{
  // Tell Simulate to show "loading" in the window title
  sim_->LoadMessage(model_path);

  // sim->Load() blocks until the render thread (RenderLoop) has processed
  // the load request (loadrequest goes to 0).
  sim_->Load(mj_model_, mj_data_, model_path);

  {
    // Run forward dynamics once to initialise rendering state
    const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
    mj_forward(mj_model_, mj_data_);
  }

  // Signal on_configure() that the simulation is ready
  sim_start_ok.set_value(true);

  // --- Physics loop (adapted from PhysicsLoop in MuJoCo simulate/main.cc) ---
  using Clock = mujoco::Simulate::Clock;

  constexpr double kSyncMisalign = 0.1;      // 100 ms misalignment threshold
  constexpr double kSimRefreshFraction = 0.7;

  Clock::time_point syncCPU;
  mjtNum syncSim = 0;

  while (!sim_->exitrequest.load()) {
    if (sim_->busywait) {
      std::this_thread::yield();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    {
      const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);

      if (mj_model_) {
        if (sim_->run) {
          bool stepped = false;

          const auto startCPU = Clock::now();
          const auto elapsedCPU = startCPU - syncCPU;
          double elapsedSim = mj_data_->time - syncSim;

          const double slowdown =
            100.0 / sim_->percentRealTime[sim_->real_time_index];
          const bool misaligned =
            std::abs(Seconds(elapsedCPU).count() / slowdown - elapsedSim) >
            kSyncMisalign;

          // Out-of-sync: reset and take a single step
          if (elapsedSim < 0 || elapsedCPU.count() < 0 ||
              syncCPU.time_since_epoch().count() == 0 ||
              misaligned || sim_->speed_changed) {
            syncCPU = startCPU;
            syncSim = mj_data_->time;
            sim_->speed_changed = false;

            sim_->InjectNoise();
            mj_step(mj_model_, mj_data_);
            const char* msg = check_diverged(mj_data_);
            if (msg) {
              sim_->run = 0;
              std::strncpy(sim_->load_error, msg,
                           mujoco::Simulate::kMaxFilenameLength - 1);
            } else {
              stepped = true;
            }
          } else {
            // In-sync: step until simulation is ahead of CPU
            const double refreshTime =
              kSimRefreshFraction / sim_->refresh_rate;
            mjtNum prevSim = mj_data_->time;
            bool measured = false;

            while (Seconds((mj_data_->time - syncSim) * slowdown) <
                       Clock::now() - syncCPU &&
                   Clock::now() - startCPU < Seconds(refreshTime)) {
              if (!measured && elapsedSim) {
                sim_->measured_slowdown =
                  static_cast<float>(
                    std::chrono::duration<double>(elapsedCPU).count() /
                    elapsedSim);
                measured = true;
              }
              sim_->InjectNoise();
              mj_step(mj_model_, mj_data_);
              const char* msg = check_diverged(mj_data_);
              if (msg) {
                sim_->run = 0;
                std::strncpy(sim_->load_error, msg,
                             mujoco::Simulate::kMaxFilenameLength - 1);
                break;
              }
              stepped = true;
              if (mj_data_->time < prevSim) break;
            }
          }

          if (stepped) {
            sim_->AddToHistory();
          }
        } else {
          // Paused: keep rendering up-to-date
          mj_forward(mj_model_, mj_data_);
          sim_->speed_changed = true;
        }

        // Copy joint state to shared arrays under our own mutex
        {
          std::lock_guard<std::mutex> state_lock(sim_mtx_);
          jnt_pos_state_[0] = mj_data_->qpos[1];
          jnt_pos_state_[1] = mj_data_->qpos[2];
          jnt_pos_state_[2] = mj_data_->qpos[3];
          jnt_vel_state_[0] = mj_data_->qvel[1];
          jnt_vel_state_[1] = mj_data_->qvel[2];
          jnt_vel_state_[2] = mj_data_->qvel[3];
        }
      }
    }  // release sim_->mtx
  }
}

bool InteractiveSimulator::simulate_impl(
  const char* model, std::promise<bool>&& sim_start_ok)
{
  bool success = true;

  mj_model_ = mj_loadXML(model, nullptr, err_msg_, sizeof(err_msg_));
  if (!mj_model_) {
    success = false;
  }

  if (success) {
    mj_data_ = mj_makeData(mj_model_);
    if (!mj_data_) {
      success = false;
      std::strcpy(err_msg_, "MuJoCo data preparation failed.");
    }
  }

  if (success && !get_plugin_instance(mj_model_)) {
    success = false;
    std::strcpy(err_msg_, "Index-thumb actuator plugin not found.");
  }

  if (!success) {
    sim_start_ok.set_value(false);
    return false;
  }

  // Create the Simulate UI. GlfwAdapter constructor calls glfwInit() and
  // glfwCreateWindow(), making the GL context current on THIS thread (sim_trd_).
  // RenderLoop() must therefore run on THIS thread.
  mjv_defaultCamera(&mjv_camera_);
  mjv_defaultOption(&mjv_options_);
  mjv_defaultPerturb(&mjv_pert_);

  sim_ = std::make_unique<mujoco::Simulate>(
    std::make_unique<mujoco::GlfwAdapter>(),
    &mjv_camera_, &mjv_options_, &mjv_pert_,
    /* is_passive = */ false);

  mjcb_control = &InteractiveSimulator::control_cb;

  // Physics thread: loads model into Simulate and runs physics loop.
  // RenderLoop() below (on this thread) will process the load request.
  physics_thread_ = std::thread(
    &InteractiveSimulator::physics_thread_fn, this, model, std::move(sim_start_ok));

  // Blocks until the window is closed or exitrequest is set
  sim_->RenderLoop();

  physics_thread_.join();

  mjcb_control = nullptr;

  mj_deleteData(mj_data_);
  mj_deleteModel(mj_model_);
  mj_data_ = nullptr;
  mj_model_ = nullptr;
  sim_.reset();

  return true;
}

}  // namespace mia_hand_mujoco
