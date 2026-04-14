#include "interactive_simulator.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <regex>
#include <sstream>
#include <string>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

#include "simulate_src/glfw_adapter.h"
#include "mia_hand_mujoco/plugin/index_thumb_actuator.h"

namespace
{
constexpr const char* kPlannerWorkdir   = "/miahand_ws/src/dev/grasp_preshaping";
constexpr const char* kPlannerManifest  = "/miahand_ws/src/dev/grasp_preshaping/Cargo.toml";
constexpr const char* kPlannerBinary    = "/miahand_ws/src/dev/grasp_preshaping/target/release/preshaping";
constexpr const char* kPlannerPointcloudTopic = "/segmented_object_cloud";
constexpr const char* kPlannerLogDir    = "/miahand_ws/src/dev/mujoco/log";
constexpr const char* kPlannerMostRecentLog = "/miahand_ws/src/dev/mujoco/log/most_recent.txt";

namespace fs = std::filesystem;

std::string log_display_name(const std::string& log_path)
{
  const std::string stem = fs::path(log_path).stem().string();
  const std::smatch match = [&stem]() {
    std::smatch local_match;
    std::regex_match(stem, local_match, std::regex(R"(^grasp_log_(\d{3,})$)"));
    return local_match;
  }();
  if (match.size() == 2) {
    return std::string("log_") + match[1].str();
  }
  return stem;
}

std::string short_execution_mode_label(int execution_mode)
{
  switch (execution_mode) {
    case 0: return "dry";
    case 1: return "traj";
    case 2: return "pos";
    default: return "unk";
  }
}

bool write_log_header(const std::string& log_path)
{
  std::ofstream log_stream(log_path, std::ios::trunc);
  if (!log_stream.is_open()) return false;
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm local_tm{};
  localtime_r(&now_time, &local_tm);
  log_stream << "Timestamp: " << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S") << '\n';
  return true;
}

void copy_to_most_recent_log(const std::string& log_path)
{
  std::error_code error;
  fs::copy_file(log_path, kPlannerMostRecentLog,
                fs::copy_options::overwrite_existing, error);
}

using Seconds = std::chrono::duration<double>;

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

namespace mia_hand_mujoco
{

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
  plugin_instance_(-1),
  planner_sect_id_(-1),
  planner_transform_mode_value_(0),
  planner_execution_mode_value_(0),
  planner_running_(false),
  planner_status_dirty_(false),
  scene_hand_dirty_(false),
  scene_obj_dirty_(false),
  scene_cam_dirty_(false),
  scene_ui_sync_needed_(false),
  hand_body_id_(-1),
  obj_body_id_(-1),
  cam_body_id_(-1),
  wrist_cam_body_id_(-1),
  scene_sect_id_(-1),
  motion_duration_(1.0),
  motion_sect_id_(-1)
{
  err_msg_[0] = '\0';

  mjv_defaultCamera(&mjv_camera_);
  mjv_defaultOption(&mjv_options_);
  mjv_defaultPerturb(&mjv_pert_);

  for (double& v : jnt_vel_state_) { v = 0.0; }
  for (double& v : jnt_vel_cmd_)   { v = 0.0; }
  for (double& v : jnt_pos_state_) { v = 0.0; }
  for (double& v : jnt_pos_cmd_)   { v = 0.0; }

  for (int i = 0; i < 3; ++i) {
    scene_hand_pos_[i] = 0.0; scene_hand_rpy_[i] = 0.0;
    scene_obj_pos_[i]  = 0.0; scene_obj_rpy_[i]  = 0.0;
    scene_cam_pos_[i]  = 0.0; scene_cam_rpy_[i]  = 0.0;

    motion_hand_tgt_pos_[i] = 0.0; motion_hand_tgt_rpy_[i] = 0.0;
    motion_obj_tgt_pos_[i]  = 0.0; motion_obj_tgt_rpy_[i]  = 0.0;
    motion_cam_tgt_pos_[i]  = 0.0; motion_cam_tgt_rpy_[i]  = 0.0;
  }
  ros_hand_pending_ = {{0,0,0},{1,0,0,0},false};
  ros_obj_pending_  = {{0,0,0},{1,0,0,0},false};
  ros_cam_pending_  = {{0,0,0},{1,0,0,0},false};

  for (int i = 0; i < 3; ++i) {
    imu_ang_vel_[i]    = 0.0;
    imu_lin_acc_[i]    = 0.0;
    imu_mag_field_[i]  = 0.0;
    imu_prev_pos_[i]   = 0.0;
    imu_prev_lin_vel_[i] = 0.0;
  }
  imu_orientation_wxyz_[0] = 1.0;
  imu_orientation_wxyz_[1] = 0.0;
  imu_orientation_wxyz_[2] = 0.0;
  imu_orientation_wxyz_[3] = 0.0;
  imu_prev_quat_[0] = 1.0;
  imu_prev_quat_[1] = 0.0;
  imu_prev_quat_[2] = 0.0;
  imu_prev_quat_[3] = 0.0;
  sim_time_       = 0.0;
  imu_initialized_ = false;

  for (int i = 0; i < 3; ++i) {
    imu2_ang_vel_[i]      = 0.0;
    imu2_lin_acc_[i]      = 0.0;
    imu2_mag_field_[i]    = 0.0;
    imu2_prev_pos_[i]     = 0.0;
    imu2_prev_lin_vel_[i] = 0.0;
  }
  imu2_orientation_wxyz_[0] = 1.0;
  imu2_orientation_wxyz_[1] = 0.0;
  imu2_orientation_wxyz_[2] = 0.0;
  imu2_orientation_wxyz_[3] = 0.0;
  imu2_prev_quat_[0] = 1.0;
  imu2_prev_quat_[1] = 0.0;
  imu2_prev_quat_[2] = 0.0;
  imu2_prev_quat_[3] = 0.0;
  imu2_initialized_ = false;
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
        // Apply pending scene pose overrides before each step
        apply_scene_poses(mj_model_, mj_data_);

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

          // Update IMU data from camera body state
          if (cam_body_id_ >= 0) {
            const mjtNum dt = mj_model_->opt.timestep;
            const mjtNum* pos  = &mj_data_->xpos[cam_body_id_ * 3];
            const mjtNum* quat = &mj_data_->xquat[cam_body_id_ * 4]; // wxyz
            const mjtNum* R    = &mj_data_->xmat[cam_body_id_ * 9];  // row-major

            if (!imu_initialized_) {
              // First step: seed previous values, report zero velocity/accel
              for (int i = 0; i < 3; ++i) {
                imu_prev_pos_[i]     = pos[i];
                imu_prev_lin_vel_[i] = 0.0;
              }
              for (int i = 0; i < 4; ++i) imu_prev_quat_[i] = quat[i];
              imu_initialized_ = true;
            }

            // --- Angular velocity (body frame) via quaternion finite-difference ---
            // delta_q = q_prev^{-1} * q_new  (rotation increment expressed in prev frame)
            // For unit quaternions: inv(q) = q* = [w, -x, -y, -z]
            mjtNum inv_prev[4] = {imu_prev_quat_[0], -imu_prev_quat_[1],
                                  -imu_prev_quat_[2], -imu_prev_quat_[3]};
            mjtNum delta_q[4];
            mju_mulQuat(delta_q, inv_prev, quat);
            // Ensure positive scalar part (shortest-path convention)
            if (delta_q[0] < 0) {
              for (int i = 0; i < 4; ++i) delta_q[i] = -delta_q[i];
            }
            // omega_body ≈ 2 * delta_q[1:4] / dt  (small-angle approximation)
            mjtNum omega_body[3] = {
              2.0 * delta_q[1] / dt,
              2.0 * delta_q[2] / dt,
              2.0 * delta_q[3] / dt
            };

            // --- Linear acceleration (world frame) via position second derivative ---
            mjtNum lin_vel_new[3], lin_acc_world[3];
            for (int i = 0; i < 3; ++i) {
              lin_vel_new[i]   = (pos[i] - imu_prev_pos_[i]) / dt;
              lin_acc_world[i] = (lin_vel_new[i] - imu_prev_lin_vel_[i]) / dt;
            }
            // Subtract gravity so at-rest reads +|g| along the gravity-opposite axis
            const mjtNum* grav = mj_model_->opt.gravity;
            mjtNum a_imu_world[3] = {
              lin_acc_world[0] - grav[0],
              lin_acc_world[1] - grav[1],
              lin_acc_world[2] - grav[2]
            };
            // Rotate to camera body frame: v_body[i] = R^T * v_world
            // R is row-major (R[row*3+col]); R^T * v means: v_body[i] = sum_j R[j*3+i]*v[j]
            mjtNum a_imu_body[3];
            for (int i = 0; i < 3; ++i) {
              a_imu_body[i] = R[0*3+i]*a_imu_world[0]
                            + R[1*3+i]*a_imu_world[1]
                            + R[2*3+i]*a_imu_world[2];
            }

            // --- Magnetometer: world X+ expressed in camera body frame ---
            mjtNum mag_body[3] = {R[0*3+0], R[1*3+0], R[2*3+0]};  // R^T * [1,0,0]

            // Store results (under sim_mtx_ which we already hold)
            for (int i = 0; i < 3; ++i) {
              imu_ang_vel_[i]   = static_cast<double>(omega_body[i]);
              imu_lin_acc_[i]   = static_cast<double>(a_imu_body[i]);
              imu_mag_field_[i] = static_cast<double>(mag_body[i]);
            }
            for (int i = 0; i < 4; ++i) {
              imu_orientation_wxyz_[i] = static_cast<double>(quat[i]);
            }
            sim_time_ = mj_data_->time;

            // Update prev values for next step
            for (int i = 0; i < 3; ++i) {
              imu_prev_pos_[i]     = pos[i];
              imu_prev_lin_vel_[i] = lin_vel_new[i];
            }
            for (int i = 0; i < 4; ++i) imu_prev_quat_[i] = quat[i];
          }

          // --- IMU2: wrist-mounted camera (child of palm_r) ---
          if (wrist_cam_body_id_ >= 0) {
            const mjtNum dt   = mj_model_->opt.timestep;
            const mjtNum* pos  = &mj_data_->xpos[wrist_cam_body_id_ * 3];
            const mjtNum* quat = &mj_data_->xquat[wrist_cam_body_id_ * 4];
            const mjtNum* R    = &mj_data_->xmat[wrist_cam_body_id_ * 9];

            if (!imu2_initialized_) {
              for (int i = 0; i < 3; ++i) {
                imu2_prev_pos_[i]     = pos[i];
                imu2_prev_lin_vel_[i] = 0.0;
              }
              for (int i = 0; i < 4; ++i) imu2_prev_quat_[i] = quat[i];
              imu2_initialized_ = true;
            }

            mjtNum inv_prev2[4] = {imu2_prev_quat_[0], -imu2_prev_quat_[1],
                                   -imu2_prev_quat_[2], -imu2_prev_quat_[3]};
            mjtNum delta_q2[4];
            mju_mulQuat(delta_q2, inv_prev2, quat);
            if (delta_q2[0] < 0) {
              for (int i = 0; i < 4; ++i) delta_q2[i] = -delta_q2[i];
            }
            mjtNum omega2[3] = {
              2.0 * delta_q2[1] / dt,
              2.0 * delta_q2[2] / dt,
              2.0 * delta_q2[3] / dt
            };

            mjtNum lin_vel2[3], lin_acc_world2[3];
            for (int i = 0; i < 3; ++i) {
              lin_vel2[i]       = (pos[i] - imu2_prev_pos_[i]) / dt;
              lin_acc_world2[i] = (lin_vel2[i] - imu2_prev_lin_vel_[i]) / dt;
            }
            const mjtNum* grav = mj_model_->opt.gravity;
            mjtNum a2_world[3] = {
              lin_acc_world2[0] - grav[0],
              lin_acc_world2[1] - grav[1],
              lin_acc_world2[2] - grav[2]
            };
            mjtNum a2_body[3];
            for (int i = 0; i < 3; ++i) {
              a2_body[i] = R[0*3+i]*a2_world[0]
                         + R[1*3+i]*a2_world[1]
                         + R[2*3+i]*a2_world[2];
            }
            mjtNum mag2[3] = {R[0*3+0], R[1*3+0], R[2*3+0]};

            for (int i = 0; i < 3; ++i) {
              imu2_ang_vel_[i]   = static_cast<double>(omega2[i]);
              imu2_lin_acc_[i]   = static_cast<double>(a2_body[i]);
              imu2_mag_field_[i] = static_cast<double>(mag2[i]);
            }
            for (int i = 0; i < 4; ++i) {
              imu2_orientation_wxyz_[i] = static_cast<double>(quat[i]);
            }

            for (int i = 0; i < 3; ++i) {
              imu2_prev_pos_[i]     = pos[i];
              imu2_prev_lin_vel_[i] = lin_vel2[i];
            }
            for (int i = 0; i < 4; ++i) imu2_prev_quat_[i] = quat[i];
          }
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

  // Wire Grasp Planner + Scene Control + Motion Control UI hooks
  sim_->custom_section_init = [this](mujoco::Simulate* sim) {
    add_custom_section(sim);
    add_scene_section(sim);
    add_motion_section(sim);
  };
  sim_->custom_section_event = [this](mujoco::Simulate* sim, int sec, int item) {
    handle_custom_event(sim, sec, item);
    if (sec == scene_sect_id_) {
      handle_scene_event(sim, item);
    } else if (sec == motion_sect_id_) {
      handle_motion_event(sim, item);
    }
  };
  sim_->custom_sync_fn = [this](mujoco::Simulate* sim) {
    sync_custom_status(sim);
    sync_scene_ui(sim);
  };

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

void InteractiveSimulator::add_custom_section(mujoco::Simulate* sim)
{
  planner_sect_id_ = sim->ui1.nsect;

  const mjuiDef planner_def[] = {
    {mjITEM_SECTION,   "Grasp Planner", mjPRESERVE, nullptr,                        "", 0},
    {mjITEM_SEPARATOR, "Pose Source",   1,          nullptr,                        "", 0},
    {mjITEM_RADIO,     "Pose",          1,          &planner_transform_mode_value_,  "Dynamic TF\nLegacy static", 0},
    {mjITEM_SEPARATOR, "Execution",     1,          nullptr,                        "", 0},
    {mjITEM_RADIO,     "Mode",          1,          &planner_execution_mode_value_,  "Dry run\nTrajectory\nPos FF", 0},
    {mjITEM_SEPARATOR, "",              1,          nullptr,                        "", 0},
    {mjITEM_BUTTON,    "Run Planner",   1,          nullptr,                        "", 0},
    {mjITEM_SEPARATOR, "",              1,          nullptr,                        "", 0},
    {mjITEM_STATIC,    "Status",        1,          nullptr,                        "Idle", 0},
    {mjITEM_END,       "",              0,          nullptr,                        "", 0}
  };

  mjui_add(&sim->ui1, planner_def);
  sim->ui1.sect[planner_sect_id_].state = mjSECT_OPEN;

  // Show initial status
  set_status("Idle");
  planner_status_dirty_ = true;
}

void InteractiveSimulator::handle_custom_event(
  mujoco::Simulate* /*sim*/, int sectionid, int itemid)
{
  if (sectionid != planner_sect_id_) return;

  if (itemid == kPlannerItemRunPlanner) {
    const PlannerTransformMode transform_mode =
      (planner_transform_mode_value_ == static_cast<int>(PlannerTransformMode::kLegacyStatic))
        ? PlannerTransformMode::kLegacyStatic
        : PlannerTransformMode::kDynamicTf;

    PlannerExecutionMode execution_mode = PlannerExecutionMode::kDryRun;
    if (planner_execution_mode_value_ == static_cast<int>(PlannerExecutionMode::kTrajectory)) {
      execution_mode = PlannerExecutionMode::kTrajectory;
    } else if (planner_execution_mode_value_ == static_cast<int>(PlannerExecutionMode::kPosFf)) {
      execution_mode = PlannerExecutionMode::kPosFf;
    }

    launch_planner(transform_mode, execution_mode);
  }
}

void InteractiveSimulator::sync_custom_status(mujoco::Simulate* sim)
{
  if (planner_sect_id_ < 0) return;

  std::lock_guard<std::mutex> lock(planner_status_mtx_);
  if (!planner_status_dirty_) return;

  std::snprintf(
    sim->ui1.sect[planner_sect_id_].item[kPlannerItemStatus].multi.name[0],
    mjMAXUINAME, "%s", planner_status_pending_.c_str());
  mjui_update(planner_sect_id_, kPlannerItemStatus,
              &sim->ui1, &sim->uistate, &sim->platform_ui->mjr_context());
  planner_status_dirty_ = false;
}

void InteractiveSimulator::set_status(const std::string& status)
{
  std::lock_guard<std::mutex> lock(planner_status_mtx_);
  planner_status_pending_ = status;
  planner_status_dirty_ = true;
}

void InteractiveSimulator::launch_planner(
  PlannerTransformMode transform_mode, PlannerExecutionMode execution_mode)
{
  bool expected = false;
  if (!planner_running_.compare_exchange_strong(expected, true)) {
    set_status("Planner already running");
    return;
  }

  set_status("Preparing planner");
  std::thread(&InteractiveSimulator::run_planner_worker,
              this, transform_mode, execution_mode).detach();
}

void InteractiveSimulator::run_planner_worker(
  PlannerTransformMode transform_mode, PlannerExecutionMode execution_mode)
{
  const std::string log_path = make_log_path();
  const std::string log_name = log_display_name(log_path);

  if (!write_log_header(log_path)) {
    set_status("Log init failed");
    planner_running_.store(false);
    return;
  }

  set_status(
    std::string("Run ") +
    short_execution_mode_label(static_cast<int>(execution_mode)) +
    " " + log_name);

  const std::string command =
    build_planner_command(transform_mode, execution_mode, log_path);
  const int result = std::system(command.c_str());
  copy_to_most_recent_log(log_path);

  if (result == -1) {
    set_status(std::string("ERR ") + log_name);
  } else if (WIFEXITED(result) && WEXITSTATUS(result) == 0) {
    set_status(std::string("OK ") + log_name);
  } else if (WIFEXITED(result)) {
    set_status(std::string("FAIL") + std::to_string(WEXITSTATUS(result)) +
               " " + log_name);
  } else {
    set_status(std::string("TERM ") + log_name);
  }

  planner_running_.store(false);
}

std::string InteractiveSimulator::build_planner_command(
  PlannerTransformMode transform_mode,
  PlannerExecutionMode execution_mode,
  const std::string& log_path) const
{
  std::ostringstream cmd;
  cmd << "cd " << shell_quote(kPlannerWorkdir) << " && ";

  if (execution_mode == PlannerExecutionMode::kTrajectory) {
    cmd << "ros2 control switch_controllers -c /controller_manager"
        << " --deactivate thumb_pos_ff_controller index_pos_ff_controller mrl_pos_ff_controller"
        << " --activate thumb_trajectory_controller index_trajectory_controller"
        << " mrl_trajectory_controller"
        << " >> " << shell_quote(log_path) << " 2>&1 && ";
  } else if (execution_mode == PlannerExecutionMode::kPosFf) {
    cmd << "ros2 control switch_controllers -c /controller_manager"
        << " --deactivate thumb_trajectory_controller index_trajectory_controller"
        << " mrl_trajectory_controller"
        << " --activate thumb_pos_ff_controller index_pos_ff_controller mrl_pos_ff_controller"
        << " >> " << shell_quote(log_path) << " 2>&1 && ";
  }

  if (access(kPlannerBinary, X_OK) == 0) {
    cmd << shell_quote(kPlannerBinary);
  } else {
    cmd << "cargo run --release --manifest-path "
        << shell_quote(kPlannerManifest) << " --";
  }

  cmd << " --mode ros"
      << " --pointcloud-topic " << shell_quote(kPlannerPointcloudTopic)
      << " --iterations 1"
      << " --frequency-hz 1"
      << " --pc-scale 1.0";

  if (transform_mode == PlannerTransformMode::kDynamicTf) {
    cmd << " --base-transform-mode tf"
        << " --base-parent-frame world"
        << " --base-child-frame mujoco_palm_r";
  } else {
    cmd << " --base-transform-mode static";
  }

  if (execution_mode != PlannerExecutionMode::kDryRun) {
    cmd << " --publish-commands --command-backend "
        << ((execution_mode == PlannerExecutionMode::kPosFf) ? "pos_ff" : "trajectory");
  }

  cmd << " >> " << shell_quote(log_path) << " 2>&1";
  return cmd.str();
}

std::string InteractiveSimulator::make_log_path() const
{
  std::error_code error;
  fs::create_directories(kPlannerLogDir, error);

  int max_log_number = 0;
  const std::regex log_pattern(R"(^grasp_log_(\d{3,})\.txt$)");
  if (!error) {
    for (const fs::directory_entry& entry :
         fs::directory_iterator(kPlannerLogDir, error)) {
      if (error || !entry.is_regular_file()) continue;
      const std::string filename = entry.path().filename().string();
      std::smatch match;
      if (std::regex_match(filename, match, log_pattern) && match.size() == 2) {
        max_log_number = std::max(max_log_number, std::stoi(match[1].str()));
      }
    }
  }

  std::ostringstream filename;
  filename << "grasp_log_" << std::setw(3) << std::setfill('0')
           << (max_log_number + 1) << ".txt";
  return (fs::path(kPlannerLogDir) / filename.str()).string();
}

std::string InteractiveSimulator::shell_quote(const std::string& value)
{
  std::string result;
  result.reserve(value.size() + 2);
  result.push_back('\'');
  for (char c : value) {
    if (c == '\'') {
      result += "'\\''";
    } else {
      result.push_back(c);
    }
  }
  result.push_back('\'');
  return result;
}

// --- Scene Control ---

void InteractiveSimulator::rpy_to_quat(const mjtNum rpy[3], mjtNum q[4])
{
  const double cr = std::cos(rpy[0] * 0.5), sr = std::sin(rpy[0] * 0.5);
  const double cp = std::cos(rpy[1] * 0.5), sp = std::sin(rpy[1] * 0.5);
  const double cy = std::cos(rpy[2] * 0.5), sy = std::sin(rpy[2] * 0.5);
  q[0] = cr*cp*cy + sr*sp*sy;  // w
  q[1] = sr*cp*cy - cr*sp*sy;  // x
  q[2] = cr*sp*cy + sr*cp*sy;  // y
  q[3] = cr*cp*sy - sr*sp*cy;  // z
}

void InteractiveSimulator::quat_to_rpy(const mjtNum q[4], mjtNum rpy[3])
{
  // roll
  const double sinr_cosp = 2.0*(q[0]*q[1] + q[2]*q[3]);
  const double cosr_cosp = 1.0 - 2.0*(q[1]*q[1] + q[2]*q[2]);
  rpy[0] = std::atan2(sinr_cosp, cosr_cosp);
  // pitch
  const double sinp = 2.0*(q[0]*q[2] - q[3]*q[1]);
  rpy[1] = (std::abs(sinp) >= 1.0) ? std::copysign(M_PI * 0.5, sinp) : std::asin(sinp);
  // yaw
  const double siny_cosp = 2.0*(q[0]*q[3] + q[1]*q[2]);
  const double cosy_cosp = 1.0 - 2.0*(q[2]*q[2] + q[3]*q[3]);
  rpy[2] = std::atan2(siny_cosp, cosy_cosp);
}

void InteractiveSimulator::apply_body_pose(
  mjModel* m, int body_id, const mjtNum pos[3], const mjtNum quat_wxyz[4])
{
  m->body_pos[body_id*3 + 0] = pos[0];
  m->body_pos[body_id*3 + 1] = pos[1];
  m->body_pos[body_id*3 + 2] = pos[2];
  mjtNum nq[4] = {quat_wxyz[0], quat_wxyz[1], quat_wxyz[2], quat_wxyz[3]};
  mju_normalize4(nq);
  m->body_quat[body_id*4 + 0] = nq[0];
  m->body_quat[body_id*4 + 1] = nq[1];
  m->body_quat[body_id*4 + 2] = nq[2];
  m->body_quat[body_id*4 + 3] = nq[3];
}

void InteractiveSimulator::slerp_quat(
  mjtNum res[4], const mjtNum q0[4], const mjtNum q1[4], mjtNum t)
{
  // Ensure shortest-path interpolation.
  mjtNum dot = q0[0]*q1[0] + q0[1]*q1[1] + q0[2]*q1[2] + q0[3]*q1[3];
  mjtNum q1s[4] = {q1[0], q1[1], q1[2], q1[3]};
  if (dot < 0) {
    for (int i = 0; i < 4; ++i) q1s[i] = -q1s[i];
    dot = -dot;
  }
  if (dot > 1.0) dot = 1.0;

  mjtNum scale0, scale1;
  const mjtNum theta     = std::acos(dot);
  const mjtNum sin_theta = std::sin(theta);
  if (sin_theta > static_cast<mjtNum>(1e-6)) {
    scale0 = std::sin((1.0 - t) * theta) / sin_theta;
    scale1 = std::sin(t * theta)          / sin_theta;
  } else {
    scale0 = 1.0 - t;
    scale1 = t;
  }
  for (int i = 0; i < 4; ++i) res[i] = scale0 * q0[i] + scale1 * q1s[i];
  mju_normalize4(res);
}

void InteractiveSimulator::add_scene_section(mujoco::Simulate* sim)
{
  if (!mj_model_) return;

  // Look up body IDs
  hand_body_id_       = mj_name2id(mj_model_, mjOBJ_BODY, "palm_r");
  obj_body_id_        = mj_name2id(mj_model_, mjOBJ_BODY, "target_sphere_body");
  cam_body_id_        = mj_name2id(mj_model_, mjOBJ_BODY, "depth_cam_body");
  wrist_cam_body_id_  = mj_name2id(mj_model_, mjOBJ_BODY, "wrist_cam_body");

  // Read initial poses from the model
  auto read_body_pose = [&](int id, mjtNum pos[3], mjtNum rpy[3]) {
    if (id < 0) return;
    pos[0] = mj_model_->body_pos[id*3 + 0];
    pos[1] = mj_model_->body_pos[id*3 + 1];
    pos[2] = mj_model_->body_pos[id*3 + 2];
    const mjtNum* q = &mj_model_->body_quat[id*4];
    quat_to_rpy(q, rpy);
  };

  read_body_pose(hand_body_id_, scene_hand_pos_, scene_hand_rpy_);
  read_body_pose(obj_body_id_,  scene_obj_pos_,  scene_obj_rpy_);
  read_body_pose(cam_body_id_,  scene_cam_pos_,  scene_cam_rpy_);

  scene_sect_id_ = sim->ui1.nsect;

  const mjuiDef scene_def[] = {
    {mjITEM_SECTION,   "Scene Control",  mjPRESERVE, nullptr,          "", 0},
    {mjITEM_SEPARATOR, "Hand",           1,          nullptr,          "", 0},
    {mjITEM_EDITNUM,   "X [m]",          2,          &scene_hand_pos_[0], "1", 0},
    {mjITEM_EDITNUM,   "Y [m]",          2,          &scene_hand_pos_[1], "1", 0},
    {mjITEM_EDITNUM,   "Z [m]",          2,          &scene_hand_pos_[2], "1", 0},
    {mjITEM_EDITNUM,   "Roll [rad]",     2,          &scene_hand_rpy_[0], "1", 0},
    {mjITEM_EDITNUM,   "Pitch [rad]",    2,          &scene_hand_rpy_[1], "1", 0},
    {mjITEM_EDITNUM,   "Yaw [rad]",      2,          &scene_hand_rpy_[2], "1", 0},
    {mjITEM_SEPARATOR, "Object",         1,          nullptr,          "", 0},
    {mjITEM_EDITNUM,   "X [m]",          2,          &scene_obj_pos_[0],  "1", 0},
    {mjITEM_EDITNUM,   "Y [m]",          2,          &scene_obj_pos_[1],  "1", 0},
    {mjITEM_EDITNUM,   "Z [m]",          2,          &scene_obj_pos_[2],  "1", 0},
    {mjITEM_EDITNUM,   "Roll [rad]",     2,          &scene_obj_rpy_[0],  "1", 0},
    {mjITEM_EDITNUM,   "Pitch [rad]",    2,          &scene_obj_rpy_[1],  "1", 0},
    {mjITEM_EDITNUM,   "Yaw [rad]",      2,          &scene_obj_rpy_[2],  "1", 0},
    {mjITEM_SEPARATOR, "Camera",         1,          nullptr,          "", 0},
    {mjITEM_EDITNUM,   "X [m]",          2,          &scene_cam_pos_[0],  "1", 0},
    {mjITEM_EDITNUM,   "Y [m]",          2,          &scene_cam_pos_[1],  "1", 0},
    {mjITEM_EDITNUM,   "Z [m]",          2,          &scene_cam_pos_[2],  "1", 0},
    {mjITEM_EDITNUM,   "Roll [rad]",     2,          &scene_cam_rpy_[0],  "1", 0},
    {mjITEM_EDITNUM,   "Pitch [rad]",    2,          &scene_cam_rpy_[1],  "1", 0},
    {mjITEM_EDITNUM,   "Yaw [rad]",      2,          &scene_cam_rpy_[2],  "1", 0},
    {mjITEM_END,       "",               0,          nullptr,          "", 0}
  };

  mjui_add(&sim->ui1, scene_def);
  sim->ui1.sect[scene_sect_id_].state = mjSECT_CLOSED;
}

void InteractiveSimulator::handle_scene_event(
  mujoco::Simulate* /*sim*/, int itemid)
{
  auto cancel_motion = [&](MotionCmd& cmd) {
    std::lock_guard<std::mutex> lock(motion_cmd_mtx_);
    cmd.type = MotionCmd::Type::kCancel;
  };
  if (itemid >= kSceneHandX && itemid <= kSceneHandYaw) {
    scene_hand_dirty_ = true;
    cancel_motion(motion_hand_cmd_);
  } else if (itemid >= kSceneObjX && itemid <= kSceneObjYaw) {
    scene_obj_dirty_ = true;
    cancel_motion(motion_obj_cmd_);
  } else if (itemid >= kSceneCamX && itemid <= kSceneCamYaw) {
    scene_cam_dirty_ = true;
    cancel_motion(motion_cam_cmd_);
  }
}

void InteractiveSimulator::apply_scene_poses(mjModel* m, mjData* /*d*/)
{
  // Flush ROS-pending overrides into the slider arrays first
  {
    std::lock_guard<std::mutex> lock(scene_ros_mtx_);
    auto flush_pending = [&](RosPosePending& pending,
                             mjtNum pos[3], mjtNum rpy[3],
                             std::atomic<bool>& dirty) {
      if (!pending.dirty) return;
      pos[0] = static_cast<mjtNum>(pending.pos[0]);
      pos[1] = static_cast<mjtNum>(pending.pos[1]);
      pos[2] = static_cast<mjtNum>(pending.pos[2]);
      const mjtNum q[4] = {
        static_cast<mjtNum>(pending.quat_wxyz[0]),
        static_cast<mjtNum>(pending.quat_wxyz[1]),
        static_cast<mjtNum>(pending.quat_wxyz[2]),
        static_cast<mjtNum>(pending.quat_wxyz[3])
      };
      quat_to_rpy(q, rpy);
      dirty = true;
      scene_ui_sync_needed_ = true;
      pending.dirty = false;
    };
    flush_pending(ros_hand_pending_, scene_hand_pos_, scene_hand_rpy_, scene_hand_dirty_);
    flush_pending(ros_obj_pending_,  scene_obj_pos_,  scene_obj_rpy_,  scene_obj_dirty_);
    flush_pending(ros_cam_pending_,  scene_cam_pos_,  scene_cam_rpy_,  scene_cam_dirty_);
  }

  // Advance active motions (physics-thread-only state, no extra lock needed)
  advance_motions(m);

  // Apply dirty scene control poses only when no active motion for that entity.
  // Use load+conditional-store to avoid clearing a dirty flag while motion is
  // still active (which would silently drop the user's manual edit).
  if (scene_hand_dirty_.load() && hand_body_id_ >= 0 && !motion_hand_state_.active) {
    scene_hand_dirty_.store(false);
    mjtNum q[4];
    rpy_to_quat(scene_hand_rpy_, q);
    apply_body_pose(m, hand_body_id_, scene_hand_pos_, q);
  }
  if (scene_obj_dirty_.load() && obj_body_id_ >= 0 && !motion_obj_state_.active) {
    scene_obj_dirty_.store(false);
    mjtNum q[4];
    rpy_to_quat(scene_obj_rpy_, q);
    apply_body_pose(m, obj_body_id_, scene_obj_pos_, q);
  }
  if (scene_cam_dirty_.load() && cam_body_id_ >= 0 && !motion_cam_state_.active) {
    scene_cam_dirty_.store(false);
    mjtNum q[4];
    rpy_to_quat(scene_cam_rpy_, q);
    apply_body_pose(m, cam_body_id_, scene_cam_pos_, q);
  }
}

void InteractiveSimulator::sync_scene_ui(mujoco::Simulate* sim)
{
  if (!scene_ui_sync_needed_.exchange(false)) return;
  if (scene_sect_id_ < 0) return;

  // Refresh all EDITNUM items so slider display matches the updated values.
  // Items kSceneHandX..kSceneCamYaw are the 18 EDITNUM fields (non-separator items).
  for (int item = kSceneHandX; item <= kSceneCamYaw; ++item) {
    mjui_update(scene_sect_id_, item, &sim->ui1, &sim->uistate, &sim->platform_ui->mjr_context());
  }
}

void InteractiveSimulator::set_hand_pose(const double pos[3], const double quat_wxyz[4])
{
  std::lock_guard<std::mutex> lock(scene_ros_mtx_);
  ros_hand_pending_.pos[0] = pos[0];
  ros_hand_pending_.pos[1] = pos[1];
  ros_hand_pending_.pos[2] = pos[2];
  ros_hand_pending_.quat_wxyz[0] = quat_wxyz[0];
  ros_hand_pending_.quat_wxyz[1] = quat_wxyz[1];
  ros_hand_pending_.quat_wxyz[2] = quat_wxyz[2];
  ros_hand_pending_.quat_wxyz[3] = quat_wxyz[3];
  ros_hand_pending_.dirty = true;
}

void InteractiveSimulator::set_object_pose(const double pos[3], const double quat_wxyz[4])
{
  std::lock_guard<std::mutex> lock(scene_ros_mtx_);
  ros_obj_pending_.pos[0] = pos[0];
  ros_obj_pending_.pos[1] = pos[1];
  ros_obj_pending_.pos[2] = pos[2];
  ros_obj_pending_.quat_wxyz[0] = quat_wxyz[0];
  ros_obj_pending_.quat_wxyz[1] = quat_wxyz[1];
  ros_obj_pending_.quat_wxyz[2] = quat_wxyz[2];
  ros_obj_pending_.quat_wxyz[3] = quat_wxyz[3];
  ros_obj_pending_.dirty = true;
}

void InteractiveSimulator::set_camera_pose(const double pos[3], const double quat_wxyz[4])
{
  std::lock_guard<std::mutex> lock(scene_ros_mtx_);
  ros_cam_pending_.pos[0] = pos[0];
  ros_cam_pending_.pos[1] = pos[1];
  ros_cam_pending_.pos[2] = pos[2];
  ros_cam_pending_.quat_wxyz[0] = quat_wxyz[0];
  ros_cam_pending_.quat_wxyz[1] = quat_wxyz[1];
  ros_cam_pending_.quat_wxyz[2] = quat_wxyz[2];
  ros_cam_pending_.quat_wxyz[3] = quat_wxyz[3];
  ros_cam_pending_.dirty = true;
}

void InteractiveSimulator::get_hand_pose(double pos[3], double quat_wxyz[4]) const
{
  std::lock_guard<std::mutex> lock(scene_ros_mtx_);
  for (int i = 0; i < 3; ++i) pos[i] = static_cast<double>(scene_hand_pos_[i]);
  mjtNum q[4];
  rpy_to_quat(scene_hand_rpy_, q);
  for (int i = 0; i < 4; ++i) quat_wxyz[i] = static_cast<double>(q[i]);
}

void InteractiveSimulator::get_object_pose(double pos[3], double quat_wxyz[4]) const
{
  std::lock_guard<std::mutex> lock(scene_ros_mtx_);
  for (int i = 0; i < 3; ++i) pos[i] = static_cast<double>(scene_obj_pos_[i]);
  mjtNum q[4];
  rpy_to_quat(scene_obj_rpy_, q);
  for (int i = 0; i < 4; ++i) quat_wxyz[i] = static_cast<double>(q[i]);
}

void InteractiveSimulator::get_camera_pose(double pos[3], double quat_wxyz[4]) const
{
  std::lock_guard<std::mutex> lock(scene_ros_mtx_);
  for (int i = 0; i < 3; ++i) pos[i] = static_cast<double>(scene_cam_pos_[i]);
  mjtNum q[4];
  rpy_to_quat(scene_cam_rpy_, q);
  for (int i = 0; i < 4; ++i) quat_wxyz[i] = static_cast<double>(q[i]);
}

void InteractiveSimulator::get_imu_data(
  double ang_vel[3],
  double lin_acc[3],
  double mag_field[3],
  double orientation_wxyz[4],
  double& sim_time) const
{
  std::lock_guard<std::mutex> lock(sim_mtx_);
  for (int i = 0; i < 3; ++i) {
    ang_vel[i]   = imu_ang_vel_[i];
    lin_acc[i]   = imu_lin_acc_[i];
    mag_field[i] = imu_mag_field_[i];
  }
  for (int i = 0; i < 4; ++i) orientation_wxyz[i] = imu_orientation_wxyz_[i];
  sim_time = sim_time_;
}

void InteractiveSimulator::get_wrist_cam_imu_data(
  double ang_vel[3],
  double lin_acc[3],
  double mag_field[3],
  double orientation_wxyz[4]) const
{
  std::lock_guard<std::mutex> lock(sim_mtx_);
  for (int i = 0; i < 3; ++i) {
    ang_vel[i]   = imu2_ang_vel_[i];
    lin_acc[i]   = imu2_lin_acc_[i];
    mag_field[i] = imu2_mag_field_[i];
  }
  for (int i = 0; i < 4; ++i) orientation_wxyz[i] = imu2_orientation_wxyz_[i];
}

void InteractiveSimulator::request_hand_move(
  const double pos[3], const double quat_wxyz[4], double duration_s)
{
  std::lock_guard<std::mutex> lock(motion_cmd_mtx_);
  for (int i = 0; i < 3; ++i) motion_hand_cmd_.tgt_pos[i]  = static_cast<mjtNum>(pos[i]);
  for (int i = 0; i < 4; ++i) motion_hand_cmd_.tgt_quat[i] = static_cast<mjtNum>(quat_wxyz[i]);
  motion_hand_cmd_.duration = std::max(0.001, duration_s);
  motion_hand_cmd_.type = MotionCmd::Type::kStart;
}

void InteractiveSimulator::request_object_move(
  const double pos[3], const double quat_wxyz[4], double duration_s)
{
  std::lock_guard<std::mutex> lock(motion_cmd_mtx_);
  for (int i = 0; i < 3; ++i) motion_obj_cmd_.tgt_pos[i]  = static_cast<mjtNum>(pos[i]);
  for (int i = 0; i < 4; ++i) motion_obj_cmd_.tgt_quat[i] = static_cast<mjtNum>(quat_wxyz[i]);
  motion_obj_cmd_.duration = std::max(0.001, duration_s);
  motion_obj_cmd_.type = MotionCmd::Type::kStart;
}

void InteractiveSimulator::request_camera_move(
  const double pos[3], const double quat_wxyz[4], double duration_s)
{
  std::lock_guard<std::mutex> lock(motion_cmd_mtx_);
  for (int i = 0; i < 3; ++i) motion_cam_cmd_.tgt_pos[i]  = static_cast<mjtNum>(pos[i]);
  for (int i = 0; i < 4; ++i) motion_cam_cmd_.tgt_quat[i] = static_cast<mjtNum>(quat_wxyz[i]);
  motion_cam_cmd_.duration = std::max(0.001, duration_s);
  motion_cam_cmd_.type = MotionCmd::Type::kStart;
}

// --- Motion Control ---

void InteractiveSimulator::add_motion_section(mujoco::Simulate* sim)
{
  if (!mj_model_) return;

  // Initialise target fields to match current scene poses (set in add_scene_section).
  for (int i = 0; i < 3; ++i) {
    motion_hand_tgt_pos_[i] = scene_hand_pos_[i];
    motion_hand_tgt_rpy_[i] = scene_hand_rpy_[i];
    motion_obj_tgt_pos_[i]  = scene_obj_pos_[i];
    motion_obj_tgt_rpy_[i]  = scene_obj_rpy_[i];
    motion_cam_tgt_pos_[i]  = scene_cam_pos_[i];
    motion_cam_tgt_rpy_[i]  = scene_cam_rpy_[i];
  }

  motion_sect_id_ = sim->ui1.nsect;

  const mjuiDef motion_def[] = {
    {mjITEM_SECTION,   "Motion Control",  mjPRESERVE, nullptr,                   "", 0},
    {mjITEM_EDITNUM,   "Duration [s]",    2,          &motion_duration_,          "1", 0},
    {mjITEM_SEPARATOR, "Hand Target",     1,          nullptr,                   "", 0},
    {mjITEM_EDITNUM,   "X [m]",           2,          &motion_hand_tgt_pos_[0],  "1", 0},
    {mjITEM_EDITNUM,   "Y [m]",           2,          &motion_hand_tgt_pos_[1],  "1", 0},
    {mjITEM_EDITNUM,   "Z [m]",           2,          &motion_hand_tgt_pos_[2],  "1", 0},
    {mjITEM_EDITNUM,   "Roll [rad]",      2,          &motion_hand_tgt_rpy_[0],  "1", 0},
    {mjITEM_EDITNUM,   "Pitch [rad]",     2,          &motion_hand_tgt_rpy_[1],  "1", 0},
    {mjITEM_EDITNUM,   "Yaw [rad]",       2,          &motion_hand_tgt_rpy_[2],  "1", 0},
    {mjITEM_BUTTON,    "Move Hand",       2,          nullptr,                   "", 0},
    {mjITEM_SEPARATOR, "Object Target",   1,          nullptr,                   "", 0},
    {mjITEM_EDITNUM,   "X [m]",           2,          &motion_obj_tgt_pos_[0],   "1", 0},
    {mjITEM_EDITNUM,   "Y [m]",           2,          &motion_obj_tgt_pos_[1],   "1", 0},
    {mjITEM_EDITNUM,   "Z [m]",           2,          &motion_obj_tgt_pos_[2],   "1", 0},
    {mjITEM_EDITNUM,   "Roll [rad]",      2,          &motion_obj_tgt_rpy_[0],   "1", 0},
    {mjITEM_EDITNUM,   "Pitch [rad]",     2,          &motion_obj_tgt_rpy_[1],   "1", 0},
    {mjITEM_EDITNUM,   "Yaw [rad]",       2,          &motion_obj_tgt_rpy_[2],   "1", 0},
    {mjITEM_BUTTON,    "Move Object",     2,          nullptr,                   "", 0},
    {mjITEM_SEPARATOR, "Camera Target",   1,          nullptr,                   "", 0},
    {mjITEM_EDITNUM,   "X [m]",           2,          &motion_cam_tgt_pos_[0],   "1", 0},
    {mjITEM_EDITNUM,   "Y [m]",           2,          &motion_cam_tgt_pos_[1],   "1", 0},
    {mjITEM_EDITNUM,   "Z [m]",           2,          &motion_cam_tgt_pos_[2],   "1", 0},
    {mjITEM_EDITNUM,   "Roll [rad]",      2,          &motion_cam_tgt_rpy_[0],   "1", 0},
    {mjITEM_EDITNUM,   "Pitch [rad]",     2,          &motion_cam_tgt_rpy_[1],   "1", 0},
    {mjITEM_EDITNUM,   "Yaw [rad]",       2,          &motion_cam_tgt_rpy_[2],   "1", 0},
    {mjITEM_BUTTON,    "Move Camera",     2,          nullptr,                   "", 0},
    {mjITEM_END,       "",                0,          nullptr,                   "", 0}
  };

  mjui_add(&sim->ui1, motion_def);
  sim->ui1.sect[motion_sect_id_].state = mjSECT_CLOSED;
}

void InteractiveSimulator::handle_motion_event(
  mujoco::Simulate* /*sim*/, int itemid)
{
  auto post_start = [&](MotionCmd& cmd, mjtNum tgt_pos[3], mjtNum tgt_rpy[3]) {
    mjtNum tgt_quat[4];
    rpy_to_quat(tgt_rpy, tgt_quat);
    std::lock_guard<std::mutex> lock(motion_cmd_mtx_);
    for (int i = 0; i < 3; ++i) cmd.tgt_pos[i] = tgt_pos[i];
    for (int i = 0; i < 4; ++i) cmd.tgt_quat[i] = tgt_quat[i];
    cmd.duration = std::max(0.001, static_cast<double>(motion_duration_));
    cmd.type = MotionCmd::Type::kStart;
  };

  if (itemid == kMotionHandMove) {
    post_start(motion_hand_cmd_, motion_hand_tgt_pos_, motion_hand_tgt_rpy_);
  } else if (itemid == kMotionObjMove) {
    post_start(motion_obj_cmd_, motion_obj_tgt_pos_, motion_obj_tgt_rpy_);
  } else if (itemid == kMotionCamMove) {
    post_start(motion_cam_cmd_, motion_cam_tgt_pos_, motion_cam_tgt_rpy_);
  }
}

void InteractiveSimulator::advance_motions(mjModel* m)
{
  // Consume pending commands (render→physics mailbox).
  {
    std::lock_guard<std::mutex> lock(motion_cmd_mtx_);
    auto process_cmd = [&](MotionCmd& cmd, MotionState& ms, int body_id) {
      if (cmd.type == MotionCmd::Type::kNone) return;
      if (cmd.type == MotionCmd::Type::kCancel) {
        ms.active = false;
      } else if (cmd.type == MotionCmd::Type::kStart && body_id >= 0) {
        // Capture current body pose as motion source (ground truth from model).
        for (int i = 0; i < 3; ++i) ms.src_pos[i]  = m->body_pos[body_id*3 + i];
        for (int i = 0; i < 4; ++i) ms.src_quat[i] = m->body_quat[body_id*4 + i];
        mju_normalize4(ms.src_quat);
        for (int i = 0; i < 3; ++i) ms.tgt_pos[i]  = cmd.tgt_pos[i];
        for (int i = 0; i < 4; ++i) ms.tgt_quat[i] = cmd.tgt_quat[i];
        mju_normalize4(ms.tgt_quat);
        ms.duration   = cmd.duration;
        ms.start_time = std::chrono::steady_clock::now();
        ms.active     = true;
      }
      cmd.type = MotionCmd::Type::kNone;
    };
    process_cmd(motion_hand_cmd_, motion_hand_state_, hand_body_id_);
    process_cmd(motion_obj_cmd_,  motion_obj_state_,  obj_body_id_);
    process_cmd(motion_cam_cmd_,  motion_cam_state_,  cam_body_id_);
  }

  // Advance each active motion.
  const auto now = std::chrono::steady_clock::now();

  auto advance_one = [&](MotionState& ms, int body_id,
                         mjtNum pos[3], mjtNum rpy[3]) {
    if (!ms.active || body_id < 0) return;

    double t = std::chrono::duration<double>(now - ms.start_time).count()
               / ms.duration;
    if (t > 1.0) t = 1.0;

    // Smoothstep ease-in/out (uses only MuJoCo built-in math below)
    const mjtNum ts = static_cast<mjtNum>(t * t * (3.0 - 2.0 * t));

    // LERP position using MuJoCo scalar math
    mjtNum cur_pos[3];
    for (int i = 0; i < 3; ++i)
      cur_pos[i] = ms.src_pos[i] + ts * (ms.tgt_pos[i] - ms.src_pos[i]);

    // SLERP rotation (shortest path, with fallback to LERP for near-identical quats)
    mjtNum cur_quat[4];
    slerp_quat(cur_quat, ms.src_quat, ms.tgt_quat, ts);

    apply_body_pose(m, body_id, cur_pos, cur_quat);

    // Mirror interpolated pose into scene arrays so Scene Control stays in sync.
    for (int i = 0; i < 3; ++i) pos[i] = cur_pos[i];
    quat_to_rpy(cur_quat, rpy);
    scene_ui_sync_needed_ = true;

    if (t >= 1.0) {
      ms.active = false;
    }
  };

  advance_one(motion_hand_state_, hand_body_id_, scene_hand_pos_, scene_hand_rpy_);
  advance_one(motion_obj_state_,  obj_body_id_,  scene_obj_pos_,  scene_obj_rpy_);
  advance_one(motion_cam_state_,  cam_body_id_,  scene_cam_pos_,  scene_cam_rpy_);
}

}  // namespace mia_hand_mujoco
