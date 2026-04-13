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
  planner_status_dirty_(false)
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

  // Wire Grasp Planner UI hooks
  sim_->custom_section_init = [this](mujoco::Simulate* sim) {
    add_custom_section(sim);
  };
  sim_->custom_section_event = [this](mujoco::Simulate* sim, int sec, int item) {
    handle_custom_event(sim, sec, item);
  };
  sim_->custom_sync_fn = [this](mujoco::Simulate* sim) {
    sync_custom_status(sim);
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

}  // namespace mia_hand_mujoco
