#ifndef MIA_HAND_MUJOCO_INTERACTIVE_SIMULATOR_HPP
#define MIA_HAND_MUJOCO_INTERACTIVE_SIMULATOR_HPP

#include <atomic>
#include <functional>
#include <future>
#include <mutex>
#include <string>
#include <thread>

#include "mujoco/mujoco.h"
#include "simulate_src/simulate.h"

namespace mia_hand_mujoco
{
class InteractiveSimulator
{
public:
  InteractiveSimulator(InteractiveSimulator&) = delete;
  InteractiveSimulator& operator=(InteractiveSimulator&) = delete;

  static InteractiveSimulator& get_instance();

  const char* get_error_msg();

  static bool simulate(
    const char* model, std::promise<bool>&& sim_start_ok = std::promise<bool>());

  void stop_simulation();

  void read_jnt_vel(double& thumb_vel, double& index_vel, double& mrl_vel);
  void read_jnt_pos(double& thumb_pos, double& index_pos, double& mrl_pos);
  void set_jnt_pos(uint_fast8_t jnt, double pos);
  void set_jnt_vel(uint_fast8_t jnt, double vel);
  void stop_jnt(uint_fast8_t jnt);

private:
  enum class PlannerTransformMode
  {
    kDynamicTf = 0,
    kLegacyStatic = 1,
  };

  enum class PlannerExecutionMode
  {
    kDryRun = 0,
    kTrajectory = 1,
    kPosFf = 2,
  };

  // item indices within the "Grasp Planner" section (SECTION header not counted)
  static constexpr int kPlannerItemTransformMode = 1;
  static constexpr int kPlannerItemExecutionMode = 3;
  static constexpr int kPlannerItemRunPlanner    = 5;
  static constexpr int kPlannerItemStatus        = 7;

  InteractiveSimulator();

  static void control_cb(const mjModel* model, mjData* data);
  void control_cb_impl(const mjModel* model, mjData* data);

  bool simulate_impl(const char* model, std::promise<bool>&& sim_start_ok);
  void physics_thread_fn(const char* model_path, std::promise<bool>&& sim_start_ok);
  bool get_plugin_instance(const mjModel* p_mjm);

  // Grasp Planner UI
  void add_custom_section(mujoco::Simulate* sim);
  void handle_custom_event(mujoco::Simulate* sim, int sectionid, int itemid);
  void sync_custom_status(mujoco::Simulate* sim);
  void set_status(const std::string& status);
  void launch_planner(PlannerTransformMode transform_mode,
                      PlannerExecutionMode execution_mode);
  void run_planner_worker(PlannerTransformMode transform_mode,
                          PlannerExecutionMode execution_mode);
  std::string build_planner_command(PlannerTransformMode transform_mode,
                                    PlannerExecutionMode execution_mode,
                                    const std::string& log_path) const;
  std::string make_log_path() const;
  static std::string shell_quote(const std::string& value);

  mjModel* mj_model_;
  mjData* mj_data_;

  mjvCamera mjv_camera_;
  mjvOption mjv_options_;
  mjvPerturb mjv_pert_;

  std::unique_ptr<mujoco::Simulate> sim_;

  std::thread physics_thread_;

  char err_msg_[256];

  int plugin_instance_;

  double jnt_vel_state_[3];
  double jnt_vel_cmd_[3];
  double jnt_pos_state_[3];
  double jnt_pos_cmd_[3];

  std::mutex sim_mtx_;

  // Grasp Planner UI state
  int planner_sect_id_;
  int planner_transform_mode_value_;
  int planner_execution_mode_value_;
  std::atomic<bool> planner_running_;
  std::mutex planner_status_mtx_;
  std::string planner_status_pending_;
  bool planner_status_dirty_;
};
}  // namespace mia_hand_mujoco

#endif  // MIA_HAND_MUJOCO_INTERACTIVE_SIMULATOR_HPP
