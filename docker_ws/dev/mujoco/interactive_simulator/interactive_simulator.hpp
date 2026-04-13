#ifndef MIA_HAND_MUJOCO_INTERACTIVE_SIMULATOR_HPP
#define MIA_HAND_MUJOCO_INTERACTIVE_SIMULATOR_HPP

#include <atomic>
#include <chrono>
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

  // Scene pose control (thread-safe; called from ROS callbacks or scripts)
  void set_hand_pose(const double pos[3], const double quat_wxyz[4]);
  void set_object_pose(const double pos[3], const double quat_wxyz[4]);
  void set_camera_pose(const double pos[3], const double quat_wxyz[4]);
  void get_hand_pose(double pos[3], double quat_wxyz[4]) const;
  void get_object_pose(double pos[3], double quat_wxyz[4]) const;
  void get_camera_pose(double pos[3], double quat_wxyz[4]) const;

  // Smooth motion: move the named entity to target pose over duration_s seconds.
  // May be called from any thread (e.g. a ROS subscription callback).
  void request_hand_move(const double pos[3], const double quat_wxyz[4], double duration_s);
  void request_object_move(const double pos[3], const double quat_wxyz[4], double duration_s);
  void request_camera_move(const double pos[3], const double quat_wxyz[4], double duration_s);

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

  // item indices within the "Scene Control" section (SECTION header not counted)
  // Hand: separator(0), X(1), Y(2), Z(3), Roll(4), Pitch(5), Yaw(6)
  // Object: separator(7), X(8), Y(9), Z(10), Roll(11), Pitch(12), Yaw(13)
  // Camera: separator(14), X(15), Y(16), Z(17), Roll(18), Pitch(19), Yaw(20)
  static constexpr int kSceneHandX   =  1;
  static constexpr int kSceneHandYaw =  6;
  static constexpr int kSceneObjX    =  8;
  static constexpr int kSceneObjYaw  = 13;
  static constexpr int kSceneCamX    = 15;
  static constexpr int kSceneCamYaw  = 20;

  // item indices within the "Motion Control" section (SECTION header not counted)
  // Duration(0), HandSep(1), X(2)..Yaw(7), MoveHand(8),
  // ObjSep(9), X(10)..Yaw(15), MoveObj(16),
  // CamSep(17), X(18)..Yaw(23), MoveCam(24)
  static constexpr int kMotionDuration  =  0;
  static constexpr int kMotionHandX     =  2;
  static constexpr int kMotionHandYaw   =  7;
  static constexpr int kMotionHandMove  =  8;
  static constexpr int kMotionObjX      = 10;
  static constexpr int kMotionObjYaw    = 15;
  static constexpr int kMotionObjMove   = 16;
  static constexpr int kMotionCamX      = 18;
  static constexpr int kMotionCamYaw    = 23;
  static constexpr int kMotionCamMove   = 24;

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

  // Scene Control UI
  void add_scene_section(mujoco::Simulate* sim);
  void handle_scene_event(mujoco::Simulate* sim, int itemid);
  void apply_scene_poses(mjModel* m, mjData* d);
  void sync_scene_ui(mujoco::Simulate* sim);
  static void rpy_to_quat(const mjtNum rpy[3], mjtNum q_wxyz[4]);
  static void quat_to_rpy(const mjtNum q_wxyz[4], mjtNum rpy[3]);
  static void apply_body_pose(mjModel* m, int body_id,
                              const mjtNum pos[3], const mjtNum quat_wxyz[4]);
  static void slerp_quat(mjtNum res[4], const mjtNum q0[4],
                         const mjtNum q1[4], mjtNum t);

  // Motion Control UI
  // Command posted from render thread; consumed on physics thread under sim_->mtx.
  struct MotionCmd {
    mjtNum tgt_pos[3];
    mjtNum tgt_quat[4];  // wxyz (pre-converted from RPY)
    double duration;
    enum class Type { kNone, kStart, kCancel } type = Type::kNone;
  };
  // State is physics-thread-only; accessed only inside apply_scene_poses / advance_motions.
  struct MotionState {
    mjtNum src_pos[3]  = {0, 0, 0};
    mjtNum src_quat[4] = {1, 0, 0, 0};  // wxyz
    mjtNum tgt_pos[3]  = {0, 0, 0};
    mjtNum tgt_quat[4] = {1, 0, 0, 0};  // wxyz
    std::chrono::steady_clock::time_point start_time;
    double duration = 1.0;
    bool active = false;
  };

  void add_motion_section(mujoco::Simulate* sim);
  void handle_motion_event(mujoco::Simulate* sim, int itemid);
  void advance_motions(mjModel* m);

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

  // Scene Control UI state (render thread writes pdata; physics thread reads)
  mjtNum scene_hand_pos_[3];
  mjtNum scene_hand_rpy_[3];
  mjtNum scene_obj_pos_[3];
  mjtNum scene_obj_rpy_[3];
  mjtNum scene_cam_pos_[3];
  mjtNum scene_cam_rpy_[3];

  std::atomic<bool> scene_hand_dirty_;
  std::atomic<bool> scene_obj_dirty_;
  std::atomic<bool> scene_cam_dirty_;
  std::atomic<bool> scene_ui_sync_needed_;

  // Pending pose from ROS callbacks (locked by scene_ros_mtx_)
  struct RosPosePending {
    double pos[3];
    double quat_wxyz[4];
    bool dirty;
  };
  mutable std::mutex scene_ros_mtx_;
  RosPosePending ros_hand_pending_;
  RosPosePending ros_obj_pending_;
  RosPosePending ros_cam_pending_;

  // MuJoCo body IDs (set in add_scene_section after model load)
  int hand_body_id_;
  int obj_body_id_;
  int cam_body_id_;
  int scene_sect_id_;

  // Motion Control UI fields (render-thread-owned pdata)
  mjtNum motion_duration_;
  mjtNum motion_hand_tgt_pos_[3];
  mjtNum motion_hand_tgt_rpy_[3];
  mjtNum motion_obj_tgt_pos_[3];
  mjtNum motion_obj_tgt_rpy_[3];
  mjtNum motion_cam_tgt_pos_[3];
  mjtNum motion_cam_tgt_rpy_[3];

  // Motion command mailbox: render thread writes, physics thread consumes.
  std::mutex motion_cmd_mtx_;
  MotionCmd motion_hand_cmd_;
  MotionCmd motion_obj_cmd_;
  MotionCmd motion_cam_cmd_;

  // Motion execution state: physics-thread only, no extra lock needed.
  MotionState motion_hand_state_;
  MotionState motion_obj_state_;
  MotionState motion_cam_state_;

  int motion_sect_id_;
};
}  // namespace mia_hand_mujoco

#endif  // MIA_HAND_MUJOCO_INTERACTIVE_SIMULATOR_HPP
