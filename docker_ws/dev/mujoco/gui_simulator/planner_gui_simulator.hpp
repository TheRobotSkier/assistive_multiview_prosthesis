#ifndef MIA_HAND_MUJOCO_PLANNER_GUI_SIMULATOR_HPP
#define MIA_HAND_MUJOCO_PLANNER_GUI_SIMULATOR_HPP

#include <atomic>
#include <future>
#include <mutex>
#include <string>

#include <GLFW/glfw3.h>

#include "mujoco/mujoco.h"

namespace mia_hand_mujoco
{
class PlannerGuiSimulator
{
public:
  PlannerGuiSimulator(PlannerGuiSimulator&) = delete;
  PlannerGuiSimulator& operator=(PlannerGuiSimulator&) = delete;

  static PlannerGuiSimulator& get_instance();

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

  static constexpr int kUiRectMain = 0;
  static constexpr int kUiRectPanel = 1;
  static constexpr int kUiRectViewport = 2;
  static constexpr int kUiSectionPlanner = 0;
  static constexpr int kUiItemTransformMode = 1;
  static constexpr int kUiItemExecutionMode = 3;
  static constexpr int kUiItemRunPlanner = 5;
  static constexpr int kUiItemStatus = 7;

  PlannerGuiSimulator();

  static void keyboard_cb(
    GLFWwindow* window, int key, int scancode, int act, int mods);
  static void mouse_move_cb(GLFWwindow* window, double xpos, double ypos);
  static void mouse_button_cb(GLFWwindow* window, int button, int act, int mods);
  static void scroll_cb(GLFWwindow* window, double x_offset, double y_offset);
  static void control_cb(const mjModel* model, mjData* data);

  void keyboard_cb_impl(
    GLFWwindow* window, int key, int scancode, int act, int mods);
  void mouse_move_cb_impl(GLFWwindow* window, double xpos, double ypos);
  void mouse_button_cb_impl(GLFWwindow* window, int button, int act, int mods);
  void scroll_cb_impl(GLFWwindow* window, double x_offset, double y_offset);
  void control_cb_impl(const mjModel* model, mjData* data);
  bool simulate_impl(const char* model, std::promise<bool>&& sim_start_ok);
  bool get_plugin_instance(const mjModel* p_mjm);

  void init_ui();
  void refresh_ui_layout();
  void sync_status_to_ui();
  bool dispatch_ui_event(mjtEvent event_type, int button, int key, double x, double y,
                         double scroll_x, double scroll_y, int mods);
  void handle_ui_item(mjuiItem* item);
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
  static const char* transform_mode_label(PlannerTransformMode mode);
  static const char* execution_mode_label(PlannerExecutionMode mode);

  mjModel* mj_model_;
  mjData* mj_data_;

  mjvCamera mjv_camera_;
  mjvOption mjv_options_;
  mjvScene mjv_scene_;

  mjrContext mjr_context_;
  mjrRect mjr_viewport_;

  mjUI ui_;
  mjuiState ui_state_;
  bool ui_initialized_;
  int ui_last_width_;
  int ui_last_height_;
  int planner_transform_mode_value_;
  int planner_execution_mode_value_;
  bool ui_mouse_capture_;

  mjtMouse mjt_action_;
  mjtNum mjt_sim_t0_;
  const mjtNum mjt_sim_dt_;

  bool mouse_btn_left_pressed_;
  bool mouse_btn_right_pressed_;
  bool mouse_btn_mid_pressed_;

  double mouse_last_x_;
  double mouse_last_y_;

  double mouse_dx_;
  double mouse_dy_;

  GLFWwindow* window_;

  int window_w_;
  int window_h_;

  bool shift_key_pressed_;

  char err_msg_[256];

  int plugin_instance_;

  double jnt_vel_state_[3];
  double jnt_vel_cmd_[3];

  double jnt_pos_state_[3];
  double jnt_pos_cmd_[3];

  std::mutex sim_mtx_;

  std::atomic<bool> planner_running_;
  std::mutex planner_status_mtx_;
  std::string planner_status_pending_;
  bool planner_status_dirty_;
};
}  // namespace mia_hand_mujoco

#endif  // MIA_HAND_MUJOCO_PLANNER_GUI_SIMULATOR_HPP
