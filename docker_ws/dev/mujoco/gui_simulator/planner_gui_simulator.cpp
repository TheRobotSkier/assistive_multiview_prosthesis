#include "planner_gui_simulator.hpp"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <sys/wait.h>
#include <unistd.h>

#include "mia_hand_mujoco/plugin/index_thumb_actuator.h"

namespace
{
int map_glfw_button(int button)
{
  switch (button)
  {
    case GLFW_MOUSE_BUTTON_LEFT:
      return mjBUTTON_LEFT;
    case GLFW_MOUSE_BUTTON_RIGHT:
      return mjBUTTON_RIGHT;
    case GLFW_MOUSE_BUTTON_MIDDLE:
      return mjBUTTON_MIDDLE;
    default:
      return mjBUTTON_NONE;
  }
}
}  // namespace

namespace mia_hand_mujoco
{
PlannerGuiSimulator& PlannerGuiSimulator::get_instance()
{
  static PlannerGuiSimulator sim_instance;
  return sim_instance;
}

const char* PlannerGuiSimulator::get_error_msg()
{
  return err_msg_;
}

bool PlannerGuiSimulator::simulate(
  const char* model, std::promise<bool>&& sim_start_ok)
{
  if (mjVERSION_HEADER != mj_version())
  {
    mju_warning("MuJoCo API and library version do not match.");
  }

  return get_instance().simulate_impl(model, std::move(sim_start_ok));
}

void PlannerGuiSimulator::stop_simulation()
{
  if (window_ != nullptr)
  {
    glfwSetWindowShouldClose(window_, GLFW_TRUE);
  }
}

void PlannerGuiSimulator::read_jnt_vel(
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

void PlannerGuiSimulator::read_jnt_pos(
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

void PlannerGuiSimulator::set_jnt_pos(uint_fast8_t jnt, double pos)
{
  std::lock_guard<std::mutex> lock(sim_mtx_);
  jnt_pos_cmd_[jnt] = pos;
}

void PlannerGuiSimulator::set_jnt_vel(uint_fast8_t jnt, double vel)
{
  std::lock_guard<std::mutex> lock(sim_mtx_);
  jnt_vel_cmd_[jnt] = vel;
}

void PlannerGuiSimulator::stop_jnt(uint_fast8_t jnt)
{
  std::lock_guard<std::mutex> lock(sim_mtx_);
  jnt_pos_cmd_[jnt] = mj_data_->qpos[jnt];
}

bool PlannerGuiSimulator::consume_planner_request()
{
  return planner_request_pending_.exchange(false);
}

void PlannerGuiSimulator::report_planner_result(bool success, const std::string& message)
{
  if (success)
  {
    set_status(std::string("OK: ") + message);
  }
  else
  {
    set_status(std::string("FAIL: ") + message);
  }
  planner_running_.store(false);
}

PlannerGuiSimulator::PlannerGuiSimulator()
: mj_model_(nullptr),
  mj_data_(nullptr),
  mjr_viewport_({0, 0, 0, 0}),
  ui_initialized_(false),
  ui_last_width_(0),
  ui_last_height_(0),
  ui_mouse_capture_(false),
  mjt_sim_t0_(0.0),
  mjt_sim_dt_(1.0 / 60.0),
  mouse_btn_left_pressed_(false),
  mouse_btn_right_pressed_(false),
  mouse_btn_mid_pressed_(false),
  mouse_last_x_(0.0),
  mouse_last_y_(0.0),
  mouse_dx_(0.0),
  mouse_dy_(0.0),
  window_(nullptr),
  window_w_(0),
  window_h_(0),
  shift_key_pressed_(false),
  plugin_instance_(-1),
  planner_running_(false),
  planner_request_pending_(false),
  planner_status_pending_("Idle"),
  planner_status_dirty_(true)
{
  err_msg_[0] = '\0';

  for (double& value : jnt_vel_state_)
  {
    value = 0.0;
  }
  for (double& value : jnt_vel_cmd_)
  {
    value = 0.0;
  }
  for (double& value : jnt_pos_state_)
  {
    value = 0.0;
  }
  for (double& value : jnt_pos_cmd_)
  {
    value = 0.0;
  }
}

void PlannerGuiSimulator::keyboard_cb(
  GLFWwindow* window, int key, int scancode, int act, int mods)
{
  get_instance().keyboard_cb_impl(window, key, scancode, act, mods);
}

void PlannerGuiSimulator::mouse_move_cb(GLFWwindow* window, double xpos, double ypos)
{
  get_instance().mouse_move_cb_impl(window, xpos, ypos);
}

void PlannerGuiSimulator::mouse_button_cb(
  GLFWwindow* window, int button, int act, int mods)
{
  get_instance().mouse_button_cb_impl(window, button, act, mods);
}

void PlannerGuiSimulator::scroll_cb(GLFWwindow* window, double x_offset, double y_offset)
{
  get_instance().scroll_cb_impl(window, x_offset, y_offset);
}

void PlannerGuiSimulator::control_cb(const mjModel* model, mjData* data)
{
  get_instance().control_cb_impl(model, data);
}

void PlannerGuiSimulator::keyboard_cb_impl(
  GLFWwindow* window, int key, int /* scancode */, int act, int mods)
{
  if (act == GLFW_PRESS || act == GLFW_REPEAT)
  {
    if (dispatch_ui_event(mjEVENT_KEY, mjBUTTON_NONE, key, 0.0, 0.0, 0.0, 0.0, mods))
    {
      return;
    }
  }

  if ((GLFW_PRESS == act) && (GLFW_KEY_BACKSPACE == key))
  {
    mj_resetData(mj_model_, mj_data_);
    mj_forward(mj_model_, mj_data_);
  }

  (void)window;
}

void PlannerGuiSimulator::mouse_button_cb_impl(
  GLFWwindow* window, int button, int act, int mods)
{
  mouse_btn_left_pressed_ =
    glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
  mouse_btn_mid_pressed_ =
    glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
  mouse_btn_right_pressed_ =
    glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;

  glfwGetCursorPos(window, &mouse_last_x_, &mouse_last_y_);

  int window_width = 0;
  int window_height = 0;
  int fb_width = 0;
  int fb_height = 0;
  glfwGetWindowSize(window, &window_width, &window_height);
  glfwGetFramebufferSize(window, &fb_width, &fb_height);

  if (window_width > 0 && window_height > 0 && fb_width > 0 && fb_height > 0)
  {
    const double scale_x = static_cast<double>(fb_width) / static_cast<double>(window_width);
    const double scale_y = static_cast<double>(fb_height) / static_cast<double>(window_height);
    const int fb_x = static_cast<int>(mouse_last_x_ * scale_x);
    const int fb_y = static_cast<int>((static_cast<double>(window_height) - mouse_last_y_) * scale_y);
    ui_state_.mouserect =
      mjr_findRect(fb_x, fb_y, ui_state_.nrect - 1, ui_state_.rect + 1) + 1;
  }
  else
  {
    ui_state_.mouserect = 0;
  }

  const mjtEvent event_type =
    (act == GLFW_PRESS) ? mjEVENT_PRESS : mjEVENT_RELEASE;
  const int mj_button = map_glfw_button(button);

  if (act == GLFW_PRESS && ui_state_.mouserect)
  {
    ui_state_.dragrect = ui_state_.mouserect;
    ui_state_.dragbutton = mj_button;
  }

  const bool ui_consumed = dispatch_ui_event(
    event_type, mj_button, 0, mouse_last_x_, mouse_last_y_, 0.0, 0.0, mods);

  if (act == GLFW_RELEASE)
  {
    ui_state_.dragrect = 0;
    ui_state_.dragbutton = 0;
    ui_mouse_capture_ = false;
  }
  else if (act == GLFW_PRESS && ui_consumed)
  {
    ui_mouse_capture_ = true;
  }
}

void PlannerGuiSimulator::mouse_move_cb_impl(
  GLFWwindow* window, double xpos, double ypos)
{
  const bool ui_consumed = dispatch_ui_event(
    mjEVENT_MOVE, mjBUTTON_NONE, 0, xpos, ypos, 0.0, 0.0, 0);

  if (ui_consumed || ui_mouse_capture_)
  {
    mouse_last_x_ = xpos;
    mouse_last_y_ = ypos;
    return;
  }

  if (mouse_btn_right_pressed_ || mouse_btn_mid_pressed_ || mouse_btn_left_pressed_)
  {
    mouse_dx_ = xpos - mouse_last_x_;
    mouse_dy_ = ypos - mouse_last_y_;

    mouse_last_x_ = xpos;
    mouse_last_y_ = ypos;

    glfwGetWindowSize(window, &window_w_, &window_h_);

    shift_key_pressed_ =
      (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) ||
      (glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    mjt_action_ = mjMOUSE_NONE;

    if (mouse_btn_right_pressed_)
    {
      mjt_action_ = shift_key_pressed_ ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    }
    else if (mouse_btn_left_pressed_)
    {
      mjt_action_ = shift_key_pressed_ ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    }
    else
    {
      mjt_action_ = mjMOUSE_ZOOM;
    }

    mjv_moveCamera(
      mj_model_, mjt_action_, mouse_dx_ / window_h_, mouse_dy_ / window_h_,
      &mjv_scene_, &mjv_camera_);
  }
  else
  {
    mouse_last_x_ = xpos;
    mouse_last_y_ = ypos;
  }
}

void PlannerGuiSimulator::scroll_cb_impl(
  GLFWwindow* window, double x_offset, double y_offset)
{
  double xpos = 0.0;
  double ypos = 0.0;
  glfwGetCursorPos(window, &xpos, &ypos);

  if (dispatch_ui_event(mjEVENT_SCROLL, mjBUTTON_NONE, 0, xpos, ypos,
                        x_offset, y_offset, 0))
  {
    return;
  }

  mjv_moveCamera(
    mj_model_, mjMOUSE_ZOOM, 0, -0.05 * y_offset, &mjv_scene_, &mjv_camera_);
}

void PlannerGuiSimulator::control_cb_impl(const mjModel* /* model */, mjData* data)
{
  std::lock_guard<std::mutex> lock(sim_mtx_);

  data->ctrl[0] = jnt_pos_cmd_[0];
  data->ctrl[1] = jnt_pos_cmd_[1];
  data->ctrl[2] = jnt_pos_cmd_[2];
}

bool PlannerGuiSimulator::simulate_impl(
  const char* model, std::promise<bool>&& sim_start_ok)
{
  bool success = true;

  mj_model_ = mj_loadXML(model, nullptr, err_msg_, sizeof(err_msg_));

  if (nullptr != mj_model_)
  {
    mj_data_ = mj_makeData(mj_model_);

    if (nullptr == mj_data_)
    {
      success = false;
      std::strcpy(err_msg_, "MuJoCo data preparation failed.");
    }
  }
  else
  {
    success = false;
  }

  if (success)
  {
    success = get_plugin_instance(mj_model_);

    if (!success)
    {
      std::strcpy(err_msg_, "Index-thumb actuator plugin not found.");
    }
  }

  if (success && !glfwInit())
  {
    success = false;
    std::strcpy(err_msg_, "Failed to initialize GLFW.");
  }

  if (success)
  {
    sim_start_ok.set_value(true);

    window_ = glfwCreateWindow(1400, 900, "Mia Hand - Dynamic Grasp Planner", nullptr, nullptr);
    if (window_ == nullptr)
    {
      success = false;
      std::strcpy(err_msg_, "Failed to create GLFW window.");
    }
  }

  if (success)
  {
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    mjv_defaultCamera(&mjv_camera_);
    mjv_defaultOption(&mjv_options_);
    mjv_defaultScene(&mjv_scene_);
    mjr_defaultContext(&mjr_context_);

    mjv_makeScene(mj_model_, &mjv_scene_, 2000);
    mjr_makeContext(mj_model_, &mjr_context_, mjFONTSCALE_100);

    init_ui();

    glfwSetKeyCallback(window_, &PlannerGuiSimulator::keyboard_cb);
    glfwSetCursorPosCallback(window_, &PlannerGuiSimulator::mouse_move_cb);
    glfwSetMouseButtonCallback(window_, &PlannerGuiSimulator::mouse_button_cb);
    glfwSetScrollCallback(window_, &PlannerGuiSimulator::scroll_cb);
    mjcb_control = &PlannerGuiSimulator::control_cb;

    while (!glfwWindowShouldClose(window_))
    {
      mjt_sim_t0_ = mj_data_->time;
      while ((mj_data_->time - mjt_sim_t0_) < mjt_sim_dt_)
      {
        mj_step(mj_model_, mj_data_);
      }

      refresh_ui_layout();
      sync_status_to_ui();
      mjr_viewport_ = ui_state_.rect[kUiRectViewport];

      mjv_updateScene(
        mj_model_, mj_data_, &mjv_options_, nullptr,
        &mjv_camera_, mjCAT_ALL, &mjv_scene_);
      mjr_render(mjr_viewport_, &mjv_scene_, &mjr_context_);

      if (ui_initialized_)
      {
        mjui_render(&ui_, &ui_state_, &mjr_context_);
      }

      glfwSwapBuffers(window_);
      glfwPollEvents();

      std::lock_guard<std::mutex> lock(sim_mtx_);
      jnt_vel_state_[0] = mj_data_->qvel[1];
      jnt_vel_state_[1] = mj_data_->qvel[2];
      jnt_vel_state_[2] = mj_data_->qvel[3];

      jnt_pos_state_[0] = mj_data_->qpos[1];
      jnt_pos_state_[1] = mj_data_->qpos[2];
      jnt_pos_state_[2] = mj_data_->qpos[3];
    }

    mjcb_control = nullptr;
    mjv_freeScene(&mjv_scene_);
    mjr_freeContext(&mjr_context_);
    mj_deleteModel(mj_model_);
    mj_deleteData(mj_data_);
    mj_model_ = nullptr;
    mj_data_ = nullptr;
    window_ = nullptr;
    ui_initialized_ = false;
    glfwTerminate();
  }
  else
  {
    sim_start_ok.set_value(false);
  }

  return success;
}

bool PlannerGuiSimulator::get_plugin_instance(const mjModel* p_mjm)
{
  bool success = false;
  int act_it = 0;

  while ((!success) && (act_it < p_mjm->nu))
  {
    if (-1 != p_mjm->actuator_plugin[act_it])
    {
      plugin_instance_ = p_mjm->actuator_plugin[act_it];
      success = true;
    }

    ++act_it;
  }

  return success;
}

void PlannerGuiSimulator::init_ui()
{
  std::memset(&ui_state_, 0, sizeof(ui_state_));
  std::memset(&ui_, 0, sizeof(ui_));

  ui_mouse_capture_ = false;

  ui_state_.userdata = this;
  ui_state_.nrect = 3;
  ui_state_.dragrect = 0;
  ui_state_.dragbutton = 0;

  ui_.spacing = mjui_themeSpacing(1);
  ui_.color = mjui_themeColor(0);
  ui_.predicate = nullptr;
  ui_.userdata = this;
  ui_.rectid = kUiRectPanel;
  ui_.auxid = 0;
  ui_.radiocol = 1;

  const mjuiDef planner_def[] = {
    {mjITEM_SECTION,   "Grasp Planner", mjPRESERVE, nullptr,                       "", 0},
    {mjITEM_BUTTON,    "Run Planner",   1,          nullptr,                       "", 0},
    {mjITEM_STATIC,    "Status",        1,          nullptr,                       "Idle", 0},
    {mjITEM_END,       "",              0,          nullptr,                       "", 0}
  };

  mjui_add(&ui_, planner_def);
  ui_.sect[kUiSectionPlanner].state = mjSECT_OPEN;
  ui_initialized_ = true;
  refresh_ui_layout();
  sync_status_to_ui();
}

void PlannerGuiSimulator::refresh_ui_layout()
{
  if (!ui_initialized_ || window_ == nullptr)
  {
    return;
  }

  int fb_width = 0;
  int fb_height = 0;
  glfwGetFramebufferSize(window_, &fb_width, &fb_height);
  if (fb_width <= 0 || fb_height <= 0)
  {
    return;
  }

  ui_state_.rect[kUiRectMain].left = 0;
  ui_state_.rect[kUiRectMain].bottom = 0;
  ui_state_.rect[kUiRectMain].width = fb_width;
  ui_state_.rect[kUiRectMain].height = fb_height;

  const bool size_changed = fb_width != ui_last_width_ || fb_height != ui_last_height_;
  if (size_changed)
  {
    mjui_resize(&ui_, &mjr_context_);
    const int aux_id = ui_.auxid;
    if (mjr_context_.auxFBO[aux_id] == 0 ||
        mjr_context_.auxFBO_r[aux_id] == 0 ||
        mjr_context_.auxColor[aux_id] == 0 ||
        mjr_context_.auxColor_r[aux_id] == 0 ||
        mjr_context_.auxWidth[aux_id] != ui_.width ||
        mjr_context_.auxHeight[aux_id] != ui_.maxheight ||
        mjr_context_.auxSamples[aux_id] != ui_.spacing.samples)
    {
      mjr_addAux(aux_id, ui_.width, ui_.maxheight, ui_.spacing.samples, &mjr_context_);
    }
  }

  ui_state_.rect[kUiRectPanel].left = 0;
  ui_state_.rect[kUiRectPanel].bottom = 0;
  ui_state_.rect[kUiRectPanel].width = ui_.width;
  ui_state_.rect[kUiRectPanel].height = fb_height;

  ui_state_.rect[kUiRectViewport].left = ui_state_.rect[kUiRectPanel].width;
  ui_state_.rect[kUiRectViewport].bottom = 0;
  ui_state_.rect[kUiRectViewport].width =
    std::max(0, fb_width - ui_state_.rect[kUiRectPanel].width);
  ui_state_.rect[kUiRectViewport].height = fb_height;

  if (size_changed)
  {
    mjui_update(-1, -1, &ui_, &ui_state_, &mjr_context_);
    ui_last_width_ = fb_width;
    ui_last_height_ = fb_height;
  }
}

void PlannerGuiSimulator::sync_status_to_ui()
{
  if (!ui_initialized_)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(planner_status_mtx_);
  if (!planner_status_dirty_)
  {
    return;
  }

  std::snprintf(
    ui_.sect[kUiSectionPlanner].item[kUiItemStatus].multi.name[0],
    mjMAXUINAME, "%s", planner_status_pending_.c_str());
  mjui_update(kUiSectionPlanner, kUiItemStatus, &ui_, &ui_state_, &mjr_context_);
  planner_status_dirty_ = false;
}

bool PlannerGuiSimulator::dispatch_ui_event(
  mjtEvent event_type, int button, int key, double x, double y,
  double scroll_x, double scroll_y, int mods)
{
  if (!ui_initialized_ || window_ == nullptr)
  {
    return false;
  }

  refresh_ui_layout();

  int window_width = 0;
  int window_height = 0;
  int fb_width = 0;
  int fb_height = 0;
  glfwGetWindowSize(window_, &window_width, &window_height);
  glfwGetFramebufferSize(window_, &fb_width, &fb_height);
  if (window_width <= 0 || window_height <= 0 || fb_width <= 0 || fb_height <= 0)
  {
    return false;
  }

  const double scale_x = static_cast<double>(fb_width) / static_cast<double>(window_width);
  const double scale_y = static_cast<double>(fb_height) / static_cast<double>(window_height);
  const double fb_x = x * scale_x;
  const double fb_y = (static_cast<double>(window_height) - y) * scale_y;

  ui_state_.dx = fb_x - ui_state_.x;
  ui_state_.dy = fb_y - ui_state_.y;
  ui_state_.x = fb_x;
  ui_state_.y = fb_y;
  ui_state_.sx = scroll_x;
  ui_state_.sy = scroll_y;
  ui_state_.type = event_type;
  ui_state_.button = button;
  ui_state_.key = key;
  ui_state_.left = mouse_btn_left_pressed_ ? 1 : 0;
  ui_state_.right = mouse_btn_right_pressed_ ? 1 : 0;
  ui_state_.middle = mouse_btn_mid_pressed_ ? 1 : 0;
  ui_state_.doubleclick = 0;
  ui_state_.buttontime = glfwGetTime();
  ui_state_.keytime = glfwGetTime();
  ui_state_.control =
    ((mods & GLFW_MOD_CONTROL) != 0) ||
    (glfwGetKey(window_, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) ||
    (glfwGetKey(window_, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS);
  ui_state_.shift =
    ((mods & GLFW_MOD_SHIFT) != 0) ||
    (glfwGetKey(window_, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) ||
    (glfwGetKey(window_, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);
  ui_state_.alt =
    ((mods & GLFW_MOD_ALT) != 0) ||
    (glfwGetKey(window_, GLFW_KEY_LEFT_ALT) == GLFW_PRESS) ||
    (glfwGetKey(window_, GLFW_KEY_RIGHT_ALT) == GLFW_PRESS);
  ui_state_.mouserect =
    mjr_findRect(static_cast<int>(fb_x), static_cast<int>(fb_y),
                 ui_state_.nrect - 1, ui_state_.rect + 1) + 1;

  const bool directed_to_ui =
    (event_type == mjEVENT_KEY) ||
    (ui_state_.dragrect == ui_.rectid) ||
    (ui_state_.dragrect == 0 && ui_state_.mouserect == ui_.rectid);
  if (!directed_to_ui)
  {
    return false;
  }

  mjuiItem* item = mjui_event(&ui_, &ui_state_, &mjr_context_);
  if (item != nullptr)
  {
    handle_ui_item(item);
  }

  return item != nullptr ||
         (event_type == mjEVENT_KEY && ui_state_.key == 0) ||
         ui_state_.dragrect == ui_.rectid ||
         ui_state_.mouserect == ui_.rectid;
}

void PlannerGuiSimulator::handle_ui_item(mjuiItem* item)
{
  if (item == nullptr || item->sectionid != kUiSectionPlanner)
  {
    return;
  }

  if (item->itemid == kUiItemRunPlanner)
  {
    launch_planner();
  }
}

void PlannerGuiSimulator::set_status(const std::string& status)
{
  std::lock_guard<std::mutex> lock(planner_status_mtx_);
  planner_status_pending_ = status;
  planner_status_dirty_ = true;
}

void PlannerGuiSimulator::launch_planner()
{
  bool expected = false;
  if (!planner_running_.compare_exchange_strong(expected, true))
  {
    set_status("Planner already running");
    return;
  }

  planner_request_pending_.store(true);
  set_status("Calling preshaping service");
}
}  // namespace mia_hand_mujoco
