#include "mia_hand_mujoco/simulator.hpp"

#include <string.h>

#include "mia_hand_mujoco/plugin/index_thumb_actuator.h"

namespace mia_hand_mujoco
{
Simulator& Simulator::get_instance()
{
  static Simulator sim_instance;

  return sim_instance;
}

const char* Simulator::get_error_msg()
{
  return err_msg_;
}

bool Simulator::simulate(
  const char* model, std::promise<bool>&& sim_start_ok)
{
  /* Checking MuJoCo API and library version match.
   */
  if (mjVERSION_HEADER != mj_version())
  {
    mju_warning("MuJoCo API and library version do not match.");
  }

  return get_instance().simulate_impl(model, std::move(sim_start_ok));
}

void Simulator::stop_simulation()
{
  glfwSetWindowShouldClose(window_, GLFW_TRUE);

  return;
}

void Simulator::read_jnt_vel(
  double& thumb_vel, double& index_vel, double& mrl_vel)
{
  std::lock_guard<std::mutex> lock(sim_mtx_);  // Ensuring thread-safety.

  mjtByte index_reversed = 
    reinterpret_cast<plugin::IndexThumbActuator*>(
      mj_data_->plugin_data[plugin_instance_])->is_index_reversed();

  thumb_vel = jnt_vel_state_[0];

  if (!index_reversed)
  {
    index_vel = jnt_vel_state_[1];
  }
  else
  {
    index_vel = -jnt_vel_state_[1];
  }

  mrl_vel = jnt_vel_state_[2];

  return;
}

void Simulator::read_jnt_pos(
  double& thumb_pos, double& index_pos, double& mrl_pos)
{
  std::lock_guard<std::mutex> lock(sim_mtx_);  // Ensuring thread-safety.

  mjtByte index_reversed = 
    reinterpret_cast<plugin::IndexThumbActuator*>(
      mj_data_->plugin_data[plugin_instance_])->is_index_reversed();

  thumb_pos = jnt_pos_state_[0];

  if (!index_reversed)
  {
    index_pos = jnt_pos_state_[1];
  }
  else
  {
    index_pos = -jnt_pos_state_[1];
  }

  mrl_pos = jnt_pos_state_[2];

  return;
}

void Simulator::set_jnt_pos(uint_fast8_t jnt, double pos)
{
  std::lock_guard<std::mutex> lock(sim_mtx_);  // Ensuring thread-safety.

  jnt_pos_cmd_[jnt] = pos;

  return;
}

void Simulator::set_jnt_vel(uint_fast8_t jnt, double vel)
{
  std::lock_guard<std::mutex> lock(sim_mtx_);  // Ensuring thread-safety.

  jnt_vel_cmd_[jnt] = vel;

  return;
}

void Simulator::stop_jnt(uint_fast8_t jnt)
{
  std::lock_guard<std::mutex> lock(sim_mtx_);  // Ensuring thread-safety.

  jnt_pos_cmd_[jnt] = mj_data_->qpos[jnt];

  return;
}

Simulator::Simulator():
  mj_model_(nullptr),
  mj_data_(nullptr),
  mjr_viewport_({0, 0, 0, 0}),
  mjt_sim_t0_(0.0),
  mjt_sim_dt_(1.0/60.0),
  mouse_btn_left_pressed_(false),
  mouse_btn_right_pressed_(false),
  mouse_btn_mid_pressed_(false),
  mouse_last_x_(0.0),
  mouse_last_y_(0.0)
{
}

void Simulator::keyboard_cb(
  GLFWwindow* window, int key, int scancode, int act, int mods)
{
  get_instance().keyboard_cb_impl(window, key, scancode, act, mods);

  return;
}

void Simulator::mouse_move_cb(GLFWwindow* window, double xpos, double ypos)
{
  get_instance().mouse_move_cb_impl(window, xpos, ypos);

  return;
}

void Simulator::mouse_button_cb(
    GLFWwindow* window, int button, int act, int mods)
{
  get_instance().mouse_button_cb_impl(window, button, act, mods);

  return;
}

void Simulator::scroll_cb(GLFWwindow* window, double x_offset, double y_offset)
{
  get_instance().scroll_cb_impl(window, x_offset, y_offset);

  return;
}

void Simulator::control_cb(const mjModel* model, mjData* data)
{
  get_instance().control_cb_impl(model, data);

  return;
}

void Simulator::keyboard_cb_impl(
  GLFWwindow* /* window */, int key, int /* scancode */, int act, int /* mods */)
{
  /* Backspace: reset simulation.
   */
  if ((GLFW_PRESS == act) && (GLFW_KEY_BACKSPACE == key))
  {
    mj_resetData(mj_model_, mj_data_);
    mj_forward(mj_model_, mj_data_);
  }

  return;
}

void Simulator::mouse_button_cb_impl(
  GLFWwindow* window, int /* button */, int /* act */, int /* mods */)
{
  /* Updating buttons state.
   */
  mouse_btn_left_pressed_ = 
    glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
  mouse_btn_mid_pressed_ = 
    glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
  mouse_btn_right_pressed_ = 
    glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;

  /* Updating mouse position.
   */
  glfwGetCursorPos(window, &mouse_last_x_, &mouse_last_y_);

  return;
}

void Simulator::mouse_move_cb_impl(
  GLFWwindow* window, double xpos, double ypos)
{
  if (mouse_btn_right_pressed_ ||
      mouse_btn_mid_pressed_ ||
      mouse_btn_left_pressed_)
  {
    /* Computing mouse displacement.
     */
    mouse_dx_ = xpos - mouse_last_x_;
    mouse_dy_ = ypos - mouse_last_y_;

    /* Updating mouse position.
     */
    mouse_last_x_ = xpos;
    mouse_last_y_ = ypos;

    /* Getting window size.
     */
    glfwGetWindowSize(window, &window_w_, &window_h_);

    /* Updating SHIFT key status.
     */
    shift_key_pressed_ = 
      (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) ||
      (glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    /* Determining action depending on mouse buttons pressure.
     */
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

    /* Moving the camera according to the detected action.
     */
    mjv_moveCamera(
      mj_model_, mjt_action_, mouse_dx_/window_h_, mouse_dy_/window_h_, 
      &mjv_scene_, &mjv_camera_);
  }

  return;
}

void Simulator::scroll_cb_impl(
  GLFWwindow* /* window */, double /* x_offset */, double y_offset)
{
  mjv_moveCamera(
    mj_model_, mjMOUSE_ZOOM, 0, -0.05 * y_offset, &mjv_scene_, &mjv_camera_);

  return;
}

void Simulator::control_cb_impl(const mjModel* /* model */, mjData* data)
{
  std::lock_guard<std::mutex> lock(sim_mtx_);  // Ensuring thread-safety.

  /* TODO: insert also velocity commands.
   */
  data->ctrl[0] = jnt_pos_cmd_[0];
  data->ctrl[1] = jnt_pos_cmd_[1];
  data->ctrl[2] = jnt_pos_cmd_[2];

  return;
}

bool Simulator::simulate_impl(
  const char* model, std::promise<bool>&& sim_start_ok)
{
  bool success = true;

  /* Loading XML model and preparing simulation data.
   */
  mj_model_ = mj_loadXML(model, nullptr, err_msg_, sizeof(err_msg_)); // Loading model.

  if (nullptr != mj_model_)
  {
    mj_data_ = mj_makeData(mj_model_);

    if (nullptr == mj_data_)
    {
      success = false;
      strcpy(err_msg_, "MuJoCo data preparation failed.");
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
      strcpy(err_msg_, "Index-thumb actuator plugin not found.");
    }
  }

  /* Initializing GLFW.
   */
  if (success)
  {
    if (!glfwInit())
    {
      success = false;
      strcpy(err_msg_, "Failed to initialize GLFW.");
    }
  }

  /* Running simulation.
   */
  if (success)
  {
    /* Signaling simulation start successful to any main thread.
     */
    sim_start_ok.set_value(true);

    /* Setting up window.
     */
    window_ = glfwCreateWindow(
      1200, 900, "Mia Hand", nullptr, nullptr);  // Creating window.

    glfwMakeContextCurrent(window_);  // Making context current.
    glfwSwapInterval(1);              // Setting swap interval (vsync).

    /* Initializing visualization data structures.
     */
    mjv_defaultCamera(&mjv_camera_);
    mjv_defaultOption(&mjv_options_);
    mjv_defaultScene(&mjv_scene_);
    mjr_defaultContext(&mjr_context_);

    /* Creating scene and context.
     */
    mjv_makeScene(mj_model_, &mjv_scene_, 2000);
    mjr_makeContext(mj_model_, &mjr_context_, mjFONTSCALE_150);

    /* Setting callbacks.
     */
    glfwSetKeyCallback(window_, &Simulator::keyboard_cb);
    glfwSetCursorPosCallback(window_, &Simulator::mouse_move_cb);
    glfwSetMouseButtonCallback(window_, &Simulator::mouse_button_cb);
    glfwSetScrollCallback(window_, &Simulator::scroll_cb);
    mjcb_control = &Simulator::control_cb;

    /* Simulation loop.
     */
    while (!glfwWindowShouldClose(window_))
    {
      /* Advancing interactive simulation for the simulation period (1/60 s).
       * Assuming that MuJoCo can simulate faster than real-time, which usually 
       * is true, this loop should finish on time for the next frame to be 
       * rendered at 60 fps. Otherwise, a CPU timer can be added, to exit the 
       * loop when it's time to render.
       */
      mjt_sim_t0_ = mj_data_->time;

      while ((mj_data_->time - mjt_sim_t0_) < mjt_sim_dt_)
      {
        mj_step(mj_model_, mj_data_);
      }

      /* Getting framebuffer viewport.
       */
      glfwGetFramebufferSize(
        window_, &mjr_viewport_.width, &mjr_viewport_.height);

      /* Updating scene and render.
       */
      mjv_updateScene(
        mj_model_, mj_data_, &mjv_options_, nullptr, 
        &mjv_camera_, mjCAT_ALL, &mjv_scene_);
      mjr_render(mjr_viewport_, &mjv_scene_, &mjr_context_);

      /* Swapping OpenGL buffers.
       */
      glfwSwapBuffers(window_);

      /* Processing pending GUI events and calling GLFW callbacks.
       */
      glfwPollEvents();

      /* Updating joint states.
       */
      std::lock_guard<std::mutex> lock(sim_mtx_);  // Ensuring thread-safety.

      jnt_vel_state_[0] = mj_data_->qvel[1];
      jnt_vel_state_[1] = mj_data_->qvel[2];
      jnt_vel_state_[2] = mj_data_->qvel[3];

      jnt_pos_state_[0] = mj_data_->qpos[1];
      jnt_pos_state_[1] = mj_data_->qpos[2];
      jnt_pos_state_[2] = mj_data_->qpos[3];
    }

    /* Freeing visualization structures.
     */
    mjv_freeScene(&mjv_scene_);
    mjr_freeContext(&mjr_context_);

    /* Deallocating model and data.
     */
    mj_deleteModel(mj_model_);
    mj_deleteData(mj_data_);

    /* Terminating GLFW.
     */
    glfwTerminate();
  }
  else
  {
    /* Signaling simulation start failure to any main thread.
     */
    sim_start_ok.set_value(false);
  }

  return success;
}

bool Simulator::get_plugin_instance(const mjModel* p_mjm)
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
}  // namespace

