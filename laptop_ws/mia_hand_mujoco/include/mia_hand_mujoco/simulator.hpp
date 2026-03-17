#ifndef MIA_HAND_MUJOCO_SIMULATOR_HPP
#define MIA_HAND_MUJOCO_SIMULATOR_HPP

#include <future>
#include <memory>
#include <GLFW/glfw3.h>

#include "mujoco/mujoco.h"

namespace mia_hand_mujoco
{
class Simulator
{
public:

  /* Preventing Simulator objects to be copyable, either through the copy
   * constructor and the assignment operator, to ensure the unique access to
   * them through the std::unique_ptr.
   */
  Simulator(Simulator&) = delete;
  Simulator& operator=(Simulator&) = delete;

  /**
   * \brief Function for getting a Simulator object instance when callbacks need 
   *        to be accessed.
   */
  static Simulator& get_instance();

  /**
   * \brief Function for getting the error message related to the last detected 
   *        failure.
   *
   * @return Message explaining the reason for the failure.
   */
  const char* get_error_msg();

  /**
   * \brief Function for starting a MuJoCo simulation.
   *
   * @param model Path to the XML model.
   * @param sim_start_ok In case the simulation is run in a separate thread, it 
   *                     can be used to tell the main thread if the simulation 
   *                     started successfully or not.
   * @return \code true \encode, after an entire successful simulation,
   *         \code false \endcode in case of an error during simulation. In case 
   *         of any error, error information can be retrieved by calling 
   *         #get_error_msg.
   */
  static bool simulate(
    const char* model, std::promise<bool>&& sim_start_ok = std::promise<bool>());

  /**
   * \brief Function for stopping the simulation in progress.
   */
  void stop_simulation();

  /**
   * \brief Function for reading MuJoCo joint velocities.
   *
   * @param thumb_vel Variable where to store thumb joint velocity.
   * @param index_vel Variable where to store index joint velocity.
   * @param mrl_vel Variable where to store mrl joint velocity.
   */
  void read_jnt_vel(double& thumb_vel, double& index_vel, double& mrl_vel);

  /**
   * \brief Function for reading MuJoCo joint positions.
   *
   * @param thumb_pos Variable where to store thumb joint position.
   * @param index_pos Variable where to store index joint position.
   * @param mrl_pos Variable where to store mrl joint position.
   */
  void read_jnt_pos(double& thumb_pos, double& index_pos, double& mrl_pos);

  /**
   * \brief Function for setting a MuJoCo joint position.
   */
  void set_jnt_pos(uint_fast8_t jnt, double pos);

  /**
   * \brief Function for setting a MuJoCo joint velocity.
   */
  void set_jnt_vel(uint_fast8_t jnt, double vel);

  /**
   * \brief Function for stopping a MuJoCo joint.
   */
  void stop_jnt(uint_fast8_t jnt);

private:

  /**
   * \brief Class constructor.
   *
   * The class constructor is private to allow instantiations of new Simulator
   * objects only through #create() factory function.
   */
  Simulator();

  /**
   * \brief Callback function called when a key is pressed.
   */
  static void keyboard_cb(
    GLFWwindow* window, int key, int scancode, int act, int mods);

  /**
   * \brief Callback function called when the mouse is moved.
   */
  static void mouse_move_cb(GLFWwindow* window, double xpos, double ypos);

  /**
   * \brief Callback function called when a mouse button id pressed.
   */
  static void mouse_button_cb(
    GLFWwindow* window, int button, int act, int mods);

  /**
   * \brief Callback function called after scrolling.
   */
  static void scroll_cb(GLFWwindow* window, double x_offset, double y_offset);

  /**
   * \brief Simulation control callback.
   */
  static void control_cb(const mjModel* model, mjData* data);

  /**
   * \brief Implementation of key callback function.
   */
  void keyboard_cb_impl(
    GLFWwindow* window, int key, int scancode, int act, int mods);

  /**
   * \brief Implementation of mouse move callback.
   */
  void mouse_move_cb_impl(GLFWwindow* window, double xpos, double ypos);

  /**
   * \brief Implementation of mouse button callback.
   */
  void mouse_button_cb_impl(
    GLFWwindow* window, int button, int act, int mods);

  /**
   * \brief Implementation of scroll callback.
   */
  void scroll_cb_impl(GLFWwindow* window, double x_offset, double y_offset);

  /**
   * \brief Implementation of simulation control callback.
   */
  void control_cb_impl(const mjModel* model, mjData* data);
  
  /**
   * \brief Implementation of simulation function.
   */
  bool simulate_impl(
    const char* model, std::promise<bool>&& sim_start_ok);

  /**
   * \brief Function for getting index-thumb actuator plugin instance.
   */
  bool get_plugin_instance(const mjModel* p_mjm);

  mjModel* mj_model_;  //!< MuJoCo model.
  mjData* mj_data_;    //!< MuJoCo simulation data.

  mjvCamera mjv_camera_;   //!< MuJoCo abstract camera.
  mjvOption mjv_options_;  //!< MuJoCo visualization options.
  mjvScene mjv_scene_;     //!< MuJoCo visualization scene.

  mjrContext mjr_context_;  //!< MuJoCo GPU context.
  mjrRect mjr_viewport_;    //!< MuJoCo framebuffer viewport.

  mjtMouse mjt_action_;      //!< MuJoCo action.
  mjtNum mjt_sim_t0_;        //!< MuJoCo simulation start time.
  const mjtNum mjt_sim_dt_;  //!< MuJoCo simulation period.

  bool mouse_btn_left_pressed_;   //!< Mouse left button pressure flag.
  bool mouse_btn_right_pressed_;  //!< Mouse right button pressure flag.
  bool mouse_btn_mid_pressed_;    //!< Mouse middle button pressure flag.

  double mouse_last_x_;  //!< Mouse last X-axis position.
  double mouse_last_y_;  //!< Mouse last Y-axis position.

  double mouse_dx_;  //!< Mouse X-axis displacement.
  double mouse_dy_;  //!< Mouse Y-axis displacement.

  GLFWwindow* window_;  //!< Simulation window.

  int window_w_;  //!< Window width.
  int window_h_;  //!< Window height.

  bool shift_key_pressed_;  //!< SHIFT key pressure flag.

  char err_msg_[256];  //!< Error message, filled after any occurred error.

  int plugin_instance_;  //!< Instance of index-thumb actuator plugin.

  double jnt_vel_state_[3];  //!< Joint velocity states.
  double jnt_vel_cmd_[3];    //!< Joint velocity commands.

  double jnt_pos_state_[3];  //!< Joint position states.
  double jnt_pos_cmd_[3];    //!< Joint position commands.

  std::mutex sim_mtx_;  //!< For thread-safety.
};
}  // namespace

#endif  // MIA_HAND_MUJOCO_SIMULATOR_HPP

