#ifndef MIA_HAND_MUJOCO_INTERACTIVE_SIMULATOR_HPP
#define MIA_HAND_MUJOCO_INTERACTIVE_SIMULATOR_HPP

#include <future>
#include <mutex>
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
  InteractiveSimulator();

  static void control_cb(const mjModel* model, mjData* data);
  void control_cb_impl(const mjModel* model, mjData* data);

  bool simulate_impl(const char* model, std::promise<bool>&& sim_start_ok);
  void physics_thread_fn(const char* model_path, std::promise<bool>&& sim_start_ok);
  bool get_plugin_instance(const mjModel* p_mjm);

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
};
}  // namespace mia_hand_mujoco

#endif  // MIA_HAND_MUJOCO_INTERACTIVE_SIMULATOR_HPP
