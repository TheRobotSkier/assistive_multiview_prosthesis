#ifndef MIA_HAND_MUJOCO_INDEX_THUMB_ACTUATOR_H
#define MIA_HAND_MUJOCO_INDEX_THUMB_ACTUATOR_H

#include <memory>

#include "mujoco/mujoco.h"

namespace mia_hand_mujoco::plugin
{
struct ThumbOppInfo
{
  mjtNum min_angle;          //!< Thumb opposition minimum angle.
  mjtNum max_angle;          //!< Thumb opposition maximum angle.
  mjtNum scale;              //!< Scale for computing thumb opposition angle.
  mjtNum offset;             //!< Offset for computing thumb opposition angle.
  mjtNum start_index_angle;  //!< Index angle at which thumb opposition starts (attribute).
  mjtNum end_index_angle;    //!< Index angle at which thumb opposition ends (attribute).
  mjtNum* p_qpos;            //!< Thumb opposition joint position state.

  /**
   * \brief Fuction for creating a new ThumbOppInfo object, after reading 
   *        attributes from the model.
   *
   * @return std::unique_ptr to the object, if attributes reading was successful, 
   *         nullptr if not.
   */
  static std::unique_ptr<ThumbOppInfo> create(
    const mjModel* p_mjm, mjData* p_mjd, int instance);
};

class IndexThumbActuator
{
public:

  /**
   * \brief Factory function for creating a new IndexThumbActuator object.
   *
   * @return std::unique_ptr to the object, if object creation is
   *         successful. nullptr, if object creation fails.
   */
  static std::unique_ptr<IndexThumbActuator> create(
    const mjModel* p_mjm, mjData* p_mjd, int instance);

  /**
   * \brief Function for resetting plugin instance state.
   *
   * // TODO: add param info
   */
  void reset(mjtNum* p_plugin_state);

  /**
   * \brief Function for computing the rate of change for activation variables.
   *
   * // TODO: add param info
   */
  void act_dot(const mjModel* p_mjm, mjData* p_mjd, int instance) const;

  /**
   * \brief Idempotent function to update the actuator force and the internal 
   *        state of the class. Called after #act_dot.
   */
  void compute(const mjModel* p_mjm, mjData* p_mjd, int instance);

  /**
   * \brief Function for updating the plugin state.
   */
  void advance(const mjModel* p_mjm, mjData* p_mjd, int instance) const;

  /**
   * \brief Function for knowing if index finger is in reversed motion.
   */
  mjtByte is_index_reversed();

  /**
   * \brief Function for adding the plugin to the global registry of MuJoCo plugins.
   */
  static void register_plugin();

private:

  /**
   * \brief Class constructor.
   */
  IndexThumbActuator(ThumbOppInfo* p_thumb_opp);

  /**
   * \brief Function for getting current actuator control signal.
   */ 
  mjtNum get_ctrl(const mjModel* p_mjm, const mjData* p_mjd);

  /**
   * \brief Function for getting next actuator activation.
   */
  mjtNum next_activation(
    const mjModel* p_mjm, const mjData* p_mjd, int actadr, mjtNum act_dot);

  std::unique_ptr<ThumbOppInfo> p_thumb_opp_;  //!< Thumb opposition info object.

  int act_id_;  //!< Actuator ID.
  mjtNum kp_;   //!< Gain.
  mjtNum kv_;   //!< Bias.

  mjtNum* p_index_qpos_;   //!< Pointer to index flexion joint position.
  mjtNum index_qpos_old_;  //!< Previous index flexion joint position.

  mjtByte b_index_reversed_;  //!< Used for reversing index motion during thumb opposition.
};
}  // namespace

#endif  // MIA_HAND_MUJOCO_INDEX_THUMB_ACTUATOR_H
