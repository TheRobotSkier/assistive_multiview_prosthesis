#include "mia_hand_mujoco/plugin/index_thumb_actuator.h"

#include <array>

namespace mia_hand_mujoco::plugin
{
constexpr char kIndexFlxJntNameAttr[] = "index_flx_jnt_name";
constexpr char kThumbOppJntNameAttr[] = "thumb_opp_jnt_name";
constexpr char kThumbOppStartIndexAngleAttr[] = "thumb_opp_start_index_angle";
constexpr char kThumbOppEndIndexAngleAttr[] = "thumb_opp_end_index_angle";
constexpr char kKpAttr[] = "kp";
constexpr char kKvAttr[] = "kv";

std::unique_ptr<ThumbOppInfo> ThumbOppInfo::create(
  const mjModel* p_mjm, mjData* p_mjd, int instance)
{
  std::unique_ptr<ThumbOppInfo> ptr = 
    std::unique_ptr<ThumbOppInfo>(new ThumbOppInfo());

  /* Reading plugin attributes from model.
   */
  const char* jnt_name = mj_getPluginConfig(p_mjm, instance, kThumbOppJntNameAttr);

  if ((nullptr == jnt_name) || ('\0' == jnt_name[0]))
  {
    mju_warning("Thumb opposition joint name missing or empty.");
    ptr = nullptr;
  }

  if (nullptr != ptr)
  {
    const char* start_index_angle = 
      mj_getPluginConfig(p_mjm, instance, kThumbOppStartIndexAngleAttr);

    if ((nullptr != start_index_angle) && ('\0' != start_index_angle[0]))
    {
      ptr->start_index_angle = strtod(start_index_angle, nullptr);
    }
    else
    {
      mju_warning("Thumb opposition start index angle missing or empty.");
      ptr = nullptr;
    }
  }

  if (nullptr != ptr)
  {
    const char* end_index_angle = 
      mj_getPluginConfig(p_mjm, instance, kThumbOppEndIndexAngleAttr);

    if ((nullptr != end_index_angle) && ('\0' != end_index_angle[0]))
    {
      ptr->end_index_angle = strtod(end_index_angle, nullptr);
    }
    else
    {
      mju_warning("Thumb opposition end index angle missing or empty.");
      ptr = nullptr;
    }
  }

  /* Computing thumb opposition scale and offset, and linking to its position state.
   */
  if (nullptr != ptr)
  {
    int jnt_id = mj_name2id(p_mjm, mjOBJ_JOINT, jnt_name);

    if (jnt_id >= 0)
    {
      ptr->min_angle = p_mjm->jnt_range[jnt_id * 2];
      ptr->max_angle = p_mjm->jnt_range[jnt_id * 2 + 1];

      ptr->scale = 
        (ptr->max_angle - ptr->min_angle) / 
        (ptr->end_index_angle - ptr->start_index_angle);
      ptr->offset = ptr->min_angle - (ptr->scale * ptr->start_index_angle);

      ptr->p_qpos = &(p_mjd->qpos[p_mjm->jnt_qposadr[jnt_id]]);
    }
    else
    {
      mju_warning("Joint %s not found.", jnt_name);
      ptr = nullptr;
    }
  }

  return ptr;
}

std::unique_ptr<IndexThumbActuator> IndexThumbActuator::create(
  const mjModel* p_mjm, mjData* p_mjd, int instance)
{
  std::unique_ptr<IndexThumbActuator> ptr = nullptr;

  std::unique_ptr<ThumbOppInfo> p_thumb_opp = 
    ThumbOppInfo::create(p_mjm, p_mjd, instance);

  if (nullptr != p_thumb_opp)
  {
    ptr = std::unique_ptr<IndexThumbActuator>(
      new IndexThumbActuator(p_thumb_opp.release()));

    /* Getting actuator ID. TODO: evaluate if checking also the number of 
     * actuators found and act_dim.
     */
    for (int act = 0; act < p_mjm->nu; ++act)
    {
      if (p_mjm->actuator_plugin[act] == instance)
      {
        ptr->act_id_ = act;
      }
    }
  }

  /* Reading index joint name attribute and finding its ID.
   */
  if (nullptr != ptr)
  {
    const char* jnt_name = mj_getPluginConfig(p_mjm, instance, kIndexFlxJntNameAttr);

    if ((nullptr != jnt_name) && ('\0' != jnt_name[0]))
    {
      int jnt_id = mj_name2id(p_mjm, mjOBJ_JOINT, jnt_name);

      if (jnt_id >= 0)
      {
        ptr->p_index_qpos_ = &(p_mjd->qpos[p_mjm->jnt_qposadr[jnt_id]]);
      }
      else
      {
        mju_warning("Joint %s not found.", jnt_name);
        ptr = nullptr;
      }
    }
    else
    {
      mju_warning("Index flexion joint name missing or empty.");
      ptr = nullptr;
    }
  }

  /* Reading Kp and Kv attributes.
   */
  if (nullptr != ptr)
  {
    const char* gain = mj_getPluginConfig(p_mjm, instance, kKpAttr);

    if ((nullptr != gain) && ('\0' != gain[0]))
    {
      ptr->kp_ = strtod(gain, nullptr);
    }
    else
    {
      ptr->kp_ = 1;  // Default gain value.
    }

    const char* bias = mj_getPluginConfig(p_mjm, instance, kKvAttr);

    if ((nullptr != bias) && ('\0' != bias[0]))
    {
      ptr->kv_ = strtod(bias, nullptr);
    }
    else
    {
      ptr->kv_ = 0;  // Default bias value.
    }
  }

  return ptr;
}

void IndexThumbActuator::reset(mjtNum* /*p_plugin_state*/)
{
  return;
}

void IndexThumbActuator::act_dot(
  const mjModel* /*p_mjm*/, mjData* /*p_mjd*/, int /*instance*/) const
{
  // TODO

  return;
}

void IndexThumbActuator::compute(
  const mjModel* p_mjm, mjData* p_mjd, int /*instance*/)
{
  mjtNum ctrl = get_ctrl(p_mjm, p_mjd);

  /* Checking index joint reverse of motion.
   *  
   * TODO: the angle for checking the switch is not 0.0 rad, but -0.001 rad. This 
   * is done because the simulation does not start at an angle of exactly 0.0 rad, 
   * but at a slightly negative angle, causing the index finger not to reverse 
   * its motion if the first control command is a negative angle. Using a 
   * threshold of -0.001 rad eliminates this issue. Evaluate if the initial 
   * joint angle can be driven to a positive value by adjusting the joint 
   * dynamics parameters.
   */
  if ((*(p_index_qpos_) < -0.001) && (index_qpos_old_ >= -0.001))
  {
    if (0 == b_index_reversed_)
    {
      b_index_reversed_ = 1;
    }
    else
    {
      b_index_reversed_ = 0;
    }
  }

  /* Reemapping index joint angle and ctrl, in case of reverse motion.
   */
  mjtNum index_qpos_remap;

  if (0 == b_index_reversed_)
  {
    index_qpos_remap = *p_index_qpos_;
  }
  else
  {
    index_qpos_remap = -(*p_index_qpos_);
    ctrl = -ctrl;
  }

  /* Calculating actuator force, following the relation of a position actuator.
   */
  if (-1 == p_mjm->actuator_actadr[act_id_])
  {
    p_mjd->actuator_force[act_id_] = 
      kp_ * ctrl - kp_ * p_mjd->actuator_length[act_id_] 
      - kv_ * p_mjd->actuator_velocity[act_id_];
  }
  else
  {
    /* Using last activation.
     */
    int act_adr = 
      p_mjm->actuator_actadr[act_id_] + p_mjm->actuator_actnum[act_id_] - 1;

    mjtNum act;

    if (p_mjm->actuator_actearly[act_id_])
    {
      act = next_activation(p_mjm, p_mjd, act_adr, p_mjd->act_dot[act_adr]);
    }
    else
    {
      act = p_mjd->act[act_adr];
    }

    p_mjd->actuator_force[act_id_] = 
      kp_ * act - kp_ * p_mjd->actuator_length[act_id_] 
      - kv_ * p_mjd->actuator_velocity[act_id_];
  }

  /* Computing thumb opposition joint state.
   */
  *(p_thumb_opp_->p_qpos) = 
    (p_thumb_opp_->scale * index_qpos_remap) + p_thumb_opp_->offset;

  if (*(p_thumb_opp_->p_qpos) <= (p_thumb_opp_->min_angle))
  {
    *(p_thumb_opp_->p_qpos) = p_thumb_opp_->min_angle;
  }
  else if (*(p_thumb_opp_->p_qpos) >= (p_thumb_opp_->max_angle))
  {
    *(p_thumb_opp_->p_qpos) = p_thumb_opp_->max_angle;
  }
  else
  {
  }

  /* Updating previous index joint position.
   */
  index_qpos_old_ = *p_index_qpos_;

  return;
}

void IndexThumbActuator::advance(
  const mjModel* /*p_mjm*/, mjData* /*p_mjd*/, int /*instance*/) const
{
  // TODO

  return;
}

mjtByte IndexThumbActuator::is_index_reversed()
{
  return b_index_reversed_;
}

void IndexThumbActuator::register_plugin()
{
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  /* Setting plugin data.
   */
  plugin.name = "mujoco.mia_hand.index_thumb_actuator";
  plugin.capabilityflags |= mjPLUGIN_ACTUATOR;

  std::array<const char*, 6> attributes = {
    kIndexFlxJntNameAttr,
    kThumbOppJntNameAttr,
    kThumbOppStartIndexAngleAttr,
    kThumbOppEndIndexAngleAttr,
    kKpAttr,
    kKvAttr
  };

  plugin.nattribute = attributes.size();
  plugin.attributes = attributes.data();

  /* Setting plugin callbacks.
   */
  plugin.nstate = +[](const mjModel* /*p_mjm*/, int /*instance*/)
  {
    return 0;
  };

  plugin.init = +[](const mjModel* p_mjm, mjData* p_mjd, int instance)
  {
    int return_val = 0;

    std::unique_ptr<IndexThumbActuator> p_act = 
      IndexThumbActuator::create(p_mjm, p_mjd, instance);

    if (nullptr != p_act)
    {
      p_mjd->plugin_data[instance] = reinterpret_cast<uintptr_t>(p_act.release());
    }
    else
    {
    }

    return return_val;
  };

  plugin.destroy = +[](mjData* p_mjd, int instance)
  {
    delete reinterpret_cast<IndexThumbActuator*>(p_mjd->plugin_data[instance]);
    p_mjd->plugin_data[instance] = 0;

    return;
  };

  plugin.reset = +[](const mjModel* /*p_mjm*/, mjtNum* p_plugin_state, 
    void* p_plugin_data, int /*instance*/)
  {
    IndexThumbActuator* p_act = 
      reinterpret_cast<IndexThumbActuator*>(p_plugin_data);

    p_act->reset(p_plugin_state);

    return;
  };

  plugin.actuator_act_dot = +[](const mjModel* p_mjm, mjData* p_mjd, int instance)
  {
    IndexThumbActuator* p_act = 
      reinterpret_cast<IndexThumbActuator*>(p_mjd->plugin_data[instance]);

    p_act->act_dot(p_mjm, p_mjd, instance);

    return;
  };

  plugin.compute = +[](const mjModel* p_mjm, mjData* p_mjd, int instance, 
    int /*capability_bits*/)
  {
    IndexThumbActuator* p_act = 
      reinterpret_cast<IndexThumbActuator*>(p_mjd->plugin_data[instance]);

    p_act->compute(p_mjm, p_mjd, instance);

    return;
  };

  plugin.advance = +[](const mjModel* p_mjm, mjData* p_mjd, int instance)
  {
    IndexThumbActuator* p_act = 
      reinterpret_cast<IndexThumbActuator*>(p_mjd->plugin_data[instance]);

    p_act->advance(p_mjm, p_mjd, instance);

    return;
  };

  /* Registering the plugin.
   */
  mjp_registerPlugin(&plugin);

  return;
}

IndexThumbActuator::IndexThumbActuator(ThumbOppInfo* p_thumb_opp):
  p_thumb_opp_(p_thumb_opp)
{
}

mjtNum IndexThumbActuator::get_ctrl(const mjModel* p_mjm, const mjData* p_mjd)
{
  mjtNum ctrl = 0;

  if (mjDYN_NONE == p_mjm->actuator_dyntype[act_id_])
  {
    ctrl = p_mjd->ctrl[act_id_];

    if (p_mjm->actuator_ctrllimited[act_id_])
    {
      mju_clip(ctrl, p_mjm->actuator_ctrlrange[2 * act_id_], 
        p_mjm->actuator_ctrlrange[2 * act_id_ + 1]);
    }
  }
  else
  {
    /* Getting control from actuation state, to create integrated-velocity 
     * controllers or to filter the controls.
     */
    int actadr = p_mjm->actuator_actadr[act_id_] +
      p_mjm->actuator_actnum[act_id_] - 1;

    if (p_mjm->actuator_actearly[act_id_])
    {
      ctrl = next_activation(p_mjm, p_mjd, actadr, p_mjd->act_dot[actadr]);
    }
    else
    {
      ctrl = p_mjd->act[actadr];
    }
  }

  return ctrl;
}

mjtNum IndexThumbActuator::next_activation(
  const mjModel* p_mjm, const mjData* p_mjd, int actadr, mjtNum act_dot)
{
  mjtNum act = p_mjd->act[actadr];

  if (mjDYN_FILTEREXACT == p_mjm->actuator_dyntype[act_id_])
  {
    /* Exact filter integration.
     */
    mjtNum tau = mju_max(mjMINVAL, p_mjm->actuator_dynprm[act_id_ * mjNDYN]);
    act = act + act_dot * tau * (1 - mju_exp(-p_mjm->opt.timestep / tau));
  }
  else
  {
    /* Euler integration.
     */
    act = act + act_dot + p_mjm->opt.timestep;
  }

  /* Clamping to actuator range.
   */
  if (p_mjm->actuator_actlimited[act_id_])
  {
    mjtNum* actrange = p_mjm->actuator_actrange + 2 * act_id_;
    act = mju_clip(act, actrange[0], actrange[1]);
  }
  
  return act;
}
}  // namespace

