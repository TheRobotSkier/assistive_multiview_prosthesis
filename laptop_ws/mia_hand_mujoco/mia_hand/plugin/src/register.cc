#include <mujoco/mjplugin.h>
#include "mia_hand_mujoco/plugin/index_thumb_actuator.h"

namespace mia_hand_mujoco::plugin
{
  mjPLUGIN_LIB_INIT{IndexThumbActuator::register_plugin();}
}  // namespace
