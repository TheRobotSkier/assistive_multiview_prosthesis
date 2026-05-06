#include "mia_hand_description/thumb_joint_opposition.hpp"

namespace mia_hand_description
{
std::unique_ptr<ThumbJointOpposition> ThumbJointOpposition::create(
    std::shared_ptr<rclcpp::Node> node, const std::string& prefix)
{
  std::unique_ptr<ThumbJointOpposition> ptr = 
    std::unique_ptr<ThumbJointOpposition>(new ThumbJointOpposition());

  if (false == ptr->init(node, prefix))
  {
    ptr = nullptr;
  }

  return ptr;
}

ThumbJointOpposition::ThumbJointOpposition():
  thumb_jnt_min_pos_(0.0),
  thumb_jnt_max_pos_(0.0),
  thumb_opp_start_index_ang_(0.0),
  thumb_opp_stop_index_ang_(0.0)
{
}

bool ThumbJointOpposition::init(
    std::shared_ptr<rclcpp::Node> node, const std::string& prefix)
{
  bool init_success = true;

  if (load_jnt_limits_from_urdf(node, prefix))
  {
    thumb_opp_start_index_ang_ = node->declare_parameter(
        "transmissions.j_thumb_opp.th_opp_start_close_index_angle", 0.0f);
    thumb_opp_stop_index_ang_ = node->declare_parameter(
        "transmissions.j_thumb_opp.th_opp_stop_close_index_angle", 0.0f);

    thumb_opp_scale_ = (thumb_jnt_max_pos_ - thumb_jnt_min_pos_) 
                      / (thumb_opp_stop_index_ang_ - thumb_opp_start_index_ang_);
    thumb_opp_offset_ = thumb_jnt_min_pos_ 
                        - (thumb_opp_scale_ * thumb_opp_start_index_ang_);
  }
  else
  {
    init_success = false;
  }

  return init_success;
}

double ThumbJointOpposition::get_opposition_ang(double index_jnt_ang)
{
  /* Forcing index joint angle accuracy to 0.001 rad, to avoid thumb flickering
   * during RViz display.
   */
  double index_jnt_ang_round = (floor(index_jnt_ang * 1000)) / 1000;

  /* Computing opposition angle, according to the thumb joint transmission chain.
   */
  double opp_ang = (thumb_opp_scale_ * index_jnt_ang_round) + thumb_opp_offset_;

  /* Saturating opposition angle, to fit it into joint limits interval.
   */
  if (opp_ang <= thumb_jnt_min_pos_)
  {
    opp_ang = thumb_jnt_min_pos_ + 0.02;
  }
  else if (opp_ang >= thumb_jnt_max_pos_)
  {
    opp_ang = thumb_jnt_max_pos_ - 0.02;
  }
  else
  {
  }

  return opp_ang;
}

bool ThumbJointOpposition::load_jnt_limits_from_urdf(
    std::shared_ptr<rclcpp::Node> node, const std::string& prefix)
{
  bool success = true;

  std::string urdf = node->declare_parameter("robot_description", "");

  if ("" != urdf)  // URDF not empty
  {
    urdf::Model urdf_model;

    if (urdf_model.initString(urdf))  // URDF model initialized
    {
      urdf::JointConstSharedPtr thumb_jnt_urdf =
        urdf_model.getJoint(prefix + "j_thumb_opp");

      if (nullptr != thumb_jnt_urdf)  // URDF thumb joint found
      {
        joint_limits::JointLimits thumb_jnt_limits;

        if (joint_limits::getJointLimits(
              thumb_jnt_urdf, thumb_jnt_limits))  // URDF thumb joint limits found
        {
          thumb_jnt_min_pos_ = thumb_jnt_limits.min_position;
          thumb_jnt_max_pos_ = thumb_jnt_limits.max_position;
        }
        else  // URDF thumb joint limits not found
        {
          success = false;
          RCLCPP_ERROR(node->get_logger(), 
              "Failed to get thumb joint limits from URDF.");
        }
      }
      else  // URDF thumb joint not found
      {
        success = false;
        RCLCPP_ERROR(node->get_logger(), 
            "Failed to get thumb joint from URDF.");
      }
    }
    else  // URDF model not initialized
    {
      success = false;
      RCLCPP_ERROR(node->get_logger(), 
          "Failed to initialize URDF model.");
    }
  }
  else  // URDF empty
  {
    success = false;
    RCLCPP_ERROR(node->get_logger(), "Found empty URDF.");
  }

  return success;
}
}  // namespace

