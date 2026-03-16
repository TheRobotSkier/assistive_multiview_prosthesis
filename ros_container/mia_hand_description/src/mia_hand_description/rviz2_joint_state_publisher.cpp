#include "mia_hand_description/rviz2_joint_state_publisher.hpp"

namespace mia_hand_description
{
std::unique_ptr<Rviz2JointStatePublisher> Rviz2JointStatePublisher::create(
    std::shared_ptr<rclcpp::Node> node)
{
  std::unique_ptr<Rviz2JointStatePublisher> ptr = 
    std::unique_ptr<Rviz2JointStatePublisher>(new Rviz2JointStatePublisher());

  if (false == ptr->init(node))
  {
    ptr = nullptr;
  }

  return ptr;
}

Rviz2JointStatePublisher::Rviz2JointStatePublisher()
{
}

bool Rviz2JointStatePublisher::init(std::shared_ptr<rclcpp::Node> node)
{
  bool init_success = true;

  std::string prefix = node->declare_parameter("prefix", "");

  thumb_jnt_opp_ = ThumbJointOpposition::create(node, prefix);

  if (nullptr != thumb_jnt_opp_)
  {
    node_ = node;

    thumb_jnt_name_ = prefix + "j_thumb_opp";
    index_jnt_name_ = prefix + "j_index_fle";

    jnt_state_raw_sub_ = 
      node_->create_subscription<sensor_msgs::msg::JointState>(
          "joint_states", 1, 
          std::bind(&Rviz2JointStatePublisher::jnt_state_raw_sub_fun, this, std::placeholders::_1));
    jnt_state_remap_pub_ =
      node_->create_publisher<sensor_msgs::msg::JointState>(
          "remapped_joint_states", 1);
  }
  else
  {
    init_success = false;
  }

  return init_success;
}

void Rviz2JointStatePublisher::jnt_state_raw_sub_fun(
    const sensor_msgs::msg::JointState& msg)
{
  int32_t n_jnt = msg.name.size();

  int32_t jnt_thumb_opp = -1;
  int32_t jnt_index_fle = -1;
  int32_t jnt = 0;
  bool jnt_found = false;

  while (!jnt_found && (jnt < n_jnt))
  {
    if (thumb_jnt_name_ == msg.name[jnt])
    {
      jnt_thumb_opp = jnt;
    }
    else if (index_jnt_name_ == msg.name[jnt])
    {
      jnt_index_fle = jnt;
    }
    else
    {
    }

    if ((jnt_thumb_opp > -1) && (jnt_index_fle > -1))
    {
      jnt_found = true;
    }

    ++jnt;
  }

  if (jnt_found)
  {
    sensor_msgs::msg::JointState jnt_state_remap_msg = msg;

    jnt_state_remap_msg.position[jnt_thumb_opp] = 
      thumb_jnt_opp_->get_opposition_ang(msg.position[jnt_index_fle]);
    jnt_state_remap_msg.position[jnt_index_fle] = 
      std::abs(msg.position[jnt_index_fle]);
    jnt_state_remap_pub_->publish(jnt_state_remap_msg);
  }

  return;
}
}  // namespace

