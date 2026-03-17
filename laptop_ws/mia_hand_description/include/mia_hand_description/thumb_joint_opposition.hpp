#ifndef MIA_HAND_DESCRIPTION_THUMB_JOINT_OPPOSITION_HPP
#define MIA_HAND_DESCRIPTION_THUMB_JOINT_OPPOSITION_HPP

#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "urdf/model.h"
#include "joint_limits/joint_limits_urdf.hpp"

namespace mia_hand_description
{
class ThumbJointOpposition
{
public:

  /**
   * \brief Factory function for creating a new ThumbOpposition object.
   *
   * A factory function is used for avoiding to throw exceptions in the class
   * constructor, in case of errors during object initialization.
   *
   * @param node Shared pointer to ROS node where to get thumb joint limits and 
   *             parameters from.
   * @param prefix Prefix added before link and joint names in the URDF.
   * @return \code std::unique_ptr \endcode to the new ThumbOpposition object.
   */
  static std::unique_ptr<ThumbJointOpposition> create(
      std::shared_ptr<rclcpp::Node> node, const std::string& prefix);

  /* Preventing ThumbOpposition objects to be copyable, either through the copy
   * constructor and the assignment operator, to ensure the unique access to
   * them through the std::unique_ptr.
   */
  ThumbJointOpposition(ThumbJointOpposition&) = delete;
  ThumbJointOpposition& operator=(ThumbJointOpposition&) = delete;

  /**
   * \brief Function for mapping the thumb joint opposition from the index joint 
   *        angle.
   *
   * @param index_jnt_ang Index joint angle, in rad.
   * @return Thumb opposition angle, in rad.
   */
  double get_opposition_ang(double index_jnt_ang);

private:

  /**
   * \brief Class constructor.
   *
   * The class constructor is private to allow instantiations of new
   * ThumbOpposition objects only through #create() factory function.
   */
  ThumbJointOpposition();

  /**
   * \brief Function for initializing a ThumbOpposition object.
   *
   * This function is necessary for accessing all the non-static members through
   * the static factory function #create().
   *
   * @param node Shared pointer to ROS node where to get thumb joint limits and 
   *             parameters from.
   * @param prefix Prefix added before link and joint names in the URDF.
   * @return \code true \endcode if object initialization is successful, 
   *         \code false \endcode if not.
   */
  bool init(std::shared_ptr<rclcpp::Node> node, const std::string& prefix);

  /**
   * \brief Function for loading the thumb joint limits from the URDF, given as
   *        parameter to the node.
   *
   * @param node Shared pointer to ROS node where to get joint limits from.
   * @param prefix Prefix added before link and joint names in the URDF.
   * @return \code true \endcode if thumb joint limits loading was successful,
   *         \code false \endcode if not.
   */
  bool load_jnt_limits_from_urdf(
      std::shared_ptr<rclcpp::Node> node, const std::string& prefix);

  double thumb_jnt_min_pos_;  //!< Thumb opposition minimum position.
  double thumb_jnt_max_pos_;  //!< Thumb opposition maximum position.

  double thumb_opp_start_index_ang_;  //!< Index joint angle at which thumb opposition should start.
  double thumb_opp_stop_index_ang_;   //!< Index joint angle at which thumb opposition should stop.

  double thumb_opp_scale_;   //!< Used for opposition angle computation.
  double thumb_opp_offset_;  //!< Used for opposition angle computation.
};
}  // namespace

#endif  // MIA_HAND_DESCRIPTION_THUMB_JOINT_OPPOSITION_HPP

