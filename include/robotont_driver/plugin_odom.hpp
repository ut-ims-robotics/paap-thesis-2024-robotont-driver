#include "rclcpp/rclcpp.hpp"
#include "robotont_driver/driver_exception.hpp"
#include <nav_msgs/msg/odometry.hpp>
//#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

#ifndef ODOM_HPP
#define ODOM_HPP

/**
 * \brief Odometry class
 * This class holds and publishes robot's odometry together with a corresponding TF
 */
namespace robotont
{
class PluginOdom
{
public:
  /**
   * \brief Constructor
   * \param node_ Shared pointer of the driver node
   */
  PluginOdom(rclcpp::Node::SharedPtr node_);

  /**
   * \brief Destructor
   */
  ~PluginOdom();

  /**
   * \brief Resets odom message values
   */
  void reset();

  /**
   * \brief Receives the packet and transforms it into odom message
   */
  void packetReceived(const std::vector<std::string>& packet);

  /**
   * \brief Publishes the odometry and the TF messages
   */
  void publish();

  /**
   * \brief Updates the odometry
   * \pos_x X-coordinate of the position (m)
   * \pos_y Y-coordinate of the position (m)
   * \ori_z angular position around the Z axis (rad)
   * \lin_vel_x x-coordinate of the linear speed vector (m/s)
   * \lin_vel_y y-coordinate of the linear speed vector (m/s)
   * \ang_vel_z angular speed around the Z axis (rad/s)
   */
  void update(float pos_x, float pos_y, float ori_z, float lin_vel_x, float lin_vel_y, float ang_vel_z);

  /**
   * \brief Sets parent frame name for odom and its tf
   * \param frame_id ID of the frame
   */
  void setFrameId(const std::string& frame_id);

  /**
   * \brief Sets child frame name for odom and its tf
   * \param child_frame_id ID of the child frame
   */
  void setChildFrameId(const std::string& child_frame_id);


private:
  rclcpp::Node::SharedPtr node_;
  /** Pointer to odometry message */
  nav_msgs::msg::Odometry::UniquePtr odom_msg_;

  /** Pointer to transform message */
  geometry_msgs::msg::TransformStamped::UniquePtr odom_transform_;

  /** Pointer to odometry publisher */
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  /** Broadcaster class for publishing transform messages */
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> odom_broadcaster_;

  /** Weak pointer to driver node */
  rclcpp::Node::WeakPtr weak_node_;
};
typedef std::shared_ptr<PluginOdom> OdomPtr;
} // namespace robotont

#endif
