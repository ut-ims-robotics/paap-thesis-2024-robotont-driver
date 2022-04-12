#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
//#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
//#include "tf2_msgs/msg/tf_message.hpp"
#include "robotont_driver/driver_exception.hpp"

#ifndef ODOM_HPP
#define ODOM_HPP

/**
 * \brief Odometry class
 * This class holds and publishes robot's odometry together with a corresponding TF
 */
namespace robotont
{
class Odom
{
public:
  /**
   * \brief Constructor
   * \param node_ Shared pointer of the driver node
   */
  Odom(rclcpp::Node::SharedPtr node_);

  /**
   * \brief Destructor
   */
  ~Odom();

  
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
   * \brief Sets the name of the odom frame
   * \param frame_name Name of the odom frame
   */
  void setOdomFrameName(const std::string& frame_name);

  /**
   * \brief Sets the name of the robot's base frame
   * \param frame_name Name of the robot's base frame
   */
  void setRobotFrameName(const std::string& frame_name);


private:
  
  /** Pointer to odometry message */
  //nav_msgs::msg::Odometry::UniquePtr odom_msg_;

  /** Pointer to transform message */
  //geometry_msgs::msg::TransformStamped::SharedPtr odom_transform_;

  /** Pointer to odometry publisher */
  //rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  /** Broadcaster class for publishing TF messages */
  //rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;

  /** Weak pointer to driver node */
  //rclcpp::Node::WeakPtr node_;
};
} // namespace robotont

#endif
