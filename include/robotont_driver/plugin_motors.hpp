#include "rclcpp/rclcpp.hpp"
#include "robotont_driver/driver_exception.hpp"
#include <nav_msgs/msg/odometry.hpp>
//#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "robotont_driver/plugin_base.hpp"

#ifndef MOTORS_HPP
#define MOTORS_HPP


using namespace std::chrono_literals;
/**
 * \brief PluginMotors class
 * This plugin subscribes to cmd_vel topic and relays the velocities to robotont motors.
 */
namespace robotont
{
class PluginMotors
{
public:
  PluginMotors(HardwarePtr hw_ptr, rclcpp::Node::SharedPtr node_);
  ~PluginMotors();

private:
  rclcpp::Node::SharedPtr node_;
  HardwarePtr hw_ptr_;

  void writeRobotSpeed(float lin_vel_x, float lin_vel_y, float ang_vel_z);
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

typedef std::shared_ptr<PluginMotors> MotorsPtr;
}  // namespace robotont
#endif