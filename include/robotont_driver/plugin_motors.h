#ifndef PLUGIN_MOTORS_
#define PLUGIN_MOTORS_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "robotont_driver/plugin_base.h"

/**
 * \brief PluginMotors class
 * This plugin subscribes to cmd_vel topic and relays the velocities to robotont motors.
 */
namespace robotont
{
class PluginMotors : public PluginBase
{
public:
  PluginMotors(RobotontHWPtr hw_ptr, const std::string& name);
  ~PluginMotors();

private:
  void writeRobotSpeed(float lin_vel_x, float lin_vel_y, float ang_vel_z);
  void writeMotorSpeed(float speed_m1, float speed_m2, float speed_m3);
  void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel_msg);

  ros::Subscriber cmd_vel_sub_;
};
}  // namespace robotont
#endif
