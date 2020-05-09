#include "robotont_driver/plugin_motors.h"

namespace robotont
{
PluginMotors::PluginMotors(RobotontHWPtr hw_ptr, const std::string& name) : PluginBase(hw_ptr, name)
{
  // Subscribe to command velocity topic
  cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &PluginMotors::cmd_vel_callback, this);
}

PluginMotors::~PluginMotors()
{
}

void PluginMotors::writeMotorSpeed(float speed_m1, float speed_m2, float speed_m3)
{
  RobotontPacket packet;
  packet.push_back("MS");
  packet.push_back(std::to_string(speed_m1));
  packet.push_back(std::to_string(speed_m2));
  packet.push_back(std::to_string(speed_m3));
  if (hw_ptr_)
  {
    hw_ptr_->writePacket(packet);
  }
}

void PluginMotors::writeRobotSpeed(float lin_vel_x, float lin_vel_y, float ang_vel_z)
{
  RobotontPacket packet;
  packet.push_back("RS");
  packet.push_back(std::to_string(lin_vel_x));
  packet.push_back(std::to_string(lin_vel_y));
  packet.push_back(std::to_string(ang_vel_z));
  if (hw_ptr_)
  {
    hw_ptr_->writePacket(packet);
  }
}

void PluginMotors::cmd_vel_callback(const geometry_msgs::Twist& cmd_vel_msg)
{
//  ROS_INFO_STREAM("I heard: \r\n" << cmd_vel_msg);
  writeRobotSpeed(cmd_vel_msg.linear.x, cmd_vel_msg.linear.y, cmd_vel_msg.angular.z);
}
} // namespace robotont
