#include "robotont_driver/plugin_led_module.hpp"

using namespace std::chrono_literals;

namespace robotont
{
PluginLedModule::PluginLedModule(HardwarePtr hw_ptr, rclcpp::Node::SharedPtr node_) : hw_ptr_(hw_ptr), node_(node_)
{
  RCLCPP_INFO(node_->get_logger(), "Robotont LED module is starting...");

  // Subscribe to led_pixel topic
   led_pixel_sub_ = node_->create_subscription<RobotontPacket>("led_pixel", 
    rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)), 
    std::bind(&PluginLedModule::pixel_callback, this, std::placeholders::_1));
}

PluginLedModule::~PluginLedModule()
{
}

// Callback function to read data from cmd_vel topic
void PluginLedModule::pixel_callback(const RobotontPacket::SharedPtr led_seg_msg)
{
  writeRobotSpeed(cmd_vel_msg->linear.x, cmd_vel_msg->linear.y, cmd_vel_msg->angular.z);
}

// Function to create the string packet from cmd_vel topic and send it to serial
void PluginLedModule::writeRobotSpeed(float lin_vel_x, float lin_vel_y, float ang_vel_z)
{
  std::string packet = "RS:"+std::to_string(lin_vel_x)+":"+std::to_string(lin_vel_y)+":"+std::to_string(ang_vel_z)+"\r\n";
  if (hw_ptr_)
  {
    hw_ptr_->subscriber_callback(packet);
  }
}


} // namespace robotont
