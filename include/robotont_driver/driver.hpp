#ifndef DRIVER_
#define DRIVER_


#include <rclcpp/rclcpp.hpp>
#include "robotont_driver/hardware.hpp"
#include "robotont_driver/plugin_base.hpp"
#include "robotont_driver/plugin_odom.hpp"
#include "robotont_driver/plugin_motors.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include "geometry_msgs/msg/twist.hpp"

#include "io_context/io_context.hpp"
#include "serial_driver/serial_port.hpp"
#include "serial_driver/serial_driver.hpp"
#include "msg_converters/converters.hpp"
#include "robotont_driver/hardware.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace robotont
{
class Driver : public rclcpp::Node
{    
public:
  Driver();
  void initialize();
  void update_packet();
  ~Driver();

  std::vector<std::vector<std::string>> driver_packets;

private:
  //void update(const ros::TimerEvent& event);
  void update();

  //ros::NodeHandle nh_;
  HardwarePtr hw_ptr_;
  OdomPtr odom_ptr_;
  MotorsPtr motor_ptr_;

  std::vector<std::string> hardware_packet_;
  std::vector<PluginBasePtr> plugins_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace robotont
#endif
