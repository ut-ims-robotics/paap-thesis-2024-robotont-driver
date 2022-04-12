#ifndef DRIVER_
#define DRIVER_


#include <rclcpp/rclcpp.hpp>
#include "robotont_driver/hardware.hpp"
#include "robotont_driver/plugin_base.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include "geometry_msgs/msg/twist.hpp"

#include "robotont_driver/odom.hpp"
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
  ~Driver();

private:
  //void update(const ros::TimerEvent& event);
  void update();

  //ros::NodeHandle nh_;
  HardwarePtr hw_ptr_;
  std::vector<PluginBasePtr> plugins_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace robotont
#endif
