#include "robotont_driver/driver.hpp"
#include "robotont_driver/hardware.hpp"
#include "robotont_driver/plugin_odom.hpp"
#include "robotont_driver/plugin_motors.hpp"


//namespace drivers
//{
//namespace serial_driver
//{
namespace robotont
{
  Driver::Driver() : Node("driver_node")
  {
  }
  
  void Driver::initialize()
  {
    // Create a shared pointer of this node to allow ros functionality in subclasses
    auto node_ptr = shared_from_this();
    hw_ptr_ = std::make_shared<Hardware>(node_ptr);

    // Retrieve the parameter from the parameter server
    bool plugin_odom;
    bool plugin_motor;
    bool plugin_led_module;
    bool plugin_power_supply;
    bool plugin_range;
    try {
      plugin_odom = declare_parameter<bool>("plugin_odom", true);
      plugin_motor = declare_parameter<bool>("plugin_motor", true);
      plugin_led_module = declare_parameter<bool>("plugin_led_module", true);
      plugin_power_supply = declare_parameter<bool>("plugin_power_supply", true);
      plugin_range = declare_parameter<bool>("plugin_range", true);
    } catch (rclcpp::ParameterTypeException & ex) {
      RCLCPP_ERROR(get_logger(), "A provided plugin param was invalid");
      throw ex;
    }
    
    // Initialialize plugins as defined by parameters
    if (plugin_odom) {
      odom_ptr_ = std::make_shared<PluginOdom>(node_ptr);
    }
    if (plugin_motor) {
      motor_ptr_ = std::make_shared<PluginMotors>(hw_ptr_, node_ptr);
    }
    
    // Create timer to read data from the robot 
    timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&Driver::update_packet, this));
  }

  void Driver::update_packet()
  {
    hw_ptr_->get_packet(driver_packets);
    for (auto packet : driver_packets)
    {
      odom_ptr_->packetReceived(packet);
      for (auto arg : packet)
      {
        //RCLCPP_INFO(this->get_logger(), "Received packet content: %s", arg.c_str());
      }
    }
    
    //RCLCPP_INFO(this->get_logger(), "update_packet: %s", hardware_packet);
  }

  Driver::~Driver()
  {
  }

}
//}  // namespace serial_driver
//}  // namespace drivers