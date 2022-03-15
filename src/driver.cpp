#include "robotont_driver/driver.hpp"
#include <rclcpp/logging.hpp>
#include "robotont_driver/hardware.hpp"
//#include "robotont_driver/plugin_odom.h"


namespace drivers
{
namespace serial_driver
{
  Driver::Driver() : Node("driver_node")
  {

    // Create a hardware instance
    //Hardware hw;
    //hw.initialize();
    hw_ptr_ = std::make_shared<Hardware>();
    

    // Initialize builtin plugins
    //plugins_.emplace_back(std::make_shared<PluginOdom>(hw_ptr_, "Odometry"));
    
    // Here we load all the possible plugins
    /*
    for (auto plugin : plugins_)
    {
      if (plugin)
      {
        ROS_INFO("Initializing plugin: '%s'.", plugin->getName().c_str());
        plugin->initialize();
      }
    }
    */

    // Create a timer to periodically read data from the robot
    //timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&Driver::update, this) );
  }

  Driver::~Driver()
  {
  }

  void Driver::update()
  {
    //RCLCPP_INFO(this->get_logger(), "Hello from update()");  
    /*
    // Poll serial
    RobotontPacket packet;
    if (hw_ptr_)
    {
      // Check if a packet is arrived from the robot
      if(hw_ptr_->readPacket(packet))
      {
        for (auto arg : packet)
        {
          ROS_DEBUG_STREAM(arg);
        }
        // We have received the complete packet
        // Process the packet in each loaded plugin
        for (auto& plugin : plugins_)
        {
          if(plugin)
          {
            plugin->packetReceived(packet);
          }
        }
      }
    }
    */
  }
}  // namespace serial_driver
}  // namespace drivers
