#include "robotont_driver/robotont_driver.h"
#include "robotont_driver/plugin_motors.h"
#include "robotont_driver/plugin_odom.h"

namespace robotont
{
  RobotontDriver::RobotontDriver()
  {
    // Create a hardware instance
    hw_ptr_ = std::make_shared<RobotontHW>();

    // initialize builtin plugins
    plugins_.emplace_back(std::make_shared<PluginMotors>(hw_ptr_, "Motors"));
    plugins_.emplace_back(std::make_shared<PluginOdom>(hw_ptr_, "Odometry"));

    // Here we load all the possible plugins
    for (auto plugin : plugins_)
    {
      if (plugin)
      {
        ROS_DEBUG("Initializing plugin: '%s'.", plugin->getName().c_str());
        plugin->initialize();
      }
    }

    // Create a timer to periodically read data from the robot
    timer_ = nh_.createTimer(ros::Duration(0.01), &RobotontDriver::update, this);
  }

  RobotontDriver::~RobotontDriver()
  {
  }

  void RobotontDriver::update(const ros::TimerEvent& event)
  {
    // Poll serial
    RobotontPacket packet;
    if (hw_ptr_)
    {
      // Check if a packet is arrived from the robot
      if(hw_ptr_->readPacket(packet))
      {
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
  }
}  // namespace robotont
