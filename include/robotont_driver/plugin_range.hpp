#ifndef PLUGIN_RANGE_
#define PLUGIN_RANGE_

#include "rclcpp/rclcpp.hpp"
#include "robotont_driver/driver_exception.hpp"
#include "robotont_driver/plugin_base.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/Range.hpp"
#include "tf2_ros/static_transform_broadcaster.h"


/**
 * \brief PluginRange class
 * This plugin parses the RANGE packet and translates it to ROS range messages.
 */
namespace robotont
{
class PluginRange
{
public:
  PluginRange(HardwarePtr hw_ptr, rclcpp::Node::SharedPtr node_);
  ~PluginRange();

  void packetReceived(const RobotontPacket& packet);

  const static float sensor_locations_[][3];
  const static unsigned int NUM_SENSORS;

private:
  
};

typedef std::shared_ptr<PluginRange> LedModulePtr;
}  // namespace robotont
#endif