#ifndef PLUGIN_RANGE_
#define PLUGIN_RANGE_

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include "robotont_driver/plugin_base.h"

/**
 * \brief PluginRange class
 * This plugin parses the RANGE packet and translates it to ROS range messages.
 */
namespace robotont
{
class PluginRange : public PluginBase
{
public:
  PluginRange(HardwarePtr hw_ptr, const std::string& name);
  ~PluginRange();
  void packetReceived(const RobotontPacket& packet);

  const static float sensor_locations_[][3];
  const static unsigned int NUM_SENSORS;

private:
  sensor_msgs::Range range_msg_;
  geometry_msgs::TransformStamped range_transform_;
  ros::NodeHandle nh_;
  ros::Publisher range_pub_;
  tf::TransformBroadcaster range_broadcaster_;
  std::string sensor_frame_prefix_;
};

}  // namespace robotont
#endif
