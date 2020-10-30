#include "robotont_driver/plugin_range.h"

namespace robotont
{
PluginRange::PluginRange(HardwarePtr hw_ptr, const std::string& name) : PluginBase(hw_ptr, name)
{
  // Get frame names from parameter server
  nh_.param("range/sensor_frame_prefix", sensor_frame_prefix_, std::string("range_sensor_"));

  // Set some non changing fields in the TF and range messages
  range_transform_.header.frame_id = "base_link";

  range_msg_.radiation_type = sensor_msgs::Range::INFRARED;
  range_msg_.field_of_view = 25.0 / 180 * M_PI;
  range_msg_.min_range = 0.04;
  range_msg_.max_range = 4;

  // Initialize range publisher
  range_pub_ = nh_.advertise<sensor_msgs::Range>("range", 2);
}

PluginRange::~PluginRange()
{
}

// clang-format off
//constants definitions
const unsigned int PluginRange::NUM_SENSORS = 12;

// r (mm)  theta (rad) phi (rad)
const float PluginRange::sensor_locations_[][3] =
  {
    {163, 0.593, 0.52},
    {176, 1.047, 1.04},
    {163, 1.501, 1.57},
    {165, 2.094, 2.09},
    {161, 2.548, 2.63},
    {136, 3.142, 3.14},
    {162, 3.752, 3.66},
    {165, 4.206, 4.18},
    {163, 4.800, 4.71},
    {176, 5.253, 5.27},
    {163, 5.707, 5.76},
    {165, 0.0  , 0.0}
  };
// clang-format on

void PluginRange::packetReceived(const RobotontPacket& packet)
{
  // Check that we have data for all the sensors and that the packet is of type RANGE
  if (packet.size() != NUM_SENSORS + 1 || packet[0] != "RANGE")
  {
    return;
  }

  float ranges[NUM_SENSORS]; // Array for range readings
  try
  {
    for (int i = 0; i < NUM_SENSORS; i++)
    {
      ranges[i] = std::stof(packet[i + 1]) * 0.001;  // translate from str (mm) to float (meters)
    }
  }
  catch (std::exception e)
  {
    ROS_ERROR_THROTTLE(1, "Failed to parse range packet: %s", e.what());
  }

  for (int i = 0; i < NUM_SENSORS; i++)
  {
    float r = PluginRange::sensor_locations_[i][0] * 0.001;  // distance from robot's center to sensor in meters
    float theta = PluginRange::sensor_locations_[i][1];      // angle between sensor position and forward direction
    float phi = PluginRange::sensor_locations_[i][2];        // sensor orientation relative to the forward direction
    float x = r * std::cos(theta);                           // x distance from robot's center to sensor in meters
    float y = -r * std::sin(theta);                          // y distance from robot's center to sensor in meters
    std::string sensor_frame_id = sensor_frame_prefix_ + std::to_string(i);

    // Create message for TF
    geometry_msgs::Quaternion range_quat = tf::createQuaternionMsgFromYaw(-phi);
    range_transform_.header.stamp = ros::Time::now();
    range_transform_.child_frame_id = sensor_frame_id;
    range_transform_.transform.translation.x = x;
    range_transform_.transform.translation.y = y;
    range_transform_.transform.translation.z = 0.0;
    range_transform_.transform.rotation = range_quat;

    // Create the Range message
    range_msg_.header.stamp = ros::Time::now();
    range_msg_.header.frame_id = sensor_frame_id;
    range_msg_.range = ranges[i];

    // Publish TF and range messages
    range_pub_.publish(range_msg_);
    range_broadcaster_.sendTransform(range_transform_);
  }
}

}  // namespace robotont
