#include <robotont_driver/range_sensor.h>

namespace robotont
{
RangeSensor::RangeSensor()
{
  ROS_DEBUG("Sensors reading init...");

  // Initialize publishers for Range and LaserScan
  range_pub_ = nh_.advertise<sensor_msgs::Range>("range", 1);
  range_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 1);
  
}

RangeSensor::~RangeSensor()
{
}

void RangeSensor::update(const std::vector<float>& measurements)
{ 
  // Clear all previous messages
  range_msgs_.clear();

  sensor_msgs::Range range_msg;
  range_msg.header.stamp = ros::Time::now();
  range_msg.radiation_type = sensor_msgs::Range::INFRARED;
  range_msg.field_of_view = 1.5; //The cone in radians
  range_msg.min_range = 0.0; // min allowed range in meters
  range_msg.max_range = 2.0; // max allowed range in meters

  // Go over each sensor reading and copy it into std_msgs/Range message
  for (int i = 0; i < measurements.size(); i++) 
  {
    // TODO: Auto generate frame ids for each sensor
    // should match with the frames defined in the URDF file
    // range_msgs_.header.frame_id = "range_sensor_X"
    range_msg.range = measurements[i];

    // Add the constructed message to the vector.
    range_msgs_.push_back(range_msg);
  }

  // Build the laserscan message
  // TODO: The following assumes that sensors are spaced equally
  // along the circle! Take into account the actual positions and interpolate.
  laserscan_msg_.header.stamp = ros::Time::now();
  laserscan_msg_.header.frame_id = "base_link";
  laserscan_msg_.angle_min = 0;
  laserscan_msg_.angle_max = 2*M_PI;
  laserscan_msg_.angle_increment = 2 * M_PI / RANGE_SENSOR_COUNT;
  laserscan_msg_.range_min = 0;
  laserscan_msg_.range_max = 2.0;
  laserscan_msg_.ranges = measurements;
}

void RangeSensor::publish()
{
  // Publish all messages added to the vector.
  for (int i = 0; i < range_msgs_.size(); i++) 
  {
    range_pub_.publish(range_msgs_[i]);
  }
}
}
