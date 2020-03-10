#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>

#define RANGE_SENSOR_COUNT 12

namespace robotont
{
class RangeSensor
{
public:
  RangeSensor();
  ~RangeSensor();

  void update(const std::vector<float>& measurements);
  void publish();

private:
  std::vector<sensor_msgs::Range> range_msgs_;
  sensor_msgs::LaserScan laserscan_msg_;
  ros::NodeHandle nh_;
  ros::Publisher range_pub_;
  ros::Publisher laser_pub_;
};
}
