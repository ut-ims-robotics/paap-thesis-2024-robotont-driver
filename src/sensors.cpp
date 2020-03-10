// sensors node

#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>

Sensor::Sensor()
{
  ROS_DEBUG("Sensors reading init...");

  // Initialize sensors publisher for Range
  sensor_pub_ = nh_.advertise<sensor_msgs::Range>("sensor", 3); //1= command velocity topic, 2= Odometry topic, 3 empty ??
}


Sensor::~Sensor()
{
}

void Sensor::update(int measurements[]) //Siin saab sensori n2idud l2bi RobotontHW::processPacket()-i (?)
{ 
  for (i = 0; i < sizeof(measurements); i++) 
  {
    
  }
  //siin
}
/* kuidas teha nii, et NUCi terminalis funktsiooni argumendid anda ledide muutmiseks?
void RobotontHW::writeLED_State
void RobotontHW::writeLED_SEG
*/

void Sensor::publish()
{
  sensor_pub_.publish(sensor_msg_); //Siit peaks publishima sensori n2idud NUCi seriali?
  //sensor_broadcaster_.sendTransform(sensor_transform_);
}
