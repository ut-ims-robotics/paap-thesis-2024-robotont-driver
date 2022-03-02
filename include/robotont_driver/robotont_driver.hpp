/* 
#ifndef ROBOTONT_DRIVER_HPP
#define ROBOTONT_DRIVER_HPP

#include <rclcpp/rclcpp.hpp>
//#include <serial/serial.h>
#include "geometry_msgs/msg/twist.hpp"
#include "robotont_driver/odom.hpp"

//namespace robotont
//{
class RobotontDriver : public rclcpp::Node
{
public:
  RobotontDriver();
  void initialize();
  ~RobotontDriver();

private:
  /**
   * \brief Open serial connection to the robot
   * This function blocks execution until the connection is established or the node is terminated
   
  void connect();
  void read();
  void processPacket();
  void write(const std::string& packet);
  void writeRobotSpeed(float lin_vel_x, float lin_vel_y, float ang_vel_z);
//  void writeMotorSpeed(float speed_m1, float speed_m2, float speed_m3);
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg);

  std::unique_ptr<Odom> odom_;
  serial::Serial serial_;
  std::string packet_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};
//}
#endif
*/

