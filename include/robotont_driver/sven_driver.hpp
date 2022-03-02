#ifndef ROBOTONT_DRIVER__SVEN_DRIVER_HPP_
#define ROBOTONT_DRIVER__SVEN_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include "geometry_msgs/msg/twist.hpp"

#include "robotont_driver/odom.hpp"
#include "io_context/io_context.hpp"
#include "serial_driver/serial_port.hpp"
#include "serial_driver/serial_driver.hpp"
#include "msg_converters/converters.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>


using std_msgs::msg::UInt8MultiArray;

namespace drivers
{
namespace serial_driver
{

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
   */
  //void connect();
  void read();
  //void processPacket();
  //void write(const std::string& packet);
  //void writeRobotSpeed(float lin_vel_x, float lin_vel_y, float ang_vel_z);
//  void writeMotorSpeed(float speed_m1, float speed_m2, float speed_m3);
  //void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg);

  void get_params();

  std::unique_ptr<drivers::common::IoContext> m_owned_ctx{};
  std::string m_device_name{};
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> m_device_config;
  std::unique_ptr<drivers::serial_driver::SerialDriver> m_serial_driver;
  //lc::LifecyclePublisher<UInt8MultiArray>::SharedPtr m_publisher;
  rclcpp::Subscription<UInt8MultiArray>::SharedPtr m_subscriber;


  std::unique_ptr<Odom> odom_;
  //serial::Serial serial_;
  std::string packet_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};  // class robodontDriver

}  // namespace serial_driver
}  // namespace drivers

#endif  // SERIAL_DRIVER__SERIAL_BRIDGE_NODE_HPP_