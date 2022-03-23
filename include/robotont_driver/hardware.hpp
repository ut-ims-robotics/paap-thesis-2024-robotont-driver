#ifndef HARDWARE_
#define HARDWARE_
#include <rclcpp/rclcpp.hpp>
#include "robotont_driver/odom.hpp"
#include "io_context/io_context.hpp"
#include "serial_driver/serial_port.hpp"
#include "serial_driver/serial_driver.hpp"
#include "msg_converters/converters.hpp"

using std_msgs::msg::UInt8MultiArray;

namespace drivers
{
namespace serial_driver
{
typedef std::vector<std::string> RobotontPacket;

class Hardware : public rclcpp::Node
{
public:
  Hardware();
  void initialize();
  ~Hardware();

  void read();
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg);
  void get_params();
  /// \breif Callback for when serial data are received
  void receive_callback(const std::vector<uint8_t> & buffer, const size_t & bytes_transferred);
  /// \brief Callback for sending a raw serial message
  void subscriber_callback(const UInt8MultiArray::SharedPtr msg);

private:
  std::unique_ptr<drivers::common::IoContext> m_owned_ctx{};
  std::string m_device_name{};
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> m_device_config;
  std::unique_ptr<drivers::serial_driver::SerialDriver> m_serial_driver;
  rclcpp::Publisher<UInt8MultiArray>::SharedPtr m_publisher;
  rclcpp::Subscription<UInt8MultiArray>::SharedPtr m_subscriber;

  std::string packet_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  bool reconnect_requested_;
}; //class hardware

typedef std::shared_ptr<Hardware> HardwarePtr;

}  // namespace serial_driver
}  // namespace drivers

#endif