 #ifndef HARDWARE_
#define HARDWARE_
#include <rclcpp/rclcpp.hpp>
//#include <serial/serial.h>

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

  /**
   * \brief Reads the packet from the robot
   * \param packet The packet contents.
   * \return True if the complete packet was placed in \packet packet variable, False if no data has arrived or the
   * packet is not yet complete
   */
  //bool readPacket(RobotontPacket& packet);
  //void writePacket(const RobotontPacket& packet);
  void read();
  void get_params();
  /// \breif Callback for when serial data are received
  void receive_callback(const std::vector<uint8_t> & buffer, const size_t & bytes_transferred);

private:
  //void connect();
  //void checkConnection(const ros::TimerEvent& event);

  std::unique_ptr<drivers::common::IoContext> m_owned_ctx{};
  std::string m_device_name{};
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> m_device_config;
  std::unique_ptr<drivers::serial_driver::SerialDriver> m_serial_driver;
  rclcpp::Publisher<UInt8MultiArray>::SharedPtr m_publisher;
  rclcpp::Subscription<UInt8MultiArray>::SharedPtr m_subscriber;


  //std::unique_ptr<Odom> odom_;
  //serial::Serial serial_;
  std::string packet_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  //ros::Timer timer_;
  bool reconnect_requested_;
}; //class hardware

typedef std::shared_ptr<Hardware> HardwarePtr;

}  // namespace serial_driver
}  // namespace drivers

#endif