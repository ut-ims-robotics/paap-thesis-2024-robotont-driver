#include "robotont_driver/hardware.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <rclcpp/logging.hpp>
#include "rclcpp/rclcpp.hpp"

#include <utility>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <functional>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace robotont
{
//Hardware::Hardware(rclcpp::Node::SharedPtr node): rclcpp::Node("hardware"),
Hardware::Hardware(rclcpp::Node::SharedPtr node): 
    m_owned_ctx{new drivers::common::IoContext()},
    m_serial_driver{new drivers::serial_driver::SerialDriver(*m_owned_ctx)},
    node_(node)
{
  RCLCPP_INFO(node_->get_logger(), "Robotont driver is starting...");

  // Get parameters 
  get_params();
<<<<<<< HEAD
  // Initialize
  initialize();
}

void Hardware::initialize()
{

  // Create Publisher
  m_publisher = this->create_publisher<UInt8MultiArray>(
    "serial_read", rclcpp::QoS{100});
 
=======
  
>>>>>>> 52d9cb30cbf21f12a07e4d1429200bc29af459c9
  try {
    m_serial_driver->init_port(m_device_name, *m_device_config);
    if (!m_serial_driver->port()->is_open()) {
      m_serial_driver->port()->open();
      m_serial_driver->port()->async_receive(
        std::bind(&Hardware::receive_callback, this, std::placeholders::_1, std::placeholders::_2));
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      node_->get_logger(), "Error creating serial port: %s - %s",
      m_device_name.c_str(), ex.what());
  }

  cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 
    rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)), 
    std::bind(&Hardware::cmd_vel_callback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "Hardware interface is ready");
}


void Hardware::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg)
{
  RCLCPP_INFO(node_->get_logger(), "got vel cmd");
}

void Hardware::receive_callback(const std::vector<uint8_t> & buffer, const size_t & bytes_transferred)
{
  UInt8MultiArray out;
<<<<<<< HEAD
  RCLCPP_INFO(this->get_logger(), "buffer: %u", buffer);
  RCLCPP_INFO(this->get_logger(), "bytes_transferred: %u", bytes_transferred);
  drivers::common::to_msg(buffer, out, bytes_transferred);
  RCLCPP_INFO(this->get_logger(), "out: %u", out);
  m_publisher->publish(out);

=======
  auto s = std::string(buffer.begin(), buffer.end());
  drivers::common::to_msg(buffer, out, bytes_transferred);
  RCLCPP_INFO(node_->get_logger(), "Got %lu byte(s) from serial: %s", bytes_transferred, s.c_str());
>>>>>>> 52d9cb30cbf21f12a07e4d1429200bc29af459c9
}


void Hardware::subscriber_callback(const UInt8MultiArray::SharedPtr msg)
{
  RCLCPP_INFO(node_->get_logger(), "Subscriber_callback");
}


Hardware::~Hardware()
{
}

void Hardware::get_params()
{
  uint32_t baud_rate{};
  auto fc = drivers::serial_driver::FlowControl::NONE;
  auto pt = drivers::serial_driver::Parity::NONE;
  auto sb = drivers::serial_driver::StopBits::ONE;

  try {
    m_device_name = node_->declare_parameter<std::string>("device_name", "/dev/ttyACM0");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(node_->get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = node_->declare_parameter<int>("baud_rate", 115200);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(node_->get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = node_->declare_parameter<std::string>("flow_control", "none");

    if (fc_string == "none") {
      fc = drivers::serial_driver::FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = drivers::serial_driver::FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = drivers::serial_driver::FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
              "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(node_->get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = node_->declare_parameter<std::string>("parity", "none");

    if (pt_string == "none") {
      pt = drivers::serial_driver::Parity::NONE;
    } else if (pt_string == "odd") {
      pt = drivers::serial_driver::Parity::ODD;
    } else if (pt_string == "even") {
      pt = drivers::serial_driver::Parity::EVEN;
    } else {
      throw std::invalid_argument{
              "The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(node_->get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = node_->declare_parameter<std::string>("stop_bits", "1");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = drivers::serial_driver::StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = drivers::serial_driver::StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = drivers::serial_driver::StopBits::TWO;
    } else {
      throw std::invalid_argument{
              "The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(node_->get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  m_device_config = std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}
}
