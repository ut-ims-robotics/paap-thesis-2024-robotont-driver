#include "robotont_driver/sven_driver.hpp"

#include <memory>
#include <string>
#include <vector>


namespace drivers
{
namespace serial_driver
{

RobotontDriver::RobotontDriver() : Node("robotont_driver_node"),
    m_owned_ctx{new IoContext()},
    m_serial_driver{new SerialDriver(*m_owned_ctx)},
    odom_(nullptr)
{

    RCLCPP_INFO(this->get_logger(), "Robotont driver is starting...");
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&RobotontDriver::read, this) );
}

void RobotontDriver::initialize()
{
  try
  {
  odom_ = std::make_unique<Odom>(this->shared_from_this());
//  RCLCPP_INFO(this->get_logger(), "Initialize(): After constructing odom");
  }
  catch (DriverException e)
  {
    RCLCPP_ERROR(this->get_logger(), e.what());
  }
}

void RobotontDriver::read()
{
    RCLCPP_INFO(this->get_logger(), "Hello from read()");
}

RobotontDriver::~RobotontDriver()
{
  RCLCPP_INFO(this->get_logger(), "Robotont driver is shutting down...");
}

}  // namespace serial_driver
}  // namespace drivers

int main(int argc, char **argv)
{
     /*
    rclcpp::init(argc, argv);
    auto node = std::make_shared<drivers::serial_driver::RobotontDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
    */

    // Initialise ROS2
    rclcpp::init(argc, argv);

    // Create a single threaded executor
    rclcpp::executors::SingleThreadedExecutor exec;

    // Create driver and odom nodes
    auto driver_node = std::make_shared<drivers::serial_driver::RobotontDriver>();
    driver_node->initialize();

    // Add nodes to the executor
    exec.add_node(driver_node);

    // Spin
    exec.spin();

    // Do a nice shut down for ROS2
    rclcpp::shutdown();
    return 0;
    
}