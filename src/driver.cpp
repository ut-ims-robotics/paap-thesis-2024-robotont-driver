#include "robotont_driver/driver.hpp"
#include "robotont_driver/hardware.hpp"


//namespace drivers
//{
//namespace serial_driver
//{
namespace robotont
{
  Driver::Driver() : Node("driver_node")
  {
  }
  
  void Driver::initialize()
  {
    // Create a shared pointer of this node to allow ros functionality in subclasses
    auto node_ptr = shared_from_this();
    hw_ptr_ = std::make_shared<Hardware>(node_ptr);

    
    timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&Driver::update_packet, this));
  }

  void Driver::update_packet()
  {
    hardware_packet = hw_ptr_->get_packet();
    for (auto arg : hardware_packet)
    {
      RCLCPP_INFO(this->get_logger(), "update_packet %s", arg.c_str());
    }
    //RCLCPP_INFO(this->get_logger(), "update_packet: %s", hardware_packet);
  }

  Driver::~Driver()
  {
  }

}
//}  // namespace serial_driver
//}  // namespace drivers
