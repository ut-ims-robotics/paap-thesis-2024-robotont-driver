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

    // load here all plugins
    // ....
  }

  Driver::~Driver()
  {
  }

}
//}  // namespace serial_driver
//}  // namespace drivers
