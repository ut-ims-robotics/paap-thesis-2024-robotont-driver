#include "robotont_driver/driver.hpp"
#include "robotont_driver/hardware.hpp"


namespace drivers
{
namespace serial_driver
{
  Driver::Driver() : Node("driver_node")
  {
    hw_ptr_ = std::make_shared<Hardware>();
  }

  Driver::~Driver()
  {
  }
}  // namespace serial_driver
}  // namespace drivers
