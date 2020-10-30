#ifndef DRIVER_
#define DRIVER_

#include <ros/ros.h>

#include "robotont_driver/hardware.h"
#include "robotont_driver/plugin_base.h"

namespace robotont
{
class Driver
{
public:
  Driver();
  ~Driver();

private:
  void update(const ros::TimerEvent& event);

  ros::NodeHandle nh_;
  HardwarePtr hw_ptr_;
  std::vector<PluginBasePtr> plugins_;
  ros::Timer timer_;
};
}  // namespace robotont
#endif
