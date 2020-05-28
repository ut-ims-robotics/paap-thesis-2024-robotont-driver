#ifndef PLUGIN_BASE_
#define PLUGIN_BASE_

#include <ros/ros.h>
#include <robotont_driver/robotont_hardware.h>

namespace robotont
{
class PluginBase
{
public:
  PluginBase(RobotontHWPtr hw_ptr, const std::string& name);
  virtual ~PluginBase();

  virtual void initialize(); // TODO: should this method be pure virtual?
  virtual void packetReceived(const RobotontPacket& packet); // TODO: should this method be pure virtual?
  virtual const std::string& getName() const;

protected:
  RobotontHWPtr hw_ptr_;
  std::string name_;
  ros::NodeHandle nh_;
};

typedef std::shared_ptr<PluginBase> PluginBasePtr;
}  // namespace robotont
#endif
