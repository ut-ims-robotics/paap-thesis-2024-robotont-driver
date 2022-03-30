#ifndef PLUGIN_BASE_
#define PLUGIN_BASE_
#include <rclcpp/rclcpp.hpp>
#include <robotont_driver/hardware.hpp>

namespace robotont
{
class PluginBase
{
public:
  PluginBase(HardwarePtr hw_ptr, const std::string& name);
  virtual ~PluginBase();

  virtual void initialize();
  virtual void packetReceived(const RobotontPacket& packet);
  virtual const std::string& getName() const;

protected:
  HardwarePtr hw_ptr_;
  std::string name_;
  //ros::NodeHandle nh_;
};

typedef std::shared_ptr<PluginBase> PluginBasePtr;
}  // namespace robotont
#endif
