#include "robotont_driver/plugin_base.hpp"

namespace drivers
{
namespace serial_driver
{
PluginBase::PluginBase(HardwarePtr hw_ptr, const std::string& name) : hw_ptr_(hw_ptr), name_(name)
{
}

PluginBase::~PluginBase()
{
}

void PluginBase::initialize()
{
}

void PluginBase::packetReceived(const RobotontPacket& packet)
{
}

const std::string& PluginBase::getName() const
{
  return name_;
}
}  // namespace serial_driver
}  // namespace drivers