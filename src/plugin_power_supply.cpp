#include "robotont_driver/plugin_power_supply.h"

namespace robotont
{
PluginPowerSupply::PluginPowerSupply(HardwarePtr hw_ptr, const std::string& name)
: PluginBase(hw_ptr, name)
{
  // Set up the power info publisher
  power_supply_pub_ = nh_.advertise<robotont_msgs::PowerSupply>("power_supply", 1);
}

PluginPowerSupply::~PluginPowerSupply()
{
}

void PluginPowerSupply::packetReceived(const RobotontPacket& packet)
{
  if (packet.size() != 3 || packet[0] != "POWER")
  {
    return;
  }

  robotont_msgs::PowerSupply power_supply_msg;
  try
  {
    power_supply_msg.current = std::stof(packet[1]);
    power_supply_msg.voltage = std::stof(packet[2]);
    power_supply_pub_.publish(power_supply_msg);
  }
  catch (std::exception e)
  {
    ROS_ERROR_THROTTLE(1, "Failed to parse power info packet: %s", e.what());
  }
}

const std::string& PluginPowerSupply::getName() const
{
  return name_;
}

}  // namespace robotont
