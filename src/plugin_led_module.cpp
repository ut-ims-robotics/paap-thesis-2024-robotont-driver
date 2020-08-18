#include "robotont_driver/plugin_led_module.h"

namespace robotont
{
PluginLedModule::PluginLedModule(RobotontHWPtr hw_ptr, const std::string& name) : PluginBase(hw_ptr, name)
{
  // Subscribe to led topic
  led_px_sub_ = nh_.subscribe("led_pixel", 1, &PluginLedModule::pixel_callback, this);
  led_seg_sub_ = nh_.subscribe("led_segment", 1, &PluginLedModule::segment_callback, this);
}

PluginLedModule::~PluginLedModule()
{
}

void PluginLedModule::writePixel(unsigned int idx, uint8_t r, uint8_t g, uint8_t b)
{
  RobotontPacket packet;
  packet.push_back("LED");
  packet.push_back(std::to_string(idx));
  unsigned int color_combined = 0x0000;
  color_combined += (r << 16);
  color_combined += (g << 8);
  color_combined += (b << 0);
  packet.push_back(std::to_string(color_combined));
  if (hw_ptr_)
  {
    hw_ptr_->writePacket(packet);
  }
}

void PluginLedModule::writeSegment(const robotont_msgs::LedModuleSegment& led_seg_msg)
{
  RobotontPacket packet;
  packet.push_back("LED");
  packet.push_back(std::to_string(led_seg_msg.idx_start));
  for (auto& color : led_seg_msg.colors)
  {
    unsigned int color_combined = 0x0000;
    color_combined += (color.r << 16);
    color_combined += (color.g << 8);
    color_combined += (color.b << 0);
    packet.push_back(std::to_string(color_combined));
  }
  if (hw_ptr_)
  {
    hw_ptr_->writePacket(packet);
  }
}

void PluginLedModule::pixel_callback(const robotont_msgs::LedModulePixel& led_px_msg)
{
  writePixel(led_px_msg.idx, led_px_msg.color.r, led_px_msg.color.g, led_px_msg.color.b);
}

void PluginLedModule::segment_callback(const robotont_msgs::LedModuleSegment& led_seg_msg)
{
  writeSegment(led_seg_msg);
}
} // namespace robotont
