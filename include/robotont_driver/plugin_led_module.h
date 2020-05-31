#ifndef PLUGIN_LED_MODULE_
#define PLUGIN_LED_MODULE_

#include <ros/ros.h>
#include <robotont_msgs/LedModulePixel.h>
#include <robotont_msgs/LedModuleSegment.h>
#include "robotont_driver/plugin_base.h"

/**
 * \brief PluginLedModule class
 * This plugin subscribes to led_pixel and led_segment topics, which can be used to set the led module color
 */
namespace robotont
{
class PluginLedModule : public PluginBase
{
public:
  PluginLedModule(RobotontHWPtr hw_ptr, const std::string& name);
  ~PluginLedModule();

private:
  void writePixel(unsigned int idx, uint8_t r, uint8_t g, uint8_t b);
  void writeSegment(const robotont_msgs::LedModuleSegment& led_seg_msg);
  void pixel_callback(const robotont_msgs::LedModulePixel& led_px_msg);
  void segment_callback(const robotont_msgs::LedModuleSegment& led_seg_msg);

  ros::Subscriber led_px_sub_;
  ros::Subscriber led_seg_sub_;
};
}  // namespace robotont
#endif
