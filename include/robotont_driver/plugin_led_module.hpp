#include "rclcpp/rclcpp.hpp"
#include "robotont_driver/driver_exception.hpp"
#include "robotont_driver/plugin_base.hpp"

#include <robotont_msgs/LedModulePixel.h> 
#include <robotont_msgs/LedModuleSegment.h>
//TODO: above packages don't exist for ros2 rolling

#ifndef LED_HPP
#define LED_HPP


using namespace std::chrono_literals;
/**
 * \brief PluginLedModule class
 * This plugin subscribes to led_pixel and led_segment topics, which can be used to set the led module color.
 */
namespace robotont
{
class PluginLedModule
{
public:
  PluginLedModule(HardwarePtr hw_ptr, rclcpp::Node::SharedPtr node_);
  ~PluginLedModule();

private:
  rclcpp::Node::SharedPtr node_;
  HardwarePtr hw_ptr_;

  void writePixel(unsigned int idx, uint8_t r, uint8_t g, uint8_t b);
  void writeSegment(const robotont_msgs::LedModuleSegment& led_seg_msg);
  void pixel_callback(const robotont_msgs::LedModuleSegment& led_px_msg);
  void segment_callback(const robotont_msgs::LedModuleSegment& led_seg_msg);

  rclcpp::Subscription<RobotontPacket>::SharedPtr led_pixel_sub_; //TODO: fix issue with not being able to access RobotontPacket within the same namespace. When done, also update other classes to use RobotontPacket.
  rclcpp::Subscription<RobotontPacket>::SharedPtr led_segment_sub_;
};

typedef std::shared_ptr<PluginLedModule> LedModulePtr;
}  // namespace robotont
#endif