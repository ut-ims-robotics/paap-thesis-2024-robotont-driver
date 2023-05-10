#include "robotont_driver/plugin_odom.hpp"

using namespace std::chrono_literals;

namespace robotont
{
PluginOdom::PluginOdom(rclcpp::Node::SharedPtr node_) : node_(node_)
{
  RCLCPP_INFO(node_->get_logger(), "Robotont odometry is starting...");
  // Create messages
  odom_msg_ = std::make_unique<nav_msgs::msg::Odometry>();

  // Set default frame names for odom and robot's base.
  odom_msg_->header.frame_id = "odom";
  odom_msg_->child_frame_id = "base_footprint";

  // Initialize odom message
  odom_msg_->header.stamp = node_->now();
  odom_msg_->pose.pose.position.x = 0;
  odom_msg_->pose.pose.position.y = 0;
  odom_msg_->pose.pose.position.z = 0;
  odom_msg_->pose.pose.orientation.x = 0;
  odom_msg_->pose.pose.orientation.y = 0;
  odom_msg_->pose.pose.orientation.z = 0;
  odom_msg_->pose.pose.orientation.w = 1;

  // Initialize odom publisher
  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 2);
}


PluginOdom::~PluginOdom()
{
  //The weak pointer of the node_ is expired by now.
  std::cout << "ODOM SHUTDOWN" << std::endl;
}


void PluginOdom::setOdomFrameName(const std::string& frame_name)
{
  odom_msg_->header.frame_id = frame_name;
}

void PluginOdom::setRobotFrameName(const std::string& frame_name)
{
}

void PluginOdom::update(float pos_x, float pos_y, float ori_z, float lin_vel_x, float lin_vel_y, float ang_vel_z)
{
  auto node_ = weak_node_.lock();
  if(!node_)
  {
    return;
  }

  odom_msg_->header.stamp = node_->now();
  odom_msg_->pose.pose.position.x = pos_x;
  odom_msg_->pose.pose.position.y = pos_y;
  odom_msg_->pose.pose.position.z = 0.0;
  //odom_msg_->pose.pose.orientation = geometry_msgs::msg::Quaternion_<std::allocator<void> >;
  odom_msg_->twist.twist.linear.x = lin_vel_x;
  odom_msg_->twist.twist.linear.y = lin_vel_y;
  odom_msg_->twist.twist.angular.z = ang_vel_z;
}

void PluginOdom::publish()
{
  if (odom_pub_)
  {
    (odom_pub_->publish)(*odom_msg_);
  }
}

} // namespace robotont
