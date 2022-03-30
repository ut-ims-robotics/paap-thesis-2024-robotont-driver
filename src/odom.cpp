#include "robotont_driver/odom.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

namespace
{
Odom::Odom(rclcpp::Node::SharedPtr node) : node_(node)
{
  auto node_ptr = node_.lock();
  if(!node_ptr)
  {
    throw DriverException("Odom: unable to get shared pointer of the node");
  }

  RCLCPP_INFO(node_ptr->get_logger(), "Robotont odometry is starting...");
  // Create messages
  odom_msg_ = std::make_unique<nav_msgs::msg::Odometry>();
  odom_transform_ = std::make_shared<geometry_msgs::msg::TransformStamped>();

  // Set default frame names for odom and robot's base.
  odom_msg_->header.frame_id = "odom";
  odom_msg_->child_frame_id = "base_footprint";

  odom_transform_->header.frame_id = odom_msg_->header.frame_id;
  odom_transform_->child_frame_id = odom_msg_->child_frame_id;

  // Initialize odom message
  odom_msg_->header.stamp = node_ptr->now();
  odom_msg_->pose.pose.position.x = 0;
  odom_msg_->pose.pose.position.y = 0;
  odom_msg_->pose.pose.position.z = 0;
  odom_msg_->pose.pose.orientation.x = 0;
  odom_msg_->pose.pose.orientation.y = 0;
  odom_msg_->pose.pose.orientation.z = 0;
  odom_msg_->pose.pose.orientation.w = 1;

  // Initialize the transform message for the TF
  odom_transform_->header.stamp = odom_msg_->header.stamp;
  odom_transform_->transform.translation.x = 0;
  odom_transform_->transform.translation.y = 0;
  odom_transform_->transform.translation.z = 0;
  odom_transform_->transform.rotation.x = 0;
  odom_transform_->transform.rotation.y = 0;
  odom_transform_->transform.rotation.z = 0;
  odom_transform_->transform.rotation.w = 0;

  // Initialize odom publisher
  odom_pub_ = node_ptr->create_publisher<nav_msgs::msg::Odometry>("odom", 2);

  // Initialize TF publisher
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 100;
  tf_pub_ = node_ptr->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(custom_qos_profile), custom_qos_profile));
}

Odom::~Odom()
{
  //The weak pointer of the node_ is expired by now.
  std::cout << "ODOM SHUTDOWN" << std::endl;
}

void Odom::setOdomFrameName(const std::string& frame_name)
{
  odom_msg_->header.frame_id = frame_name;
  odom_transform_->header.frame_id = frame_name;
}

void Odom::setRobotFrameName(const std::string& frame_name)
{
  odom_msg_->child_frame_id = frame_name;
  odom_transform_->child_frame_id = frame_name;
}

void Odom::update(float pos_x, float pos_y, float ori_z, float lin_vel_x, float lin_vel_y, float ang_vel_z)
{
  auto node_ptr = node_.lock();
  if(!node_ptr)
  {
    return;
  }

  tf2::Quaternion quat;
  quat.setEuler(0, 0, ori_z);
  auto quat_msg = tf2::toMsg(quat);

  odom_transform_->header.stamp = node_ptr->now();
  odom_transform_->transform.translation.x = pos_x;
  odom_transform_->transform.translation.y = pos_y;
  odom_transform_->transform.translation.z = 0.0;
  odom_transform_->transform.rotation = quat_msg;

  odom_msg_->header.stamp = node_ptr->now();
  odom_msg_->pose.pose.position.x = pos_x;
  odom_msg_->pose.pose.position.y = pos_y;
  odom_msg_->pose.pose.position.z = 0.0;
  odom_msg_->pose.pose.orientation = quat_msg;
  odom_msg_->twist.twist.linear.x = lin_vel_x;
  odom_msg_->twist.twist.linear.y = lin_vel_y;
  odom_msg_->twist.twist.angular.z = ang_vel_z;
}

void Odom::publish()
{
  if (odom_pub_)
  {
    (odom_pub_->publish)(*odom_msg_);
  }

 if (tf_pub_)
  {
    tf2_msgs::msg::TFMessage message;
    message.transforms.push_back(*odom_transform_);
    tf_pub_->publish(message);
  }
}

} // namespace robotont
