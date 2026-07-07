#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <string>

namespace rtk_lio
{

class HeadingToImuNode : public rclcpp::Node
{
public:
  HeadingToImuNode();

private:
  void headingCallback(
    const geometry_msgs::msg::QuaternionStamped::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;

  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  double yaw_cov_;
  double rp_cov_;
  double yaw_offset_;
};

}  // namespace rtk_lio