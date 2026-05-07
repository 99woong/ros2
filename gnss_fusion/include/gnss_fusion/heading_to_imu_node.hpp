#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <string>

namespace gnss_fusion
{

/**
 * @brief /heading (QuaternionStamped) → /gps/imu (Imu) 변환 노드
 *
 * Dual RTK GNSS의 /heading 토픽을 robot_localization EKF가
 * 인식할 수 있는 sensor_msgs/Imu 형식으로 변환한다.
 * yaw만 신뢰하고 roll/pitch는 무시(큰 공분산)로 설정한다.
 */
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

}  // namespace gnss_fusion