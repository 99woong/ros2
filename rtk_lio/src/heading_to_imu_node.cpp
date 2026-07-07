#include "rtk_lio/heading_to_imu_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace rtk_lio
{

HeadingToImuNode::HeadingToImuNode()
: Node("heading_to_imu_node")
{
  // 파라미터 선언
  this->declare_parameter<std::string>("input_topic",  "/heading");
  this->declare_parameter<std::string>("output_topic", "/gps/imu");
  this->declare_parameter<std::string>("frame_id",     "body");
  this->declare_parameter<double>("yaw_covariance",    0.01);
  this->declare_parameter<double>("roll_pitch_cov",    9999.0);
  this->declare_parameter<double>("yaw_offset",        0.0);

  input_topic_   = this->get_parameter("input_topic").as_string();
  output_topic_  = this->get_parameter("output_topic").as_string();
  frame_id_      = this->get_parameter("frame_id").as_string();
  yaw_cov_       = this->get_parameter("yaw_covariance").as_double();
  rp_cov_        = this->get_parameter("roll_pitch_cov").as_double();
  yaw_offset_    = this->get_parameter("yaw_offset").as_double();

  // Subscriber / Publisher
  sub_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
    input_topic_, rclcpp::SensorDataQoS(),
    std::bind(&HeadingToImuNode::headingCallback, this, std::placeholders::_1));

  pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
    output_topic_, rclcpp::SensorDataQoS());

  RCLCPP_INFO(this->get_logger(),
    "HeadingToImuNode started. [%s] -> [%s]",
    input_topic_.c_str(), output_topic_.c_str());
}

void HeadingToImuNode::headingCallback(
  const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
{
  sensor_msgs::msg::Imu imu_msg;

  // 헤더 복사
  imu_msg.header          = msg->header;
  imu_msg.header.frame_id = frame_id_;

  // navsat_transform의 yaw_offset과 동일한 회전 적용 → EKF imu0 헤딩 기준 통일
  tf2::Quaternion q_raw;
  tf2::fromMsg(msg->quaternion, q_raw);
  tf2::Quaternion q_offset;
  q_offset.setRPY(0.0, 0.0, yaw_offset_);
  tf2::Quaternion q_corrected = (q_offset * q_raw).normalized();
  imu_msg.orientation = tf2::toMsg(q_corrected);

  // Orientation 공분산
  // [0]=roll, [4]=pitch, [8]=yaw (row-major 3x3)
  imu_msg.orientation_covariance[0] = rp_cov_;  // roll  무시
  imu_msg.orientation_covariance[4] = rp_cov_;  // pitch 무시
  imu_msg.orientation_covariance[8] = yaw_cov_; // yaw   신뢰

  // angular_velocity / linear_acceleration 미사용 → -1로 표시
  imu_msg.angular_velocity_covariance[0]    = -1.0;
  imu_msg.linear_acceleration_covariance[0] = -1.0;

  pub_->publish(imu_msg);
}

}  // namespace rtk_lio

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rtk_lio::HeadingToImuNode>());
  rclcpp::shutdown();
  return 0;
}