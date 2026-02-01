#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <Eigen/Dense>
#include <deque>

using geometry_msgs::msg::PoseStamped;
using SyncPolicy =
  message_filters::sync_policies::ApproximateTime<PoseStamped, PoseStamped>;

class SlamTfAlignNode : public rclcpp::Node
{
public:
  SlamTfAlignNode() : Node("slam_tf_align_node")
  {
    sick_sub_.subscribe(this, "/localization_pose");
    seven_sub_.subscribe(this, "/vslam/pose");

    sync_ = std::make_shared<
      message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(50),
                                                  sick_sub_, seven_sub_);
    sync_->registerCallback(
      std::bind(&SlamTfAlignNode::syncCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    aligned_pub_ =
      create_publisher<PoseStamped>("/vslam/pose_aligned", 10);

    RCLCPP_INFO(get_logger(), "slam_tf_align_node started");
  }

private:
  /* ----------------------- Utils ----------------------- */

  static Eigen::Vector3d poseToVec(const PoseStamped &msg)
  {
    double x = msg.pose.position.x;
    double y = msg.pose.position.y;

    double qw = msg.pose.orientation.w;
    double qz = msg.pose.orientation.z;
    double yaw = std::atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz);

    return Eigen::Vector3d(x, y, yaw);
  }

  static Eigen::Vector3d transform(
    const Eigen::Vector3d &p,
    const Eigen::Vector3d &t)
  {
    double c = std::cos(t.z());
    double s = std::sin(t.z());

    Eigen::Vector3d out;
    out.x() = c * p.x() - s * p.y() + t.x();
    out.y() = s * p.x() + c * p.y() + t.y();
    out.z() = p.z() + t.z();
    return out;
  }

  /* ----------------------- Sync CB ----------------------- */

  void syncCallback(
    const PoseStamped::ConstSharedPtr sick,
    const PoseStamped::ConstSharedPtr seven)
  {
    sick_buf_.push_back(poseToVec(*sick));
    seven_buf_.push_back(poseToVec(*seven));

    if (sick_buf_.size() > max_buf_) {
      sick_buf_.pop_front();
      seven_buf_.pop_front();
    }

    if (!offset_ready_ && sick_buf_.size() >= min_samples_) {
      estimateExtrinsic();
    }

    if (offset_ready_) {
      auto aligned = *seven;
      Eigen::Vector3d p = poseToVec(*seven);
      Eigen::Vector3d pa = transform(p, offset_);

      aligned.pose.position.x = pa.x();
      aligned.pose.position.y = pa.y();
      aligned.pose.orientation.z = std::sin(pa.z() * 0.5);
      aligned.pose.orientation.w = std::cos(pa.z() * 0.5);

      aligned_pub_->publish(aligned);
    }
  }

  /* ----------------------- Core ----------------------- */

  void estimateExtrinsic()
  {
    const int N = sick_buf_.size();

    Eigen::Vector2d cs(0, 0), cl(0, 0);
    for (int i = 0; i < N; i++) {
      cs += seven_buf_[i].head<2>();
      cl += sick_buf_[i].head<2>();
    }
    cs /= N;
    cl /= N;

    Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
    for (int i = 0; i < N; i++) {
      H += (seven_buf_[i].head<2>() - cs) *
           (sick_buf_[i].head<2>() - cl).transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix2d> svd(
      H, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix2d R = svd.matrixV() * svd.matrixU().transpose();

    double yaw = std::atan2(R(1, 0), R(0, 0));
    Eigen::Vector2d t = cl - R * cs;

    offset_ << t.x(), t.y(), yaw;
    offset_ready_ = true;

    RCLCPP_INFO(get_logger(),
      "Estimated TF offset: dx=%.3f dy=%.3f dyaw=%.3f deg",
      offset_.x(), offset_.y(), yaw * 180.0 / M_PI);
  }

  /* ----------------------- Members ----------------------- */

  message_filters::Subscriber<PoseStamped> sick_sub_;
  message_filters::Subscriber<PoseStamped> seven_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Publisher<PoseStamped>::SharedPtr aligned_pub_;

  std::deque<Eigen::Vector3d> sick_buf_;
  std::deque<Eigen::Vector3d> seven_buf_;

  Eigen::Vector3d offset_{0, 0, 0};
  bool offset_ready_{false};

  const size_t max_buf_{500};
  const size_t min_samples_{100};
};

/* ----------------------- main ----------------------- */

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlamTfAlignNode>());
  rclcpp::shutdown();
  return 0;
}
