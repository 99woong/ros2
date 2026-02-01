#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <deque>
#include <fstream>

using geometry_msgs::msg::PoseStamped;
using SyncPolicy =
  message_filters::sync_policies::ApproximateTime<PoseStamped, PoseStamped>;

class SlamTfObserveNode : public rclcpp::Node
{
public:
  SlamTfObserveNode() : Node("slam_tf_observe")
  {
    min_samples_ =
      declare_parameter<int>("min_samples", 300);
    min_rotation_rad_ =
      declare_parameter<double>("min_rotation_deg", 60.0) * M_PI / 180.0;

    stationary_required_sec_ =
      declare_parameter<double>("stationary_sec", 5.0);

    lin_vel_thresh_ =
      declare_parameter<double>("linear_vel_thresh", 0.02);   // m/s
    ang_vel_thresh_ =
      declare_parameter<double>("angular_vel_thresh", 0.01);  // rad/s

    output_yaml_ =
      declare_parameter<std::string>("output_yaml", "extrinsic.yaml");

    sick_sub_.subscribe(this, "/localization_pose");
    seven_sub_.subscribe(this, "/vslam/pose");

    sync_ = std::make_shared<
      message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(50), sick_sub_, seven_sub_);

    sync_->registerCallback(
      std::bind(&SlamTfObserveNode::syncCallback, this,
                std::placeholders::_1,
                std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "slam_tf_observe node started");
  }

private:
  /* ---------------- Utils ---------------- */

  static Eigen::Vector3d poseToVec(const PoseStamped &msg)
  {
    double x = msg.pose.position.x;
    double y = msg.pose.position.y;

    double qw = msg.pose.orientation.w;
    double qz = msg.pose.orientation.z;
    double yaw =
      std::atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz);

    return {x, y, yaw};
  }

  static double accumulatedYaw(
    const std::deque<Eigen::Vector3d> &buf)
  {
    double sum = 0.0;
    for (size_t i = 1; i < buf.size(); ++i)
      sum += std::fabs(buf[i].z() - buf[i - 1].z());
    return sum;
  }

  /* ---------------- Sync Callback ---------------- */

  void syncCallback(
    const PoseStamped::ConstSharedPtr sick,
    const PoseStamped::ConstSharedPtr seven)
  {
    if (estimated_) return;

    const rclcpp::Time now = sick->header.stamp;

    Eigen::Vector3d sick_p = poseToVec(*sick);
    Eigen::Vector3d seven_p = poseToVec(*seven);

    sick_buf_.push_back(sick_p);
    seven_buf_.push_back(seven_p);

    if (sick_buf_.size() > max_buf_) {
      sick_buf_.pop_front();
      seven_buf_.pop_front();
    }

    /* --------- motion check --------- */

    if (!last_time_.nanoseconds()) {
      last_time_ = now;
      last_pose_ = sick_p;
      return;
    }

    double dt = (now - last_time_).seconds();
    if (dt <= 0.0) return;

    double lin_vel =
      (sick_p.head<2>() - last_pose_.head<2>()).norm() / dt;
    double ang_vel =
      std::fabs(sick_p.z() - last_pose_.z()) / dt;

    bool stationary =
      lin_vel < lin_vel_thresh_ &&
      ang_vel < ang_vel_thresh_;

    if (stationary) {
      stationary_time_ += dt;
    } else {
      stationary_time_ = 0.0;
    }

    last_time_ = now;
    last_pose_ = sick_p;

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "samples=%zu rot=%.1fdeg stationary=%.1fs",
      sick_buf_.size(),
      accumulatedYaw(seven_buf_) * 180.0 / M_PI,
      stationary_time_);

    /* --------- completion condition --------- */

    if (sick_buf_.size() < min_samples_) return;
    if (accumulatedYaw(seven_buf_) < min_rotation_rad_) return;
    if (stationary_time_ < stationary_required_sec_) return;

    estimateExtrinsic();
    writeYaml();
    estimated_ = true;

    RCLCPP_INFO(get_logger(),
      "Extrinsic observation finished (stationary %.1fs)",
      stationary_time_);
  }

  /* ---------------- Core Estimation ---------------- */

  void estimateExtrinsic()
  {
    const int N = sick_buf_.size();

    Eigen::Vector2d cs(0, 0), cl(0, 0);
    for (int i = 0; i < N; ++i) {
      cs += seven_buf_[i].head<2>();
      cl += sick_buf_[i].head<2>();
    }
    cs /= N;
    cl /= N;

    Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
    for (int i = 0; i < N; ++i) {
      H += (seven_buf_[i].head<2>() - cs) *
           (sick_buf_[i].head<2>() - cl).transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix2d> svd(
      H, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix2d R =
      svd.matrixV() * svd.matrixU().transpose();

    if (R.determinant() < 0) {
      Eigen::Matrix2d V = svd.matrixV();
      V.col(1) *= -1;
      R = V * svd.matrixU().transpose();
    }

    double yaw = std::atan2(R(1, 0), R(0, 0));
    Eigen::Vector2d t = cl - R * cs;

    offset_ << t.x(), t.y(), yaw;

    RCLCPP_INFO(get_logger(),
      "Estimated extrinsic: dx=%.3f dy=%.3f dyaw=%.2f deg",
      offset_.x(), offset_.y(),
      yaw * 180.0 / M_PI);
  }

  /* ---------------- YAML ---------------- */

  void writeYaml()
  {
    YAML::Node n;
    n["extrinsic_x"] = offset_.x();
    n["extrinsic_y"] = offset_.y();
    n["extrinsic_yaw"] = offset_.z();

    std::ofstream fout(output_yaml_);
    fout << n;
    fout.close();

    RCLCPP_INFO(get_logger(),
      "Saved extrinsic yaml: %s",
      output_yaml_.c_str());
  }

  /* ---------------- Members ---------------- */

  message_filters::Subscriber<PoseStamped> sick_sub_;
  message_filters::Subscriber<PoseStamped> seven_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  std::deque<Eigen::Vector3d> sick_buf_;
  std::deque<Eigen::Vector3d> seven_buf_;

  const size_t max_buf_{1500};

  int min_samples_;
  double min_rotation_rad_;
  double stationary_required_sec_;
  double lin_vel_thresh_;
  double ang_vel_thresh_;

  std::string output_yaml_;

  rclcpp::Time last_time_;
  Eigen::Vector3d last_pose_{0, 0, 0};
  double stationary_time_{0.0};

  bool estimated_{false};
  Eigen::Vector3d offset_{0, 0, 0};
};

/* ---------------- main ---------------- */

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlamTfObserveNode>());
  rclcpp::shutdown();
  return 0;
}
