#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <deque>
#include <fstream>
#include <thread>
#include <atomic>
#include <termios.h>
#include <unistd.h>

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

    keyboard_thread_ =
      std::thread(&SlamTfObserveNode::keyboardLoop, this);

    RCLCPP_INFO(get_logger(),
      "slam_tf_observe started");
    RCLCPP_INFO(get_logger(),
      "Move & rotate robot 충분히 수행 후");
    RCLCPP_INFO(get_logger(),
      "키보드에서 's' 를 누르면 extrinsic 계산");
  }

  ~SlamTfObserveNode()
  {
    running_ = false;
    if (keyboard_thread_.joinable())
      keyboard_thread_.join();
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

    sick_buf_.push_back(poseToVec(*sick));
    seven_buf_.push_back(poseToVec(*seven));

    if (sick_buf_.size() > max_buf_) {
      sick_buf_.pop_front();
      seven_buf_.pop_front();
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "observing... samples=%zu rot=%.1f deg",
      sick_buf_.size(),
      accumulatedYaw(seven_buf_) * 180.0 / M_PI);
  }

  /* ---------------- Keyboard Thread ---------------- */

  void keyboardLoop()
  {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    while (running_) {
      char c = getchar();
      if (c == 's') {
        RCLCPP_INFO(get_logger(), "'s' pressed → finish observation");
        finalizeObservation();
        break;
      }
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  }

  /* ---------------- Finalize ---------------- */

  void finalizeObservation()
  {
    if (estimated_) return;

    if (sick_buf_.size() < min_samples_) {
      RCLCPP_WARN(get_logger(),
        "Not enough samples (%zu < %d)",
        sick_buf_.size(), min_samples_);
      return;
    }

    if (accumulatedYaw(seven_buf_) < min_rotation_rad_) {
      RCLCPP_WARN(get_logger(),
        "Not enough rotation");
      return;
    }

    estimateExtrinsic();
    writeYaml();
    estimated_ = true;

    RCLCPP_INFO(get_logger(),
      "Extrinsic observation finished");
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

  const size_t max_buf_{2000};

  int min_samples_;
  double min_rotation_rad_;
  std::string output_yaml_;

  std::thread keyboard_thread_;
  std::atomic<bool> running_{true};

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
