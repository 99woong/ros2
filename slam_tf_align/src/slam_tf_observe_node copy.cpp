#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <fstream>

using geometry_msgs::msg::PoseStamped;
using SyncPolicy =
    message_filters::sync_policies::ApproximateTime<
        PoseStamped, PoseStamped>;

class SlamTfObserveNode : public rclcpp::Node
{
public:
    SlamTfObserveNode() : Node("slam_tf_observe")
    {
        min_samples_ = declare_parameter("min_samples", 300);
        min_rotation_rad_ =
            declare_parameter("min_rotation_deg", 45.0) * M_PI / 180.0;
        output_yaml_ = declare_parameter("output_yaml", "extrinsic.yaml");

        // sick_sub_.subscribe(this, "/localization_pose");
        // seven_sub_.subscribe(this, "/vslam/pose");
        sick_sub_.subscribe(this, "/localization_pose");
        seven_sub_.subscribe(this, "/vslam/pose");

        sync_ = std::make_shared<
            message_filters::Synchronizer<SyncPolicy>>(
                SyncPolicy(50), sick_sub_, seven_sub_);

        sync_->registerCallback(
            std::bind(&SlamTfObserveNode::callback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));

        // sync_->registerCallback(
        //     std::bind(&SlamTfObserveNode::callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(), "slam_tf_observe node started");
    }

private:
    void callback(const PoseStamped::ConstSharedPtr& sick,
                  const PoseStamped::ConstSharedPtr& seven)
    {
        RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "sync callback running, samples=%zu",
        seven_buf_.size());

        if (estimated_) return;

        Eigen::Vector3d p_s = poseToVec(*seven);
        Eigen::Vector3d p_l = poseToVec(*sick);

        seven_buf_.push_back(p_s);
        sick_buf_.push_back(p_l);

        if (seven_buf_.size() < min_samples_) return;

        if (accumulatedYaw(seven_buf_) < min_rotation_rad_) return;

        estimateExtrinsic();
        writeYaml();
        estimated_ = true;

        RCLCPP_INFO(get_logger(), "Extrinsic estimation finished");
    }

    Eigen::Vector3d poseToVec(const PoseStamped& p)
    {
        double yaw = std::atan2(
            2.0 * (p.pose.orientation.w * p.pose.orientation.z),
            1.0 - 2.0 * p.pose.orientation.z * p.pose.orientation.z);

        return {p.pose.position.x, p.pose.position.y, yaw};
    }

    double accumulatedYaw(const std::vector<Eigen::Vector3d>& buf)
    {
        double sum = 0.0;
        for (size_t i = 1; i < buf.size(); ++i)
            sum += std::abs(buf[i][2] - buf[i-1][2]);
        return sum;
    }

    void estimateExtrinsic()
    {
        // 평균 yaw 차이
        double yaw_sum = 0.0;
        for (size_t i = 0; i < seven_buf_.size(); ++i)
            yaw_sum += sick_buf_[i][2] - seven_buf_[i][2];
        yaw_ = yaw_sum / seven_buf_.size();

        Eigen::Matrix2d R;
        R << cos(yaw_), -sin(yaw_),
             sin(yaw_),  cos(yaw_);

        Eigen::Vector2d t_sum(0,0);
        for (size_t i = 0; i < seven_buf_.size(); ++i)
        {
            Eigen::Vector2d ps(seven_buf_[i][0], seven_buf_[i][1]);
            Eigen::Vector2d pl(sick_buf_[i][0], sick_buf_[i][1]);
            t_sum += pl - R * ps;
        }
        t_ = t_sum / seven_buf_.size();
    }

    void writeYaml()
    {
        YAML::Node n;
        n["extrinsic_x"] = t_.x();
        n["extrinsic_y"] = t_.y();
        n["extrinsic_yaw"] = yaw_;

        std::ofstream fout(output_yaml_);
        fout << n;
        fout.close();

        RCLCPP_INFO(get_logger(),
            "Saved extrinsic.yaml (x=%.3f y=%.3f yaw=%.2f deg)",
            t_.x(), t_.y(), yaw_ * 180.0 / M_PI);
    }

    message_filters::Subscriber<PoseStamped> sick_sub_, seven_sub_;
    // std::shared_ptr<message_filters::TimeSynchronizer<PoseStamped, PoseStamped>> sync_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    std::vector<Eigen::Vector3d> sick_buf_, seven_buf_;


    bool estimated_ = false;
    int min_samples_;
    double min_rotation_rad_;
    std::string output_yaml_;

    Eigen::Vector2d t_;
    double yaw_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlamTfObserveNode>());
    rclcpp::shutdown();
    return 0;
}
