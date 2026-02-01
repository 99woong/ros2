#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

class SlamTfApplyNode : public rclcpp::Node
{
public:
    SlamTfApplyNode() : Node("slam_tf_apply")
    {
        yaml_path_ = declare_parameter("extrinsic_yaml", "extrinsic.yaml");

        loadYaml();

        sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vslam/pose", 10,
            std::bind(&SlamTfApplyNode::callback, this, std::placeholders::_1));

        pub_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/vslam/pose_aligned", 10);
        pub_path_ = create_publisher<nav_msgs::msg::Path>(
            "/vslam/path_aligned", 10);

        path_.header.frame_id = "map";
    }

private:
    void loadYaml()
    {
        YAML::Node n = YAML::LoadFile(yaml_path_);
        tx_ = n["extrinsic_x"].as<double>();
        ty_ = n["extrinsic_y"].as<double>();
        yaw_ = n["extrinsic_yaw"].as<double>();

        R_ << cos(yaw_), -sin(yaw_),
              sin(yaw_),  cos(yaw_);
    }

    void callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        Eigen::Vector2d p(msg->pose.position.x, msg->pose.position.y);
        Eigen::Vector2d pa = R_ * p + Eigen::Vector2d(tx_, ty_);

        auto out = *msg;
        out.pose.position.x = pa.x();
        out.pose.position.y = pa.y();
        pub_pose_->publish(out);

        path_.poses.push_back(out);
        path_.header.stamp = now();
        pub_path_->publish(path_);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;

    nav_msgs::msg::Path path_;
    std::string yaml_path_;

    double tx_, ty_, yaw_;
    Eigen::Matrix2d R_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SlamTfApplyNode>());
    rclcpp::shutdown();
    return 0;
}
