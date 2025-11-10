#ifndef GLS100_DRIVER__GLS100_DEVICE_HPP_
#define GLS100_DRIVER__GLS100_DEVICE_HPP_

#include "interfaces.hpp"

class GLS100Device : public DeviceInterface
{
public:
    GLS100Device(rclcpp::Logger logger);
    ~GLS100Device() override = default;

    void setup(rclcpp::Node& node) override;
    void process_can_message(const TPCANMsg& msg) override;
    void publish_data(rclcpp::Node& node) override;

private:
    // void publishPose();
    // void publishVelocity();
    // void publishStatus();
    void publishPose(rclcpp::Node& node);
    void publishVelocity(rclcpp::Node& node);
    void publishStatus(rclcpp::Node& node); 
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr status_pub_;
    
    int node_id_ = 0;
    std::string frame_id_ = "";
    rclcpp::Logger logger_;

    double x_position_ = 0.0;
    double y_position_ = 0.0;
    double z_position_ = 0.0;
    double heading_ = 0.0;
    double x_velocity_ = 0.0;
    double y_velocity_ = 0.0;
    uint16_t status_ = 0;
    
    bool position_updated_ = false;
    bool angle_updated_ = false;
    bool velocity_updated_ = false;
};

#endif 