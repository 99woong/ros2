#ifndef RFM21_DRIVER__RFM21_DEVICE_HPP_
#define RFM21_DRIVER__RFM21_DEVICE_HPP_

#include "interfaces.hpp"

class RFM21Device : public DeviceInterface {
public:
    RFM21Device(rclcpp::Logger logger);
    void setup(rclcpp::Node& node) override;
    void process_can_message(const TPCANMsg& msg) override;
    void publish_data(rclcpp::Node& node) override;

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr abs_x_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr abs_y_pub_;

    int node_id_ = 15;
    std::string frame_id_ = "rfm_link";
    rclcpp::Logger logger_;

    double rel_x_ = 0.0, rel_y_ = 0.0;
    int32_t abs_x_ = 0, abs_y_ = 0;
    uint16_t status_ = 0;
    bool rel_updated_ = false, abs_updated_ = false;
};

#endif