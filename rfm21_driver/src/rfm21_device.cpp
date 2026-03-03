#include "rfm21_driver/rfm21_device.hpp"

RFM21Device::RFM21Device(rclcpp::Logger logger) : logger_(logger) {}

void RFM21Device::setup(rclcpp::Node& node) {
    node_id_ = node.declare_parameter("node_id", 15);
    frame_id_ = node.declare_parameter("frame_id", "rfm_link");

    pose_pub_ = node.create_publisher<geometry_msgs::msg::PoseStamped>("rfm21/relative_pose", 10);
    status_pub_ = node.create_publisher<std_msgs::msg::UInt16>("rfm21/status", 10);
    abs_x_pub_ = node.create_publisher<std_msgs::msg::Int32>("rfm21/abs_x", 10);
    abs_y_pub_ = node.create_publisher<std_msgs::msg::Int32>("rfm21/abs_y", 10);
}

void RFM21Device::process_can_message(const TPCANMsg& msg) {
    // Result 1: 0x180 (Trigger모드) 또는 0x280 (Sync모드) [cite: 62, 81]
    if (msg.ID == (uint32_t)(0x180 + node_id_) || msg.ID == (uint32_t)(0x280 + node_id_)) {
        if (msg.LEN < 6) return;
        status_ = (msg.DATA[1] << 8) | msg.DATA[0];
        // 2의 보수 mm 해석 [cite: 71, 84]
        rel_x_ = (double)((int16_t)((msg.DATA[3] << 8) | msg.DATA[2])) / 1000.0;
        rel_y_ = (double)((int16_t)((msg.DATA[5] << 8) | msg.DATA[4])) / 1000.0;
        rel_updated_ = true;
    }
    // Result 2: 0x280 (Trigger모드) 또는 0x380 (Sync모드) - 절대 좌표 [cite: 71, 85]
    else if (msg.ID == (uint32_t)(0x280 + node_id_) || msg.ID == (uint32_t)(0x380 + node_id_)) {
        if (msg.LEN < 8) return;
        // LSB First 32비트 해석 (8-8 Format 가정) [cite: 72, 86]
        abs_y_ = (msg.DATA[3] << 24) | (msg.DATA[2] << 16) | (msg.DATA[1] << 8) | msg.DATA[0];
        abs_x_ = (msg.DATA[7] << 24) | (msg.DATA[6] << 16) | (msg.DATA[5] << 8) | msg.DATA[4];
        abs_updated_ = true;
    }
}

void RFM21Device::publish_data(rclcpp::Node& node) {
    if (rel_updated_) {
        auto p_msg = geometry_msgs::msg::PoseStamped();
        p_msg.header.stamp = node.now();
        p_msg.header.frame_id = frame_id_;
        p_msg.pose.position.x = rel_x_;
        p_msg.pose.position.y = rel_y_;
        pose_pub_->publish(p_msg);
        
        auto s_msg = std_msgs::msg::UInt16();
        s_msg.data = status_;
        status_pub_->publish(s_msg);
        rel_updated_ = false;
    }
    if (abs_updated_) {
        auto x_msg = std_msgs::msg::Int32(); x_msg.data = abs_x_;
        auto y_msg = std_msgs::msg::Int32(); y_msg.data = abs_y_;
        abs_x_pub_->publish(x_msg); abs_y_pub_->publish(y_msg);
        abs_updated_ = false;
    }
}