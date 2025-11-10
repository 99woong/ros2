#include "gls100_driver/gls100_device.hpp"
#include <cmath> // M_PI for heading calculation

GLS100Device::GLS100Device(rclcpp::Logger logger)
: logger_(logger)
{

}

void GLS100Device::setup(rclcpp::Node& node)
{
    node.declare_parameter("node_id", 10);
    node.declare_parameter("frame_id", "gls100");

    node_id_ = node.get_parameter("node_id").as_int();
    frame_id_ = node.get_parameter("frame_id").as_string();

    pose_pub_ = node.create_publisher<geometry_msgs::msg::PoseStamped>("gls100/pose", 10);
    velocity_pub_ = node.create_publisher<geometry_msgs::msg::TwistStamped>("gls100/velocity", 10);
    status_pub_ = node.create_publisher<std_msgs::msg::UInt16>("gls100/status", 10);
        
    RCLCPP_INFO(logger_, "GLS100 Device data processor setup. Node ID: %d, Frame ID: %s", node_id_, frame_id_.c_str());
}

void GLS100Device::process_can_message(const TPCANMsg& msg)
{
    uint32_t pdo1_id = 0x180 + node_id_;  // TPDO1: x, y position
    uint32_t pdo2_id = 0x280 + node_id_;  // TPDO2: status, z, angle, time offset
    uint32_t pdo3_id = 0x380 + node_id_;  // TPDO3: tag_id, x_vel, y_vel
    
    if (msg.ID == pdo1_id && msg.LEN == 8) 
    {
        int32_t x_raw = *reinterpret_cast<const int32_t*>(&msg.DATA[0]);
        int32_t y_raw = *reinterpret_cast<const int32_t*>(&msg.DATA[4]);
        
        x_position_ = x_raw * 0.0001;  // 0.1mm to meters
        y_position_ = y_raw * 0.0001;
        
        position_updated_ = true;
    }
    else if (msg.ID == pdo2_id && msg.LEN == 8)
    {
        // TPDO2: Status (bytes 0-1), Z-Position (bytes 2-3), 
        //        Angle (bytes 4-5), Time Offset (bytes 6-7)
        uint16_t status = *reinterpret_cast<const uint16_t*>(&msg.DATA[0]);
        int16_t z_raw = *reinterpret_cast<const int16_t*>(&msg.DATA[2]);
        int16_t angle_raw = *reinterpret_cast<const int16_t*>(&msg.DATA[4]);
        
        status_ = status;

        z_position_ = z_raw * 0.0001;  // 0.1mm to meters
        
        // Convert angle (default scaling: pi/32767 rad per increment)
        heading_ = angle_raw * M_PI / 32767.0;
        
        angle_updated_ = true;
    }
    else if (msg.ID == pdo3_id && msg.LEN == 8) 
    {
        // uint32_t tag_id = *reinterpret_cast<const uint32_t*>(&msg.DATA[0]);
        int16_t x_vel_raw = *reinterpret_cast<const int16_t*>(&msg.DATA[4]);
        int16_t y_vel_raw = *reinterpret_cast<const int16_t*>(&msg.DATA[6]);
        
        // Convert to m/s (default scaling: 0.1mm/s per increment)
        x_velocity_ = x_vel_raw * 0.0001;
        y_velocity_ = y_vel_raw * 0.0001;
        
        velocity_updated_ = true;
    }
}

void GLS100Device::publish_data(rclcpp::Node& node)
{\
    if (position_updated_ && angle_updated_) 
    {
        publishPose(node); // 노드 참조 전달
        publishStatus(node); // 노드 참조 전달
        position_updated_ = false;
        angle_updated_ = false;
    }
    
    if (velocity_updated_) 
    {
        publishVelocity(node); // 노드 참조 전달
        velocity_updated_ = false;
    }
}

void GLS100Device::publishPose(rclcpp::Node& node) // node 인자 추가
{
    auto msg = geometry_msgs::msg::PoseStamped();

    msg.header.stamp = node.now(); 
    msg.header.frame_id = frame_id_;
    
    msg.pose.position.x = x_position_;
    msg.pose.position.y = y_position_;
    msg.pose.position.z = z_position_;
    
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = sin(heading_ / 2.0);
    msg.pose.orientation.w = cos(heading_ / 2.0);
    
    pose_pub_->publish(msg);
}

void GLS100Device::publishVelocity(rclcpp::Node& node) // node 인자 추가
{
    auto msg = geometry_msgs::msg::TwistStamped();

    msg.header.stamp = node.now();
    msg.header.frame_id = frame_id_;
    
    msg.twist.linear.x = x_velocity_;
    msg.twist.linear.y = y_velocity_;
    msg.twist.linear.z = 0.0;
    
    msg.twist.angular.x = 0.0;
    msg.twist.angular.y = 0.0;
    msg.twist.angular.z = 0.0;
    
    velocity_pub_->publish(msg);
}

void GLS100Device::publishStatus(rclcpp::Node& node) // node 인자 추가
{
    (void)node; // 현재 상태 메시지 발행에는 시간이 필요 없으므로 사용하지 않음
    auto msg = std_msgs::msg::UInt16();
    msg.data = status_;
    status_pub_->publish(msg);
}
