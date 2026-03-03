#ifndef RFM21_DRIVER__INTERFACES_HPP_
#define RFM21_DRIVER__INTERFACES_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <PCANBasic.h> 

class CanDriverInterface {
public:
    using Ptr = std::shared_ptr<CanDriverInterface>;
    virtual ~CanDriverInterface() = default;
    virtual bool initialize() = 0;
    virtual TPCANStatus read_message(TPCANMsg& msg) = 0;
    // 명령 전송을 위한 인터페이스 추가
    virtual TPCANStatus write_message(TPCANMsg& msg) = 0; 
};

class DeviceInterface {
public:
    using Ptr = std::shared_ptr<DeviceInterface>;
    virtual ~DeviceInterface() = default;
    virtual void setup(rclcpp::Node& node) = 0;
    virtual void process_can_message(const TPCANMsg& msg) = 0;
    virtual void publish_data(rclcpp::Node& node) = 0;
};

#endif