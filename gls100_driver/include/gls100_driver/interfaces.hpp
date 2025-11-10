#ifndef GLS100_DRIVER__INTERFACES_HPP_
#define GLS100_DRIVER__INTERFACES_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <PCANBasic.h> 

class CanDriverInterface
{
public:
    using Ptr = std::shared_ptr<CanDriverInterface>;
    
    virtual ~CanDriverInterface() = default;
    virtual bool initialize() = 0;
    virtual TPCANStatus read_message(TPCANMsg& msg) = 0;
};

class DeviceInterface
{
public:
    using Ptr = std::shared_ptr<DeviceInterface>;
    
    virtual ~DeviceInterface() = default;
    virtual void setup(rclcpp::Node& node) = 0;
    virtual void process_can_message(const TPCANMsg& msg) = 0;
    virtual void publish_data(rclcpp::Node& node) = 0;
};

#endif