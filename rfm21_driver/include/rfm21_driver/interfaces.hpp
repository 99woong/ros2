#ifndef RFM21_DRIVER__INTERFACES_HPP_
#define RFM21_DRIVER__INTERFACES_HPP_

#include <rclcpp/rclcpp.hpp>
#include <PCANBasic.h>

class CanDriverInterface
{
public:
    using Ptr = std::shared_ptr<CanDriverInterface>;

    virtual ~CanDriverInterface() = default;
    virtual bool initialize() = 0;
    virtual TPCANStatus read_message(TPCANMsg& msg) = 0;
    virtual TPCANStatus write_message(const TPCANMsg& msg) = 0;
};

class DeviceInterface
{
public:
    using Ptr = std::shared_ptr<DeviceInterface>;

    virtual ~DeviceInterface() = default;
    virtual void setup(rclcpp::Node& node) = 0;
    virtual void process_can_message(const TPCANMsg& msg) = 0;
    virtual void publish_data(rclcpp::Node& node) = 0;
    virtual void send_startup(rclcpp::Node& node) = 0;
};

#endif  // RFM21_DRIVER__INTERFACES_HPP_
