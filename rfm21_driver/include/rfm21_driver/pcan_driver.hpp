#ifndef RFM21_DRIVER__PCAN_DRIVER_HPP_
#define RFM21_DRIVER__PCAN_DRIVER_HPP_

#include "rfm21_driver/interfaces.hpp"

class PCANDriver : public CanDriverInterface
{
public:
    PCANDriver(const std::string& device_name, int baudrate, rclcpp::Logger logger);
    ~PCANDriver() override;

    bool initialize() override;
    TPCANStatus read_message(TPCANMsg& msg) override;
    TPCANStatus write_message(const TPCANMsg& msg) override;

private:
    TPCANHandle   can_handle_;
    std::string   device_name_;
    int           baudrate_;
    rclcpp::Logger logger_;

    TPCANBaudrate convert_baudrate(int baudrate);
    TPCANHandle   convert_device_name(const std::string& device_name);
};

#endif  // RFM21_DRIVER__PCAN_DRIVER_HPP_
