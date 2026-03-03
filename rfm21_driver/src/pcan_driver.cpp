#include "rfm21_driver/pcan_driver.hpp"

PCANDriver::PCANDriver(const std::string& device_name, int baudrate, rclcpp::Logger logger)
: can_handle_(0), device_name_(device_name), baudrate_(baudrate), logger_(logger) {}

PCANDriver::~PCANDriver() {
    if (can_handle_ != 0) CAN_Uninitialize(can_handle_);
}

bool PCANDriver::initialize() {
    can_handle_ = convert_device_name(device_name_);
    if (can_handle_ == 0) return false;
    
    TPCANBaudrate pcan_baudrate = convert_baudrate(baudrate_);
    TPCANStatus status = CAN_Initialize(can_handle_, pcan_baudrate);
    return (status == PCAN_ERROR_OK);
}

TPCANStatus PCANDriver::read_message(TPCANMsg& msg) {
    return CAN_Read(can_handle_, &msg, NULL);
}

TPCANStatus PCANDriver::write_message(TPCANMsg& msg) {
    return CAN_Write(can_handle_, &msg);
}

TPCANBaudrate PCANDriver::convert_baudrate(int baudrate)
{
    switch (baudrate) 
    {
        case 125000: return PCAN_BAUD_125K;
        case 250000: return PCAN_BAUD_250K;
        case 500000: return PCAN_BAUD_500K;
        case 1000000: return PCAN_BAUD_1M;
        default: return PCAN_BAUD_5K; 
    }
}

TPCANHandle PCANDriver::convert_device_name(const std::string& device_name)
{
    if (device_name == "PCAN_USBBUS1") 
    {
        return PCAN_USBBUS1;
    } 
    else if (device_name == "PCAN_USBBUS2") 
    {
        return PCAN_USBBUS2;
    } 
    else 
    {
        return 0; 
    }
}
