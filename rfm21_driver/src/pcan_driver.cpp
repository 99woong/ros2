#include "rfm21_driver/pcan_driver.hpp"

PCANDriver::PCANDriver(const std::string& device_name, int baudrate, rclcpp::Logger logger)
: can_handle_(0), device_name_(device_name), baudrate_(baudrate), logger_(logger)
{
}

PCANDriver::~PCANDriver()
{
    if (can_handle_ != 0) {
        CAN_Uninitialize(can_handle_);
        RCLCPP_INFO(logger_, "PCAN uninitialised.");
    }
}

// ─── private helpers ─────────────────────────────────────────────────────────

TPCANBaudrate PCANDriver::convert_baudrate(int baudrate)
{
    switch (baudrate) {
        case 125000:  return PCAN_BAUD_125K;
        case 250000:  return PCAN_BAUD_250K;
        case 500000:  return PCAN_BAUD_500K;
        case 1000000: return PCAN_BAUD_1M;
        default:
            RCLCPP_WARN(logger_,
                "Unsupported baudrate %d. Supported: 125000, 250000, 500000, 1000000.", baudrate);
            return PCAN_BAUD_5K;   // sentinel – caller checks for this
    }
}

TPCANHandle PCANDriver::convert_device_name(const std::string& device_name)
{
    if (device_name == "PCAN_USBBUS1")  return PCAN_USBBUS1;
    if (device_name == "PCAN_USBBUS2")  return PCAN_USBBUS2;
    if (device_name == "PCAN_USBBUS3")  return PCAN_USBBUS3;
    if (device_name == "PCAN_USBBUS4")  return PCAN_USBBUS4;
    if (device_name == "PCAN_USBBUS5")  return PCAN_USBBUS5;
    if (device_name == "PCAN_USBBUS6")  return PCAN_USBBUS6;
    if (device_name == "PCAN_USBBUS7")  return PCAN_USBBUS7;
    if (device_name == "PCAN_USBBUS8")  return PCAN_USBBUS8;
    RCLCPP_ERROR(logger_, "Unknown CAN device: %s", device_name.c_str());
    return 0;
}

// ─── public interface ────────────────────────────────────────────────────────

bool PCANDriver::initialize()
{
    can_handle_ = convert_device_name(device_name_);
    if (can_handle_ == 0) {
        return false;
    }

    TPCANBaudrate pcan_baudrate = convert_baudrate(baudrate_);
    if (pcan_baudrate == PCAN_BAUD_5K) {
        return false;
    }

    TPCANStatus status = CAN_Initialize(can_handle_, pcan_baudrate, 0, 0, 0);
    if (status != PCAN_ERROR_OK) {
        RCLCPP_ERROR(logger_, "CAN_Initialize failed: 0x%x", status);
        return false;
    }

    RCLCPP_INFO(logger_,
        "PCAN initialised on %s at %d bps.", device_name_.c_str(), baudrate_);
    return true;
}

TPCANStatus PCANDriver::read_message(TPCANMsg& msg)
{
    return CAN_Read(can_handle_, &msg, nullptr);
}

TPCANStatus PCANDriver::write_message(const TPCANMsg& msg)
{
    return CAN_Write(can_handle_, const_cast<TPCANMsg*>(&msg));
}
