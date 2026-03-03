#include <rclcpp/rclcpp.hpp>
#include <PCANBasic.h> 
#include "rfm21_driver/interfaces.hpp"
#include "rfm21_driver/pcan_driver.hpp"
#include "rfm21_driver/rfm21_device.hpp"

class GenericCanNode : public rclcpp::Node {
public:
    GenericCanNode(CanDriverInterface::Ptr can_driver, DeviceInterface::Ptr device_processor)
    : Node("rfm21_driver_node"), can_driver_(can_driver), device_processor_(device_processor) {
        device_processor_->setup(*this);

        if (!can_driver_->initialize()) {
            RCLCPP_ERROR(this->get_logger(), "CAN Driver Init Failed");
            return;
        }

        // 1. RFM-2.1 활성화를 위한 Start Node 메시지 전송 
        TPCANMsg start_msg;
        start_msg.ID = 0x000;
        start_msg.MSGTYPE = PCAN_MESSAGE_STANDARD;
        start_msg.LEN = 2;
        start_msg.DATA[0] = 0x01; // Start Command
        start_msg.DATA[1] = 0x0F; // Node ID (필요시 파라미터로 변경)
        can_driver_->write_message(start_msg);
        RCLCPP_INFO(this->get_logger(), "Sent Start Node (0x01) to RFM-2.1");

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&GenericCanNode::readCANMessages, this));
    }

private:
    void readCANMessages() {
        TPCANMsg msg;
        while (can_driver_->read_message(msg) == PCAN_ERROR_OK) {
            device_processor_->process_can_message(msg);
        }
        device_processor_->publish_data(*this);
    }
    CanDriverInterface::Ptr can_driver_;
    DeviceInterface::Ptr device_processor_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // RFM-2.1 기본 설정: 250k Baudrate, PCAN_USBBUS1 
    auto pcan_driver = std::make_shared<PCANDriver>("PCAN_USBBUS1", 250000, rclcpp::get_logger("pcan"));
    auto rfm_device = std::make_shared<RFM21Device>(rclcpp::get_logger("rfm21"));

    rclcpp::spin(std::make_shared<GenericCanNode>(pcan_driver, rfm_device));
    rclcpp::shutdown();
    return 0;
}