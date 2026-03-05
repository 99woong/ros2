#include <rclcpp/rclcpp.hpp>
#include <PCANBasic.h>

#include "rfm21_driver/interfaces.hpp"
#include "rfm21_driver/pcan_driver.hpp"
#include "rfm21_driver/rfm21_device.hpp"

// ─────────────────────────────────────────────────────────────────────────────
//  GenericCanNode
//  Reads CAN messages at a configurable frequency and forwards them to the
//  device processor.  Designed to work with any DeviceInterface implementation.
// ─────────────────────────────────────────────────────────────────────────────

class GenericCanNode : public rclcpp::Node
{
public:
    GenericCanNode(CanDriverInterface::Ptr can_driver,
                   DeviceInterface::Ptr    device_processor)
    : Node("rfm21_can_node"),
      can_driver_(can_driver),
      device_processor_(device_processor)
    {
        this->declare_parameter("read_frequency_ms", 10);

        // Let the device set up its publishers and read its own parameters.
        device_processor_->setup(*this);

        // Initialise the CAN bus.
        if (!can_driver_->initialize()) {
            RCLCPP_ERROR(this->get_logger(),
                "Failed to initialise CAN driver. Shutting down.");
            rclcpp::shutdown();
            return;
        }

        // Tell the RFM to start measuring (NMT Start-Node).
        // device_processor_->send_startup(*this);

        int freq_ms = this->get_parameter("read_frequency_ms").as_int();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(freq_ms),
            std::bind(&GenericCanNode::readCANMessages, this));

        RCLCPP_INFO(this->get_logger(),
            "RFM 2.1 CAN node running. Poll interval = %d ms.", freq_ms);
    }

private:
    void readCANMessages()
    {
        TPCANMsg msg;
        // Drain all messages in the receive queue
        while (can_driver_->read_message(msg) == PCAN_ERROR_OK) {
            device_processor_->process_can_message(msg);
        }
        // Publish whatever was updated this cycle
        device_processor_->publish_data(*this);
    }

    CanDriverInterface::Ptr can_driver_;
    DeviceInterface::Ptr    device_processor_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// ─────────────────────────────────────────────────────────────────────────────
//  main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Use a temporary node to read launch / command-line parameters before
    // constructing the real node.
    auto tmp = std::make_shared<rclcpp::Node>("rfm21_param_node");

    tmp->declare_parameter("can_device",       "PCAN_USBBUS1");
    tmp->declare_parameter("can_baudrate",     250000);   // RFM default = 250 kbit/s
    tmp->declare_parameter("node_id",          15);       // 15=front, 16=rear
    tmp->declare_parameter("trigger_mode",     false);    // false = RFM sync-input mode

    std::string can_device   = tmp->get_parameter("can_device").as_string();
    int         can_baudrate = tmp->get_parameter("can_baudrate").as_int();

    // Build driver and device objects
    auto pcan_driver  = std::make_shared<PCANDriver>(can_device, can_baudrate,
                                                      tmp->get_logger());
    auto rfm21_device = std::make_shared<RFM21Device>(tmp->get_logger());

    // Give the device access to the driver so it can send NMT / trigger messages
    rfm21_device->set_can_driver(pcan_driver);

    // Build the ROS 2 node and spin
    auto node = std::make_shared<GenericCanNode>(pcan_driver, rfm21_device);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
