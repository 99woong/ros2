#include <rclcpp/rclcpp.hpp>
#include <PCANBasic.h> 
#include "gls100_driver/interfaces.hpp"
#include "gls100_driver/pcan_driver.hpp"
#include "gls100_driver/gls100_device.hpp"

class GenericCanNode : public rclcpp::Node
{
public:
    GenericCanNode(CanDriverInterface::Ptr can_driver, DeviceInterface::Ptr device_processor)
    : Node("can_driver_node"),
      can_driver_(can_driver),
      device_processor_(device_processor)
    {
        this->declare_parameter("read_frequency_ms", 10);
        device_processor_->setup(*this);

        if (!can_driver_->initialize()) 
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize CAN Driver. Shutting down node.");
            rclcpp::shutdown();
            return;
        }
        
        int read_frequency_ms = this->get_parameter("read_frequency_ms").as_int();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(read_frequency_ms),
            std::bind(&GenericCanNode::readCANMessages, this));
        
        RCLCPP_INFO(this->get_logger(), "Generic CAN Node initialized and running.");
    }
    
private:
    void readCANMessages()
    {
        TPCANMsg msg;
        while (can_driver_->read_message(msg) == PCAN_ERROR_OK) 
        {
            device_processor_->process_can_message(msg);
        }

        device_processor_->publish_data(*this);
    }
    
    CanDriverInterface::Ptr can_driver_;
    DeviceInterface::Ptr device_processor_;
    
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto temp_node = std::make_shared<rclcpp::Node>("temp_node");
    temp_node->declare_parameter("can_device", "PCAN_USBBUS1");
    temp_node->declare_parameter("can_baudrate", 500000);
    
    std::string can_device = temp_node->get_parameter("can_device").as_string();
    int baudrate = temp_node->get_parameter("can_baudrate").as_int();
    auto pcan_driver = std::make_shared<PCANDriver>(can_device, baudrate, temp_node->get_logger());
    
    auto gls100_device = std::make_shared<GLS100Device>(temp_node->get_logger());

    auto node = std::make_shared<GenericCanNode>(pcan_driver, gls100_device);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}