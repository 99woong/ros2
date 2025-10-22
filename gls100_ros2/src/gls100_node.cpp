#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <PCANBasic.h>
#include <cstring>
#include <cmath>

class GLS100Node : public rclcpp::Node
{
public:
    GLS100Node() : Node("gls100_node")
    {
        // Parameters
        this->declare_parameter("can_device", "PCAN_USBBUS1");
        this->declare_parameter("can_baudrate", 500000);
        this->declare_parameter("node_id", 10);
        this->declare_parameter("frame_id", "gls100");
        
        std::string can_device = this->get_parameter("can_device").as_string();
        int baudrate = this->get_parameter("can_baudrate").as_int();
        node_id_ = this->get_parameter("node_id").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        
        // Publishers
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "gls100/pose", 10);
        velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "gls100/velocity", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::UInt16>(
            "gls100/status", 10);
        
        // Initialize PCAN
        if (!initPCAN(can_device, baudrate)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize PCAN");
            return;
        }
        
        // Timer for reading CAN messages
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&GLS100Node::readCANMessages, this));
        
        RCLCPP_INFO(this->get_logger(), "GLS100 Node initialized1111");
    }
    
    ~GLS100Node()
    {
        if (can_handle_ != 0) {
            CAN_Uninitialize(can_handle_);
        }
    }

private:
    bool initPCAN(const std::string& device, int baudrate)
    {
        // Convert device string to PCAN handle
        if (device == "PCAN_USBBUS1") {
            can_handle_ = PCAN_USBBUS1;
        } else if (device == "PCAN_USBBUS2") {
            can_handle_ = PCAN_USBBUS2;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unknown CAN device: %s", device.c_str());
            return false;
        }
        
        // Convert baudrate to PCAN format
        TPCANBaudrate pcan_baudrate;
        switch (baudrate) {
            case 125000: pcan_baudrate = PCAN_BAUD_125K; break;
            case 250000: pcan_baudrate = PCAN_BAUD_250K; break;
            case 500000: pcan_baudrate = PCAN_BAUD_500K; break;
            case 1000000: pcan_baudrate = PCAN_BAUD_1M; break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unsupported baudrate: %d", baudrate);
                return false;
        }
        
        // Initialize CAN
        TPCANStatus status = CAN_Initialize(can_handle_, pcan_baudrate, 0, 0, 0);
        if (status != PCAN_ERROR_OK) {
            RCLCPP_ERROR(this->get_logger(), "CAN_Initialize failed with error: 0x%x", status);
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "PCAN initialized successfully");
        return true;
    }
    
    void readCANMessages()
    {
        TPCANMsg msg;
        TPCANTimestamp timestamp;
        
        // Read all available messages
        while (CAN_Read(can_handle_, &msg, &timestamp) == PCAN_ERROR_OK) {
            // RCLCPP_INFO(this->get_logger(), "CAN msg received: id=0x%x len=%d", msg.ID, msg.LEN);
            processCANMessage(msg);
        }
    }
    
    void processCANMessage(const TPCANMsg& msg)
    {
        // Calculate base PDO IDs for this node
        uint32_t pdo1_id = 0x180 + node_id_;  // TPDO1: x, y position
        uint32_t pdo2_id = 0x280 + node_id_;  // TPDO2: status, z, angle, time offset
        uint32_t pdo3_id = 0x380 + node_id_;  // TPDO3: tag_id, x_vel, y_vel
        
        if (msg.ID == pdo1_id && msg.LEN == 8) {
            // TPDO1: X-Position (bytes 0-3), Y-Position (bytes 4-7)
            int32_t x_raw = *reinterpret_cast<const int32_t*>(&msg.DATA[0]);
            int32_t y_raw = *reinterpret_cast<const int32_t*>(&msg.DATA[4]);
            
            // Convert to meters (default scaling: 0.1mm per increment)
            x_position_ = x_raw * 0.0001;  // 0.1mm to meters
            y_position_ = y_raw * 0.0001;
            
            position_updated_ = true;
        }
        else if (msg.ID == pdo2_id && msg.LEN == 8) {
            // TPDO2: Status (bytes 0-1), Z-Position (bytes 2-3), 
            //        Angle (bytes 4-5), Time Offset (bytes 6-7)
            uint16_t status = *reinterpret_cast<const uint16_t*>(&msg.DATA[0]);
            int16_t z_raw = *reinterpret_cast<const int16_t*>(&msg.DATA[2]);
            int16_t angle_raw = *reinterpret_cast<const int16_t*>(&msg.DATA[4]);
            uint16_t time_offset = *reinterpret_cast<const uint16_t*>(&msg.DATA[6]);
            
            status_ = status;

            // std::cout << "status(device status) : " <<  (status & 0x8000) << std::endl;
            // std::cout << "status(marker type) : " <<  ((status & 0x00f0)>>4) << std::endl;
            // std::cout << "status(tag recognized) : " <<  ((status & 0x0004)>>2) << std::endl;
            // std::cout << "status(code read) : " <<  ((status && 0x0002)>>1) << std::endl;
            // std::cout << "status(code detected) : " <<  (status & 0x0001) << std::endl << std::endl;


            z_position_ = z_raw * 0.0001;  // 0.1mm to meters
            
            // Convert angle (default scaling: pi/32767 rad per increment)
            heading_ = angle_raw * M_PI / 32767.0;
            
            angle_updated_ = true;
        }
        else if (msg.ID == pdo3_id && msg.LEN == 8) {
            // TPDO3: Tag ID (bytes 0-3), X-Velocity (bytes 4-5), Y-Velocity (bytes 6-7)
            uint32_t tag_id = *reinterpret_cast<const uint32_t*>(&msg.DATA[0]);
            int16_t x_vel_raw = *reinterpret_cast<const int16_t*>(&msg.DATA[4]);
            int16_t y_vel_raw = *reinterpret_cast<const int16_t*>(&msg.DATA[6]);
            
            // Convert to m/s (default scaling: 0.1mm/s per increment)
            x_velocity_ = x_vel_raw * 0.0001;
            y_velocity_ = y_vel_raw * 0.0001;
            
            velocity_updated_ = true;
        }
        
        // Publish data when all required messages are received
        if (position_updated_ && angle_updated_) {
            publishPose();
            publishStatus();
            position_updated_ = false;
            angle_updated_ = false;
        }
        
        if (velocity_updated_) {
            publishVelocity();
            velocity_updated_ = false;
        }
    }
    
    void publishPose()
    {
        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = frame_id_;
        
        // Position
        msg.pose.position.x = x_position_;
        msg.pose.position.y = y_position_;
        msg.pose.position.z = z_position_;
        
        // Orientation (quaternion from heading angle)
        msg.pose.orientation.x = 0.0;
        msg.pose.orientation.y = 0.0;
        msg.pose.orientation.z = sin(heading_ / 2.0);
        msg.pose.orientation.w = cos(heading_ / 2.0);
        
        pose_pub_->publish(msg);
    }
    
    void publishVelocity()
    {
        auto msg = geometry_msgs::msg::TwistStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = frame_id_;
        
        msg.twist.linear.x = x_velocity_;
        msg.twist.linear.y = y_velocity_;
        msg.twist.linear.z = 0.0;
        
        msg.twist.angular.x = 0.0;
        msg.twist.angular.y = 0.0;
        msg.twist.angular.z = 0.0;
        
        velocity_pub_->publish(msg);
    }
    
    void publishStatus()
    {
        auto msg = std_msgs::msg::UInt16();
        msg.data = status_;
        status_pub_->publish(msg);
    }
    
    // ROS2 members
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // PCAN members
    TPCANHandle can_handle_;
    int node_id_;
    std::string frame_id_;
    
    // Data storage
    double x_position_ = 0.0;
    double y_position_ = 0.0;
    double z_position_ = 0.0;
    double heading_ = 0.0;
    double x_velocity_ = 0.0;
    double y_velocity_ = 0.0;
    uint16_t status_ = 0;
    
    // Update flags
    bool position_updated_ = false;
    bool angle_updated_ = false;
    bool velocity_updated_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GLS100Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

