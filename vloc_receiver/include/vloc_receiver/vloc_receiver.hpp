#ifndef VLOC_RECEIVER_HPP
#define VLOC_RECEIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <vector>
#include <memory>
#include <thread>
#include <atomic>

#define PACKET_HEADER_1 0xAA
#define PACKET_HEADER_2 0x55

#pragma pack(push, 1)
struct VlocPacket
{
    uint8_t header1;
    uint8_t header2;
    uint16_t length;
    int32_t pos_x;
    int32_t pos_y;
    int32_t pos_z;
    int32_t vx;
    int32_t vy;
    int32_t vz;
    int32_t pitch;
    int32_t roll;
    int32_t yaw;
    uint8_t quality;
    uint8_t reloc;
    uint8_t checksum;
};
#pragma pack(pop)

class VlocReceiver : public rclcpp::Node
{
public:
    VlocReceiver();
    ~VlocReceiver();
    
    bool init();

private:
    void readSerialLoop();
    bool openSerialPort();
    void configureSerial();
    bool parsePacket(std::vector<uint8_t>& packet);
    uint8_t calculateChecksum(const uint8_t* data, size_t length);
    void publishVlocData(const VlocPacket& pkt);

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    
    int serial_fd_;
    std::string serial_port_;
    int baud_rate_;
    
    std::thread recv_thread_;
    std::atomic<bool> running_;
    
    static constexpr size_t BUFFER_SIZE = 1024;
};

#endif // VLOC_RECEIVER_HPP