#ifndef VLOC_RECEIVER_HPP
#define VLOC_RECEIVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <vloc_receiver/msg/vloc_data.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <vector>
#include <memory>

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
    void timerCallback();
    bool openSerialPort();
    bool readSerialData();
    bool parsePacket();
    uint8_t calculateChecksum(const uint8_t* data, size_t length);
    void publishVlocData(const VlocPacket& packet);

    rclcpp::Publisher<vloc_receiver::msg::VlocData>::SharedPtr vloc_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    int serial_fd_;
    std::string serial_port_;
    int baud_rate_;
    
    std::vector<uint8_t> buffer_;
    static constexpr size_t BUFFER_SIZE = 1024;
};

#endif // VLOC_RECEIVER_HPP
