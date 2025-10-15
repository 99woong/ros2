#include "vloc_receiver/vloc_receiver.hpp"

VlocReceiver::VlocReceiver() 
    : Node("vloc_receiver_node"), serial_fd_(-1)
{
    // Declare and get parameters
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud_rate", 115200);
    
    this->get_parameter("serial_port", serial_port_);
    this->get_parameter("baud_rate", baud_rate_);
    
    // Create publisher
    vloc_pub_ = this->create_publisher<vloc_receiver::msg::VlocData>(
        "vloc_data", 10);
    
    // Reserve buffer
    buffer_.reserve(BUFFER_SIZE);
    
    // Create timer (10ms = 100Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&VlocReceiver::timerCallback, this));
}

VlocReceiver::~VlocReceiver()
{
    if(serial_fd_ >= 0)
    {
        close(serial_fd_);
    }
}

bool VlocReceiver::init()
{
    return openSerialPort();
}

bool VlocReceiver::openSerialPort()
{
    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    
    if(serial_fd_ < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", 
                     serial_port_.c_str());
        return false;
    }
    
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    
    if(tcgetattr(serial_fd_, &tty) != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr");
        return false;
    }
    
    // Baud rate 설정
    speed_t speed = B115200;
    switch(baud_rate_)
    {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        case 460800: speed = B460800; break;
        case 921600: speed = B921600; break;
        default: speed = B115200;
    }
    
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
    
    // 8N1 설정
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;
    
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    
    if(tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error from tcsetattr");
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), 
                "Serial port opened successfully: %s at %d baud", 
                serial_port_.c_str(), baud_rate_);
    return true;
}

void VlocReceiver::timerCallback()
{
    if(readSerialData())
    {
        parsePacket();
    }
}

uint8_t VlocReceiver::calculateChecksum(const uint8_t* data, size_t length)
{
    uint8_t checksum = 0;
    for(size_t i = 0; i < length; i++)
    {
        checksum ^= data[i];
    }
    return checksum;
}

bool VlocReceiver::readSerialData()
{
    uint8_t temp_buf[256];
    int n = read(serial_fd_, temp_buf, sizeof(temp_buf));
    
    if(n > 0)
    {
        buffer_.insert(buffer_.end(), temp_buf, temp_buf + n);
        return true;
    }
    
    return false;
}

bool VlocReceiver::parsePacket()
{
    while(buffer_.size() >= sizeof(VlocPacket))
    {
        // 헤더 찾기
        auto it = buffer_.begin();
        bool header_found = false;
        
        for(; it != buffer_.end() - 1; ++it)
        {
            if(*it == PACKET_HEADER_1 && *(it + 1) == PACKET_HEADER_2)
            {
                header_found = true;
                break;
            }
        }
        
        if(!header_found)
        {
            // 마지막 바이트 제외하고 모두 삭제
            if(buffer_.size() > 1)
            {
                buffer_.erase(buffer_.begin(), buffer_.end() - 1);
            }
            return false;
        }
        
        // 헤더 이전 데이터 삭제
        if(it != buffer_.begin())
        {
            buffer_.erase(buffer_.begin(), it);
        }
        
        // 패킷 크기 확인
        if(buffer_.size() < sizeof(VlocPacket))
        {
            return false;
        }
        
        // 패킷 복사
        VlocPacket packet;
        memcpy(&packet, buffer_.data(), sizeof(VlocPacket));
        
        // 체크섬 검증
        uint8_t calc_checksum = calculateChecksum(
            buffer_.data() + 2,  // length 필드부터
            sizeof(VlocPacket) - 3  // header(2) + checksum(1) 제외
        );
        
        if(calc_checksum == packet.checksum)
        {
            // 패킷 발행
            publishVlocData(packet);
            
            // 처리된 패킷 삭제
            buffer_.erase(buffer_.begin(), buffer_.begin() + sizeof(VlocPacket));
            return true;
        }
        else
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Checksum mismatch! Expected: 0x%02X, Got: 0x%02X", 
                                 calc_checksum, packet.checksum);
            // 헤더만 삭제하고 다음 헤더 찾기
            buffer_.erase(buffer_.begin(), buffer_.begin() + 2);
        }
    }
    
    return false;
}

void VlocReceiver::publishVlocData(const VlocPacket& packet)
{
    auto msg = vloc_receiver::msg::VlocData();
    
    msg.header.stamp = this->now();
    msg.header.frame_id = "vloc";
    
    msg.pos_x = packet.pos_x;
    msg.pos_y = packet.pos_y;
    msg.pos_z = packet.pos_z;
    msg.vx = packet.vx;
    msg.vy = packet.vy;
    msg.vz = packet.vz;
    msg.pitch = packet.pitch;
    msg.roll = packet.roll;
    msg.yaw = packet.yaw;
    msg.quality = packet.quality;
    msg.reloc = packet.reloc;
    
    vloc_pub_->publish(msg);
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "VLOC: X=%d, Y=%d, Yaw=%d, Quality=%d, Reloc=%d",
                         packet.pos_x, packet.pos_y, packet.yaw, 
                         packet.quality, packet.reloc);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<VlocReceiver>();
    
    if(!node->init())
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize VLOC receiver");
        return -1;
    }
    
    RCLCPP_INFO(node->get_logger(), "VLOC Receiver started");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}

