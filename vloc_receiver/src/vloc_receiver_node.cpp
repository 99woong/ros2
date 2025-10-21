#include "vloc_receiver/vloc_receiver.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

VlocReceiver::VlocReceiver() 
    : Node("vloc_receiver_node"), serial_fd_(-1), running_(false)
{
    // Declare and get parameters
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud_rate", 115200);
    
    this->get_parameter("serial_port", serial_port_);
    this->get_parameter("baud_rate", baud_rate_);
    
    // Create publisher for PoseStamped
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "vloc/pose", 10);
    
    RCLCPP_INFO(this->get_logger(), "VLOC Receiver Node initialized");
}

VlocReceiver::~VlocReceiver()
{
    running_ = false;
    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }
    if (serial_fd_ >= 0) {
        close(serial_fd_);
    }
    RCLCPP_INFO(this->get_logger(), "VLOC Receiver Node shutdown");
}

bool VlocReceiver::init()
{
    if (!openSerialPort()) {
        return false;
    }
    
    configureSerial();
    
    // 수신 스레드 시작
    running_ = true;
    recv_thread_ = std::thread(&VlocReceiver::readSerialLoop, this);
    
    RCLCPP_INFO(this->get_logger(), "Serial receive thread started");
    return true;
}

bool VlocReceiver::openSerialPort()
{
    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    
    if (serial_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", 
                     serial_port_.c_str());
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), 
                "Serial port opened successfully: %s", 
                serial_port_.c_str());
    return true;
}

void VlocReceiver::configureSerial()
{
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    
    if (tcgetattr(serial_fd_, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr");
        return;
    }
    
    // Raw mode 설정
    cfmakeraw(&tty);
    
    // Baud rate 설정
    speed_t speed = B115200;
    switch(baud_rate_) {
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
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_iflag = 0;
    
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;
    
    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error from tcsetattr");
        return;
    }
    
    // 버퍼 비우기
    tcflush(serial_fd_, TCIFLUSH);
    
    RCLCPP_INFO(this->get_logger(), 
                "Serial port configured: %d baud, 8N1", baud_rate_);
}

void VlocReceiver::readSerialLoop()
{
    uint8_t buf[256];
    std::vector<uint8_t> packet;
    packet.reserve(BUFFER_SIZE);
    
    RCLCPP_INFO(this->get_logger(), "Serial read loop started");
    
    while (running_) {
        int n = read(serial_fd_, buf, sizeof(buf));
        
        if (n > 0) {
            // 수신된 데이터를 패킷 버퍼에 추가
            for (int i = 0; i < n; ++i) {
                packet.push_back(buf[i]);
            }
            
            // 패킷 파싱 시도
            parsePacket(packet);
        } else if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // 논블로킹 모드에서 데이터가 없을 때
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            } else {
                RCLCPP_ERROR(this->get_logger(), "Serial read error: %d", errno);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        } else {
            // n == 0
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Serial read loop stopped");
}

uint8_t VlocReceiver::calculateChecksum(const uint8_t* data, size_t length)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

bool VlocReceiver::parsePacket(std::vector<uint8_t>& packet)
{
    bool parsed = false;
    
    while (packet.size() >= sizeof(VlocPacket)) {
        // 헤더 찾기
        auto it = packet.begin();
        bool header_found = false;
        
        for (; it != packet.end() - 1; ++it) {
            if (*it == PACKET_HEADER_1 && *(it + 1) == PACKET_HEADER_2) {
                header_found = true;
                break;
            }
        }
        
        if (!header_found) {
            // 마지막 바이트 제외하고 모두 삭제
            if (packet.size() > 1) {
                packet.erase(packet.begin(), packet.end() - 1);
            }
            return parsed;
        }
        
        // 헤더 이전 데이터 삭제
        if (it != packet.begin()) {
            packet.erase(packet.begin(), it);
        }
        
        // 패킷 크기 확인
        if (packet.size() < sizeof(VlocPacket)) {
            return parsed;
        }
        
        // 패킷 복사
        VlocPacket vloc_pkt;
        memcpy(&vloc_pkt, packet.data(), sizeof(VlocPacket));
        
        // 체크섬 검증
        uint8_t calc_checksum = calculateChecksum(
            packet.data() + 2,  // length 필드부터
            sizeof(VlocPacket) - 3  // header(2) + checksum(1) 제외
        );
        
        if (calc_checksum == vloc_pkt.checksum) {
            // 패킷 발행
            publishVlocData(vloc_pkt);
            parsed = true;
            
            // 처리된 패킷 삭제
            packet.erase(packet.begin(), packet.begin() + sizeof(VlocPacket));
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Checksum mismatch! Expected: 0x%02X, Got: 0x%02X", 
                                 calc_checksum, vloc_pkt.checksum);
            // 헤더만 삭제하고 다음 헤더 찾기
            packet.erase(packet.begin(), packet.begin() + 2);
        }
    }
    
    return parsed;
}
struct PoseData {
    double x;
    double y;
    double yaw;
};

void VlocReceiver::publishVlocData(const VlocPacket& pkt)
{
    auto msg = geometry_msgs::msg::PoseStamped();
    PoseData pose;
    
    // 헤더 설정
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";  // 또는 "odom", "vloc_frame" 등
    
    // 위치 설정 (mm 단위를 m 단위로 변환)
    msg.pose.position.x = pkt.pos_x / 1000.0;  // mm -> m
    msg.pose.position.y = pkt.pos_y / 1000.0;
    msg.pose.position.z = pkt.pos_z / 1000.0;
    
    // 자세 설정 (pitch, roll, yaw를 quaternion으로 변환)
    // 단위가 degree라면 radian으로 변환 필요
    // RPY를 Quaternion으로 변환
    double roll_deg = pkt.roll / 1000.0;   // 단위 확인 필요
    double pitch_deg = pkt.pitch / 1000.0;
    double yaw_deg = pkt.yaw / 100.0;

    // Degree를 Quaternion 공식에 사용하기 위한 Radian으로 변환
    double roll = roll_deg * M_PI / 180.0;
    double pitch = pitch_deg * M_PI / 180.0;
    double yaw = yaw_deg * M_PI / 180.0; // **<-- 핵심 수정 사항!**    
    
    // Quaternion 계산 (수동 계산)
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    
    // msg.pose.orientation.w = cr * cp * cy + sr * sp * sy;
    // msg.pose.orientation.x = sr * cp * cy - cr * sp * sy;
    // msg.pose.orientation.y = cr * sp * cy + sr * cp * sy;
    // msg.pose.orientation.z = cr * cp * sy - sr * sp * cy;

    // tf2::Quaternion q(
    //     msg.pose.orientation.x,
    //     msg.pose.orientation.y,
    //     msg.pose.orientation.z,
    //     msg.pose.orientation.w
    // );
    
    // tf2::Matrix3x3 m(q);
    // double roll_, pitch_, yaw_;
    // m.getRPY(roll_, pitch_, yaw_);
    // pose.yaw = yaw_ * 180.0 / M_PI; // Convert to degrees\

    // std::cout << "after yaw: " << pose.yaw <<std::endl;    

    // 토픽 발행
    pose_pub_->publish(msg);
    
    // 로그 출력 (1초에 한 번)
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "VLOC Pose: X=%.3f, Y=%.3f, Z=%.3f, Yaw=%.3f, Quality=%d, Reloc=%d",
                         msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                         yaw_deg, pkt.quality, pkt.reloc);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<VlocReceiver>();
    
    if (!node->init()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize VLOC receiver");
        return -1;
    }
    
    RCLCPP_INFO(node->get_logger(), "VLOC Receiver started - spinning...");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}