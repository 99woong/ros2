#include <iostream>
#include <cstring>
#include <cstdint>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <atomic>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#pragma pack(push, 1)
struct VSLAMData {
    int64_t timestamp;      // unit : [nsec]
    int64_t mmx;           // unit : 0.0001 [m]
    int64_t mmy;           // unit : 0.0001 [m]
    int64_t mmz;           // unit : 0.0001 [m]
    uint64_t myaw;         // 0.001 [degree]
    uint8_t inlier_ratio;  // 0(min) ~ 100(max), good:30~50
};
#pragma pack(pop)

class VSLAMReceiver : public rclcpp::Node
{
private:
    int sockfd;
    struct sockaddr_in local_addr;
    std::atomic<bool> running;
    std::thread recv_thread;
    
    // ROS2 Publisher
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
    
    // 패킷 레이트 측정
    std::atomic<int> packet_count;
    std::chrono::steady_clock::time_point last_rate_time;
    std::thread rate_thread;

public:
    VSLAMReceiver() : Node("vslam_receiver"), sockfd(-1), running(false), packet_count(0)
    {
        // ROS2 Publisher 생성
        pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "vslam/pose", 10);

        last_rate_time = std::chrono::steady_clock::now();
    }

    ~VSLAMReceiver()
    {
        stop();
    }

    bool init(const std::string& local_ip, int local_port)
    {
        // UDP 소켓 생성
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "소켓 생성 실패");
            return false;
        }

        // 로컬 주소 설정
        memset(&local_addr, 0, sizeof(local_addr));
        local_addr.sin_family = AF_INET;
        local_addr.sin_addr.s_addr = inet_addr(local_ip.c_str());
        local_addr.sin_port = htons(local_port);

        // 바인드
        if (bind(sockfd, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "바인드 실패");
            close(sockfd);
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "VSLAM UDP Receiver 초기화 완료");
        RCLCPP_INFO(this->get_logger(), "로컬: %s:%d", local_ip.c_str(), local_port);
        return true;
    }

    void start()
    {
        running = true;
        
        // 수신 스레드 시작
        recv_thread = std::thread(&VSLAMReceiver::receiveLoop, this);
        
        // 레이트 측정 스레드 시작
        rate_thread = std::thread(&VSLAMReceiver::rateMonitorLoop, this);
        
        RCLCPP_INFO(this->get_logger(), "수신 스레드 시작");
    }

    void stop()
    {
        running = false;
        
        if (recv_thread.joinable())
            recv_thread.join();
        
        if (rate_thread.joinable())
            rate_thread.join();
        
        if (sockfd >= 0)
        {
            close(sockfd);
            sockfd = -1;
        }
    }

private:
    void receiveLoop()
    {
        uint8_t buffer[256];
        struct sockaddr_in sender_addr;
        socklen_t sender_len = sizeof(sender_addr);

        while (running)
        {
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(sockfd, &readfds);

            struct timeval tv;
            tv.tv_sec = 1;
            tv.tv_usec = 0;

            int ret = select(sockfd + 1, &readfds, NULL, NULL, &tv);
            if (ret > 0)
            {
                int recv_len = recvfrom(sockfd, buffer, sizeof(buffer), 0,
                                       (struct sockaddr*)&sender_addr, &sender_len);
                
                if (recv_len == sizeof(VSLAMData))
                {
                    processPacket(buffer);
                    packet_count++;
                }
                else if (recv_len > 0)
                {
                    RCLCPP_WARN(this->get_logger(), 
                        "잘못된 패킷 크기: %d (예상: %lu)", recv_len, sizeof(VSLAMData));
                }
            }
        }
    }

    void processPacket(uint8_t* buffer)
    {
        VSLAMData data;
        memcpy(&data, buffer, sizeof(data));

        // ROS2 PoseStamped 메시지 생성
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        
        // 헤더 설정
        pose_msg.header.stamp.sec = data.timestamp / 1000000000;
        pose_msg.header.stamp.nanosec = data.timestamp % 1000000000;
        pose_msg.header.frame_id = "map";

        // Position 설정 (0.0001m -> m)
        pose_msg.pose.position.x = data.mmx * 0.0001;
        pose_msg.pose.position.y = data.mmy * 0.0001;
        pose_msg.pose.position.z = data.mmz * 0.0001;

        // Orientation 설정 (Yaw만 있으므로 Z축 회전 쿼터니언 변환)
        double yaw_rad = (data.myaw * 0.001) * M_PI / 180.0;  // degree -> radian
        pose_msg.pose.orientation.x = 0.0;
        pose_msg.pose.orientation.y = 0.0;
        pose_msg.pose.orientation.z = std::sin(yaw_rad / 2.0);
        pose_msg.pose.orientation.w = std::cos(yaw_rad / 2.0);

        // ROS2 토픽 발행
        pose_pub->publish(pose_msg);

        // 로그 출력 (너무 많으면 성능 저하 가능)
        static int log_count = 0;
        if (++log_count % 10 == 0)  // 10개당 1번만 출력
        {
            RCLCPP_INFO(this->get_logger(),
                "수신: pos(%.4f, %.4f, %.4f) m, yaw: %.2f deg, inlier: %d%%",
                pose_msg.pose.position.x,
                pose_msg.pose.position.y,
                pose_msg.pose.position.z,
                data.myaw * 0.001,
                data.inlier_ratio);
        }
    }

    void rateMonitorLoop()
    {
        while (running)
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                current_time - last_rate_time).count();
            
            if (elapsed >= 1000)
            {
                int count = packet_count.exchange(0);
                double rate = count * 1000.0 / elapsed;
                
                std::cout << std::fixed << std::setprecision(2)
                          << "[수신 레이트] " << rate << " Hz (" 
                          << count << " packets/sec)" << std::endl;
                
                last_rate_time = current_time;
            }
        }
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    std::string local_ip = "192.168.54.100";
    int local_port = 8086;

    // 커맨드 라인 인자 처리
    if (argc >= 2)
        local_ip = argv[1];
    if (argc >= 3)
        local_port = std::atoi(argv[2]);

    auto receiver = std::make_shared<VSLAMReceiver>();

    if (!receiver->init(local_ip, local_port))
    {
        RCLCPP_ERROR(receiver->get_logger(), "초기화 실패");
        return -1;
    }

    receiver->start();

    // ROS2 spin
    rclcpp::spin(receiver);

    rclcpp::shutdown();
    return 0;
}