#include <iostream>
#include <cstring>
#include <cstdint>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <atomic>
#include <cmath>
#include <csignal>

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

class VSLAMSender
{
private:
    int sockfd;
    struct sockaddr_in target_addr;
    std::atomic<bool> running;
    int packet_count;
    
    // 시뮬레이션 변수
    double sim_time;
    double radius;
    double angular_velocity;

public:
    VSLAMSender() : sockfd(-1), running(false), packet_count(0), 
                    sim_time(0.0), radius(5.0), angular_velocity(0.1) {}

    ~VSLAMSender()
    {
        stop();
    }

    bool init(const std::string& target_ip, int target_port)
    {
        // UDP 소켓 생성
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd < 0)
        {
            std::cerr << "소켓 생성 실패" << std::endl;
            return false;
        }

        // 타겟 주소 설정
        memset(&target_addr, 0, sizeof(target_addr));
        target_addr.sin_family = AF_INET;
        target_addr.sin_addr.s_addr = inet_addr(target_ip.c_str());
        target_addr.sin_port = htons(target_port);

        std::cout << "VSLAM UDP Sender 초기화 완료" << std::endl;
        std::cout << "타겟: " << target_ip << ":" << target_port << std::endl;
        return true;
    }

    void sendPacket()
    {
        VSLAMData data;
        
        // 현재 타임스탬프 (나노초)
        auto now = std::chrono::system_clock::now();
        data.timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
            now.time_since_epoch()).count();

        // 원형 궤적 시뮬레이션 (0.0001m 단위)
        // x = r * cos(wt), y = r * sin(wt)
        data.mmx = static_cast<int64_t>(radius * std::cos(sim_time) * 10000);  // m -> 0.0001m
        data.mmy = static_cast<int64_t>(radius * std::sin(sim_time) * 10000);
        data.mmz = static_cast<int64_t>(0.5 * 10000);  // 고정 높이 0.5m

        // Yaw 각도 (0.001 degree 단위)
        double yaw_deg = std::fmod(sim_time * 180.0 / M_PI * angular_velocity * 10.0, 360.0);
        if (yaw_deg < 0) yaw_deg += 360.0;
        data.myaw = static_cast<uint64_t>(yaw_deg * 1000);

        // Inlier ratio (30~50 사이 랜덤 시뮬레이션)
        data.inlier_ratio = 30 + (packet_count % 21);

        // 패킷 전송
        ssize_t sent = sendto(sockfd, &data, sizeof(data), 0,
                             (struct sockaddr*)&target_addr, sizeof(target_addr));

        if (sent > 0)
        {
            packet_count++;
            std::cout << "[송신 #" << packet_count << "] "
                      << "timestamp: " << data.timestamp << " ns, "
                      << "pos: (" << data.mmx * 0.0001 << ", " 
                      << data.mmy * 0.0001 << ", " 
                      << data.mmz * 0.0001 << ") m, "
                      << "yaw: " << data.myaw * 0.001 << " deg, "
                      << "inlier: " << (int)data.inlier_ratio << "%"
                      << std::endl;
        }
        else
        {
            std::cerr << "전송 실패" << std::endl;
        }

        // 시뮬레이션 시간 증가
        sim_time += angular_velocity;
    }

    void startSending()
    {
        running = true;
        std::cout << "\n10Hz로 패킷 송신 시작... (Ctrl+C로 종료)\n" << std::endl;

        auto next_time = std::chrono::steady_clock::now();
        const auto interval = std::chrono::milliseconds(100);  // 10Hz = 100ms

        while (running)
        {
            sendPacket();
            
            // 정확한 10Hz 유지
            next_time += interval;
            std::this_thread::sleep_until(next_time);
        }

        std::cout << "\n총 " << packet_count << "개 패킷 송신 완료" << std::endl;
    }

    void stop()
    {
        running = false;
        if (sockfd >= 0)
        {
            close(sockfd);
            sockfd = -1;
        }
    }
};

int main(int argc, char* argv[])
{
    std::string target_ip = "192.168.1.11";
    int target_port = 7777;

    if (argc >= 2)
        target_ip = argv[1];
    if (argc >= 3)
        target_port = std::atoi(argv[2]);

    VSLAMSender sender;

    if (!sender.init(target_ip, target_port))
    {
        std::cerr << "초기화 실패" << std::endl;
        return -1;
    }

    // Ctrl+C 핸들링을 위한 전역 포인터
    static VSLAMSender* g_sender = &sender;
    signal(SIGINT, [](int) {
        std::cout << "\n종료 신호 수신..." << std::endl;
        if (g_sender) {
            g_sender->stop();
        }
        exit(0);
    });

    sender.startSending();

    return 0;
}

// #include <iostream>
// #include <cstring>
// #include <cstdint>
// #include <arpa/inet.h>
// #include <sys/socket.h>
// #include <unistd.h>
// #include <thread>
// #include <chrono>
// #include <atomic>
// #include <cmath>

// #pragma pack(push, 1)
// struct VSLAMData {
//     int64_t timestamp;      // unit : [nsec]
//     int64_t mmx;           // unit : 0.0001 [m]
//     int64_t mmy;           // unit : 0.0001 [m]
//     int64_t mmz;           // unit : 0.0001 [m]
//     uint64_t myaw;         // 0.001 [degree]
//     uint8_t inlier_ratio;  // 0(min) ~ 100(max), good:30~50
// };
// #pragma pack(pop)

// class VSLAMSender
// {
// private:
//     int sockfd;
//     struct sockaddr_in target_addr;
//     std::atomic<bool> running;
//     int packet_count;
    
//     // 시뮬레이션 변수
//     double sim_time;
//     double radius;
//     double angular_velocity;

// public:
//     VSLAMSender() : sockfd(-1), running(false), packet_count(0), 
//                     sim_time(0.0), radius(5.0), angular_velocity(0.1) {}

//     ~VSLAMSender()
//     {
//         stop();
//     }

//     bool init(const std::string& target_ip, int target_port)
//     {
//         // UDP 소켓 생성
//         sockfd = socket(AF_INET, SOCK_DGRAM, 0);
//         if (sockfd < 0)
//         {
//             std::cerr << "소켓 생성 실패" << std::endl;
//             return false;
//         }

//         // 타겟 주소 설정
//         memset(&target_addr, 0, sizeof(target_addr));
//         target_addr.sin_family = AF_INET;
//         target_addr.sin_addr.s_addr = inet_addr(target_ip.c_str());
//         target_addr.sin_port = htons(target_port);

//         std::cout << "VSLAM UDP Sender 초기화 완료" << std::endl;
//         std::cout << "타겟: " << target_ip << ":" << target_port << std::endl;
//         return true;
//     }

//     void sendPacket()
//     {
//         VSLAMData data;
        
//         // 현재 타임스탬프 (나노초)
//         auto now = std::chrono::system_clock::now();
//         data.timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
//             now.time_since_epoch()).count();

//         // 원형 궤적 시뮬레이션 (0.0001m 단위)
//         // x = r * cos(wt), y = r * sin(wt)
//         data.mmx = static_cast<int64_t>(radius * std::cos(sim_time) * 10000);  // m -> 0.0001m
//         data.mmy = static_cast<int64_t>(radius * std::sin(sim_time) * 10000);
//         data.mmz = static_cast<int64_t>(0.5 * 10000);  // 고정 높이 0.5m

//         // Yaw 각도 (0.001 degree 단위)
//         double yaw_deg = std::fmod(sim_time * 180.0 / M_PI * angular_velocity * 10.0, 360.0);
//         if (yaw_deg < 0) yaw_deg += 360.0;
//         data.myaw = static_cast<uint64_t>(yaw_deg * 1000);

//         // Inlier ratio (30~50 사이 랜덤 시뮬레이션)
//         data.inlier_ratio = 30 + (packet_count % 21);

//         // 패킷 전송
//         ssize_t sent = sendto(sockfd, &data, sizeof(data), 0,
//                              (struct sockaddr*)&target_addr, sizeof(target_addr));

//         if (sent > 0)
//         {
//             packet_count++;
//             std::cout << "[송신 #" << packet_count << "] "
//                       << "timestamp: " << data.timestamp << " ns, "
//                       << "pos: (" << data.mmx * 0.0001 << ", " 
//                       << data.mmy * 0.0001 << ", " 
//                       << data.mmz * 0.0001 << ") m, "
//                       << "yaw: " << data.myaw * 0.001 << " deg, "
//                       << "inlier: " << (int)data.inlier_ratio << "%"
//                       << std::endl;
//         }
//         else
//         {
//             std::cerr << "전송 실패" << std::endl;
//         }

//         // 시뮬레이션 시간 증가
//         sim_time += angular_velocity;
//     }

//     void startSending()
//     {
//         running = true;
//         std::cout << "\n10Hz로 패킷 송신 시작... (Ctrl+C로 종료)\n" << std::endl;

//         auto next_time = std::chrono::steady_clock::now();
//         const auto interval = std::chrono::milliseconds(100);  // 10Hz = 100ms

//         while (running)
//         {
//             sendPacket();
            
//             // 정확한 10Hz 유지
//             next_time += interval;
//             std::this_thread::sleep_until(next_time);
//         }

//         std::cout << "\n총 " << packet_count << "개 패킷 송신 완료" << std::endl;
//     }

//     void stop()
//     {
//         running = false;
//         if (sockfd >= 0)
//         {
//             close(sockfd);
//             sockfd = -1;
//         }
//     }
// };

// int main(int argc, char* argv[])
// {
//     std::string target_ip = "192.168.1.11";
//     int target_port = 7777;

//     if (argc >= 2)
//         target_ip = argv[1];
//     if (argc >= 3)
//         target_port = std::atoi(argv[2]);

//     VSLAMSender sender;

//     if (!sender.init(target_ip, target_port))
//     {
//         std::cerr << "초기화 실패" << std::endl;
//         return -1;
//     }

//     // Ctrl+C 핸들링
//     signal(SIGINT, [](int) {
//         std::cout << "\n종료 신호 수신..." << std::endl;
//         exit(0);
//     });

//     sender.startSending();

//     return 0;
// }