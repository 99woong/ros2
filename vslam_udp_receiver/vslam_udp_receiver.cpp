#include <iostream>
#include <fstream>
#include <thread>
#include <atomic>
#include <chrono>
#include <cstring>
#include <sstream>
#include <cmath>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#pragma pack(push, 1)
struct VSLAMData {
    int64_t timestamp;     // nsec
    int64_t mmx;           // 0.001 m
    int64_t mmy;
    int64_t mmz;
    uint64_t myaw;         // 0.001 deg
    uint8_t inlier_ratio;
};
#pragma pack(pop)

class VSLAMReceiver : public rclcpp::Node
{
public:
    VSLAMReceiver() : Node("vslam_udp_receiver") {}

    bool init(const std::string& ip, int port)
    {
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0) {
            RCLCPP_ERROR(get_logger(), "socket failed");
            return false;
        }

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = inet_addr(ip.c_str());
        addr.sin_port = htons(port);

        if (bind(sockfd_, (sockaddr*)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(get_logger(), "bind failed");
            return false;
        }

        pose_pub_ =
            create_publisher<geometry_msgs::msg::PoseStamped>("vslam/pose", 10);

        running_ = true;
        recv_thread_ = std::thread(&VSLAMReceiver::recvLoop, this);
        key_thread_  = std::thread(&VSLAMReceiver::keyboardLoop, this);

        RCLCPP_INFO(get_logger(), "UDP receiver ready (%s:%d)",
                    ip.c_str(), port);
        return true;
    }

    ~VSLAMReceiver()
    {
        running_ = false;
        if (recv_thread_.joinable()) recv_thread_.join();
        if (key_thread_.joinable())  key_thread_.join();
        if (ofs_.is_open()) ofs_.close();
        if (sockfd_ >= 0) close(sockfd_);
    }

private:
    /* ---------------- UDP receive ---------------- */

    void recvLoop()
    {
        uint8_t buf[256];

        while (running_) {
            int len = recv(sockfd_, buf, sizeof(buf), 0);
            if (len != sizeof(VSLAMData)) continue;

            VSLAMData d;
            std::memcpy(&d, buf, sizeof(d));

            publishPose(d);
            logToFile(d);
        }
    }

    void publishPose(const VSLAMData& d)
    {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp.sec =
            static_cast<int32_t>(d.timestamp / 1000000000);
        msg.header.stamp.nanosec =
            static_cast<uint32_t>(d.timestamp % 1000000000);
        msg.header.frame_id = "map";

        msg.pose.position.x = d.mmx * 0.001;
        msg.pose.position.y = d.mmy * 0.001;
        msg.pose.position.z = d.mmz * 0.001;

        double yaw =
            (d.myaw * 0.001) * M_PI / 180.0;

        msg.pose.orientation.x = 0.0;
        msg.pose.orientation.y = 0.0;
        msg.pose.orientation.z = std::sin(yaw * 0.5);
        msg.pose.orientation.w = std::cos(yaw * 0.5);

        pose_pub_->publish(msg);
    }

    /* ---------------- Logging ---------------- */

    void logToFile(const VSLAMData& d)
    {
        if (!logging_) return;

        ofs_ << d.timestamp << ","
             << d.mmx * 0.001 << ","
             << d.mmy * 0.001 << ","
             << d.mmz * 0.001 << ","
             << d.myaw * 0.001 << "\n";
    }

    /* ---------------- Keyboard ---------------- */

    void keyboardLoop()
    {
        while (running_) {
            char c = std::cin.get();
            if (c == 's') startLogging();
            if (c == 'e') stopLogging();
        }
    }

    void startLogging()
    {
        stopLogging();

        auto t = std::time(nullptr);
        std::ostringstream name;
        name << "vslam_log_" << t << ".csv";

        ofs_.open(name.str());
        ofs_ << "timestamp_nsec,x,y,z,yaw_deg\n";
        logging_ = true;

        RCLCPP_INFO(get_logger(), "Logging started: %s",
                    name.str().c_str());
    }

    void stopLogging()
    {
        if (!logging_) return;
        ofs_.close();
        logging_ = false;
        RCLCPP_INFO(get_logger(), "Logging stopped");
    }

private:
    int sockfd_{-1};
    std::atomic<bool> running_{false};

    std::thread recv_thread_;
    std::thread key_thread_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    std::ofstream ofs_;
    std::atomic<bool> logging_{false};
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    std::string ip = "192.168.54.100";
    int port = 8086;

    if (argc >= 2) ip = argv[1];
    if (argc >= 3) port = std::atoi(argv[2]);

    auto node = std::make_shared<VSLAMReceiver>();
    if (!node->init(ip, port)) {
        rclcpp::shutdown();
        return -1;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
