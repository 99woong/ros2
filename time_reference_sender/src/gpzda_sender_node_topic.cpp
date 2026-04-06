#include <rclcpp/rclcpp.hpp>
#include <nmea_msgs/msg/sentence.hpp> // NMEA 메시지 타입 추가

#include <sched.h>
#include <time.h>
#include <chrono>
#include <string>
#include <iomanip>
#include <sstream>
#include <thread>

class GPZDASender : public rclcpp::Node
{
public:
    GPZDASender() : Node("gpzda_sender")
    {
        // 파라미터 선언 (port, baudrate는 이제 필요 없으므로 제거하거나 무시)
        advance_ms_ = this->declare_parameter<double>("advance_ms", 20.0);
        topic_name_ = this->declare_parameter<std::string>("nmea_topic", "/nmea");

        // Publisher 생성: nmea_msgs/msg/Sentence 타입
        nmea_pub_ = this->create_publisher<nmea_msgs::msg::Sentence>(topic_name_, 10);

        // RT priority thread 실행
        sender_thread_ = std::thread(&GPZDASender::run, this);

        RCLCPP_INFO(this->get_logger(),
            "GPZDA topic sender started (target topic=%s, advance=%.2f ms)", 
            topic_name_.c_str(), advance_ms_);
    }

    ~GPZDASender()
    {
        if (sender_thread_.joinable()) {
            sender_thread_.join();
        }
    }

private:
    double advance_ms_;
    std::string topic_name_;
    std::thread sender_thread_;
    rclcpp::Publisher<nmea_msgs::msg::Sentence>::SharedPtr nmea_pub_;

    // ---------------- GPZDA 생성 (NMEA 규격) ----------------
    std::string make_gpzda(time_t next_sec)
    {
        struct tm tm{};
        gmtime_r(&next_sec, &tm);

        // GPZDA 포맷: $GPZDA,hhmmss.ss,dd,mm,yyyy,hh,mm*CS
        // PPS 직후에 적용될 '다음 초'를 보냄
        char buf[128];
        snprintf(buf, sizeof(buf),
            "GPZDA,%02d%02d%02d.00,%02d,%02d,%04d,00,00",
            tm.tm_hour, tm.tm_min, tm.tm_sec,
            tm.tm_mday, tm.tm_mon + 1, tm.tm_year + 1900);

        // Checksum 계산
        uint8_t cs = 0;
        for (int i = 0; buf[i] != '\0'; ++i)
            cs ^= buf[i];

        char final_msg[128];
        snprintf(final_msg, sizeof(final_msg), "$%s*%02X", buf, cs);

        return std::string(final_msg);
    }

    // ---------------- RT Thread ----------------
    void run()
    {
        // 리눅스 실시간 우선순위 설정
        struct sched_param param;
        param.sched_priority = 80;
        if(sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
            RCLCPP_WARN(this->get_logger(), "Failed to set RT priority. Are you root?");
        }

        while (rclcpp::ok())
        {
            struct timespec now;
            clock_gettime(CLOCK_REALTIME, &now);

            // 다음 정각 초 계산
            time_t next_sec = now.tv_sec + 1;

            // 전송 목표 시점 계산 (정각보다 advance_ms만큼 일찍 전송)
            double advance_sec = advance_ms_ / 1000.0;
            struct timespec target;
            target.tv_sec = next_sec;
            target.tv_nsec = 0;

            long advance_ns = (long)(advance_sec * 1e9);
            target.tv_nsec -= advance_ns;
            if (target.tv_nsec < 0) {
                target.tv_sec -= 1;
                target.tv_nsec += 1000000000;
            }

            // 정밀 대기 로직 (Busy wait 전까지 sleep)
            while (rclcpp::ok())
            {
                struct timespec t;
                clock_gettime(CLOCK_REALTIME, &t);

                long diff_ns = (target.tv_sec - t.tv_sec) * 1000000000L + (target.tv_nsec - t.tv_nsec);
                if (diff_ns <= 2000000) break; // 2ms 전에는 spin 시작

                struct timespec sleep_ts{0, 1000000}; // 1ms sleep
                nanosleep(&sleep_ts, nullptr);
            }

            // 마지막 정밀 Spin
            while (rclcpp::ok())
            {
                struct timespec t;
                clock_gettime(CLOCK_REALTIME, &t);
                if ((t.tv_sec > target.tv_sec) || (t.tv_sec == target.tv_sec && t.tv_nsec >= target.tv_nsec))
                    break;
            }

            // GPZDA 생성 및 ROS 2 토픽 발행
            auto msg = nmea_msgs::msg::Sentence();
            msg.header.stamp = this->now();
            msg.header.frame_id = "imu_link";
            msg.sentence = make_gpzda(next_sec);

            nmea_pub_->publish(msg);
            
            // 전송 후 다음 루프까지 잠시 대기 (중복 전송 방지)
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GPZDASender>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}