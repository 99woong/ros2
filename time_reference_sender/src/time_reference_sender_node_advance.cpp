#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/time_reference.hpp>

#include <sched.h>
#include <time.h>
#include <thread>

class TimeReferenceSender : public rclcpp::Node
{
public:
    TimeReferenceSender() : Node("time_reference_sender")
    {
        // ⭐ offset (ms 단위, 튜닝 포인트)
        advance_ms_ = this->declare_parameter<double>("advance_ms", 5.0);

        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.reliable();
        qos.durability_volatile();

        pub_ = this->create_publisher<sensor_msgs::msg::TimeReference>("ext/time", qos);

        sender_thread_ = std::thread(&TimeReferenceSender::run, this);

        RCLCPP_INFO(this->get_logger(),
            "TimeReference sender started (advance=%.2f ms)", advance_ms_);
    }

    ~TimeReferenceSender()
    {
        if (sender_thread_.joinable()) {
            sender_thread_.join();
        }
    }

private:
    double advance_ms_;
    std::thread sender_thread_;
    rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr pub_;

    void run()
    {
        // RT priority 설정
        struct sched_param param;
        param.sched_priority = 80;
        if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
            RCLCPP_WARN(this->get_logger(), "Failed to set RT priority");
        }

        while (rclcpp::ok())
        {
            // 현재 시간
            struct timespec now;
            clock_gettime(CLOCK_REALTIME, &now);

            // 다음 정시초
            time_t next_sec = now.tv_sec + 1;

            // ⭐ offset 계산
            long advance_ns = (long)(advance_ms_ * 1e6);

            // ⭐ publish 시점 = 정시초 - offset
            struct timespec target;
            target.tv_sec = next_sec;
            target.tv_nsec = -advance_ns;

            if (target.tv_nsec < 0) {
                target.tv_sec -= 1;
                target.tv_nsec += 1000000000;
            }

            // sleep + busy wait
            while (rclcpp::ok())
            {
                struct timespec t;
                clock_gettime(CLOCK_REALTIME, &t);

                long diff_ns = (target.tv_sec - t.tv_sec) * 1000000000L
                             + (target.tv_nsec - t.tv_nsec);

                if (diff_ns <= 2000000) break; // 2ms 이내면 busy wait

                struct timespec sleep_ts{0, 1000000}; // 1ms sleep
                nanosleep(&sleep_ts, nullptr);
            }

            // busy wait (정밀 맞춤)
            while (rclcpp::ok())
            {
                struct timespec t;
                clock_gettime(CLOCK_REALTIME, &t);

                if ((t.tv_sec > target.tv_sec) ||
                    (t.tv_sec == target.tv_sec && t.tv_nsec >= target.tv_nsec))
                    break;
            }

            // "정시초"를 값으로 사용 (중요)
            rclcpp::Time exact_time(next_sec, 0);

            auto msg = sensor_msgs::msg::TimeReference();

            // 실제 publish 시각 (디버깅용)
            auto now_ros = this->get_clock()->now();

            msg.header.stamp = now_ros;     // 실제 전송 시각
            msg.time_ref = exact_time;      // 기준 시간 (정시초 유지)
            msg.source = "ptp_time";

            pub_->publish(msg);

            RCLCPP_INFO(this->get_logger(),
                "Publish @ %.6f | time_ref = %.6f (advance=%.2f ms)",
                now_ros.seconds(),
                exact_time.seconds(),
                advance_ms_);
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TimeReferenceSender>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}