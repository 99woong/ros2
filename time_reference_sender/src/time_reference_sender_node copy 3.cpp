#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <sched.h>
#include <time.h>
#include <chrono>
#include <thread>

class TimeReferenceSender : public rclcpp::Node
{
public:
    TimeReferenceSender() : Node("time_reference_sender")
    {
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.reliable(); 
        qos.durability_volatile();
        pub_ = this->create_publisher<sensor_msgs::msg::TimeReference>("ext/time", qos);

        // 정밀한 타이밍 제어를 위해 별도 스레드에서 실행
        sender_thread_ = std::thread(&TimeReferenceSender::run, this);

        RCLCPP_INFO(this->get_logger(), "TimeReference sender started (Target: Exact Second 0.000s)");
    }

    ~TimeReferenceSender()
    {
        if (sender_thread_.joinable()) {
            sender_thread_.join();
        }
    }

private:
    std::thread sender_thread_;
    rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr pub_;
    int publish_count_ = 0;

    void run()
    {
        // 실시간(RT) 우선순위 설정: 스케줄링 지연 최소화
        struct sched_param param;
        param.sched_priority = 80;
        if(sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
            RCLCPP_WARN(this->get_logger(), "Failed to set RT priority. Run with sudo or check limits.");
        }

        while (rclcpp::ok())
        {
            struct timespec now;
            clock_gettime(CLOCK_REALTIME, &now);

            // 목표 시간: 다음 정시초 (나노초 = 0)
            time_t target_sec = now.tv_sec + 1;
            struct timespec target;
            target.tv_sec = target_sec;
            target.tv_nsec = 0; 

            // 1. 근접 대기 (Sleep): 목표 2ms 전까지
            while (rclcpp::ok())
            {
                struct timespec t;
                clock_gettime(CLOCK_REALTIME, &t);
                long diff_ns = (target.tv_sec - t.tv_sec) * 1000000000L + (target.tv_nsec - t.tv_nsec);
                
                if (diff_ns <= 2000000) break; // 2ms 남으면 Busy wait로 전환

                struct timespec sleep_ts{0, 1000000}; // 1ms sleep
                nanosleep(&sleep_ts, nullptr);
            }

            // 2. 정밀 대기 (Busy Wait): 정확히 0.000s가 될 때까지 CPU 점유
            while (rclcpp::ok())
            {
                struct timespec t;
                clock_gettime(CLOCK_REALTIME, &t);
                if ((t.tv_sec > target.tv_sec) ||
                    (t.tv_sec == target.tv_sec && t.tv_nsec >= target.tv_nsec))
                    break;
            }

            // 3. 토픽 발행: 정시초 정각에 실행
            auto msg = sensor_msgs::msg::TimeReference();
            
            // IMU에 전달할 정수 초 정보 (GPS Time Update Command 대응) 
            rclcpp::Time pps_val(target.tv_sec, 0, RCL_ROS_TIME);

            msg.header.stamp = this->get_clock()->now(); // 발행 시점의 시스템 시간
            msg.time_ref = pps_val;                      // 동기화할 기준 정시초 
            msg.source = "ptp_master_pps";

            // 초기 동기화를 위해 가이드 권장대로 5회 정도 발행 후 중단 
            if(publish_count_ < 5)
            {
                pub_->publish(msg);
                publish_count_++;
                RCLCPP_INFO(this->get_logger(), "Published Exact Second: %ld.000", target.tv_sec);
            }
            else
            {
                // 동기화 완료 후에는 IMU 내부 시계가 PPS에 의해 유지되도록 발행 중단
                RCLCPP_INFO_ONCE(this->get_logger(), "Initial sync complete. IMU is now running on hardware PPS.");
            }

            // 다음 정시초 계산을 위해 충분히 휴식
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
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