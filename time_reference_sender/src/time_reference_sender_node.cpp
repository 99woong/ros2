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
        advance_ms_ = this->declare_parameter<double>("advance_ms", 20.0);

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
        // RT priority
        struct sched_param param;
        param.sched_priority = 80;
        if(sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
            RCLCPP_WARN(this->get_logger(), "Failed to set RT priority");
        }

        while (rclcpp::ok())
        {
            struct timespec now;
            clock_gettime(CLOCK_REALTIME, &now);

            time_t next_sec = now.tv_sec + 1;

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

            // sleep until near target
            while (rclcpp::ok())
            {
                struct timespec t;
                clock_gettime(CLOCK_REALTIME, &t);

                long diff_ns = (target.tv_sec - t.tv_sec) * 1000000000L + (target.tv_nsec - t.tv_nsec);
                if (diff_ns <= 2000000) break;

                struct timespec sleep_ts{0, 1000000};
                nanosleep(&sleep_ts, nullptr);
            }

            // busy wait
            while (rclcpp::ok())
            {
                struct timespec t;
                clock_gettime(CLOCK_REALTIME, &t);
                if ((t.tv_sec > target.tv_sec) ||
                    (t.tv_sec == target.tv_sec && t.tv_nsec >= target.tv_nsec))
                    break;
            }

            // publish TimeReference
            auto msg = sensor_msgs::msg::TimeReference();
            auto now_ros = this->get_clock()->now();

            msg.header.stamp = now_ros;
            msg.time_ref = now_ros;  // 핵심!
            msg.source = "ptp_time";

            pub_->publish(msg);

            RCLCPP_INFO(this->get_logger(),
                "Published TimeReference: %.6f", now_ros.seconds());

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