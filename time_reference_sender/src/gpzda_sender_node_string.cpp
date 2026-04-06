// gpzda_sender_node.cpp

#include <rclcpp/rclcpp.hpp>

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sched.h>
#include <time.h>

#include <chrono>
#include <cstring>
#include <string>
#include <iomanip>
#include <sstream>

class GPZDASender : public rclcpp::Node
{
public:
    GPZDASender() : Node("gpzda_sender")
    {
        port_ = this->declare_parameter<std::string>("port", "/dev/ttyUSB1");
        baud_ = this->declare_parameter<int>("baudrate", 115200);
        advance_ms_ = this->declare_parameter<double>("advance_ms", 20.0);

        open_serial();

        // RT priority thread
        sender_thread_ = std::thread(&GPZDASender::run, this);

        RCLCPP_INFO(this->get_logger(),
            "GPZDA sender started (advance=%.2f ms)", advance_ms_);
    }

private:
    int fd_;
    std::string port_;
    int baud_;
    double advance_ms_;
    std::thread sender_thread_;

    // ---------------- UART ----------------
    void open_serial()
    {
        fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            perror("open");
            exit(1);
        }

        struct termios tty{};
        tcgetattr(fd_, &tty);

        cfmakeraw(&tty);
        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);

        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CRTSCTS;

        tcsetattr(fd_, TCSANOW, &tty);

        // write latency 최소화
        int flags = TIOCM_DTR;
        ioctl(fd_, TIOCMBIS, &flags);
    }

    // ---------------- GPZDA 생성 ----------------
    std::string make_gpzda(struct timespec ts)
    {
        struct tm tm{};
        gmtime_r(&ts.tv_sec, &tm);

        int ms = ts.tv_nsec / 1000000;

        char buf[128];
        snprintf(buf, sizeof(buf),
            "$GPZDA,%02d%02d%02d.%02d,%02d,%02d,%04d,00,00*",
            tm.tm_hour, tm.tm_min, tm.tm_sec,
            ms / 10,
            tm.tm_mday, tm.tm_mon + 1, tm.tm_year + 1900);

        // checksum
        uint8_t cs = 0;
        for (int i = 1; buf[i] != '*'; ++i)
            cs ^= buf[i];

        char final[128];
        snprintf(final, sizeof(final), "%s%02X\r\n", buf, cs);

        return std::string(final);
    }

    // ---------------- RT Thread ----------------
    void run()
    {
        // real-time priority
        struct sched_param param;
        param.sched_priority = 80;
        sched_setscheduler(0, SCHED_FIFO, &param);

        while (rclcpp::ok())
        {
            struct timespec now;
            clock_gettime(CLOCK_REALTIME, &now);

            // 다음 초 계산
            time_t next_sec = now.tv_sec + 1;

            // 목표 시간 = next_sec - advance
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

            // sleep until 근접
            while (true)
            {
                struct timespec t;
                clock_gettime(CLOCK_REALTIME, &t);

                long diff_ns =
                    (target.tv_sec - t.tv_sec) * 1000000000L +
                    (target.tv_nsec - t.tv_nsec);

                if (diff_ns <= 5000000) // 5ms 이하 → busy wait
                    break;

                struct timespec sleep_ts{0, diff_ns - 2000000};
                nanosleep(&sleep_ts, nullptr);
            }

            // 마지막 μs 정밀 spin
            while (true)
            {
                struct timespec t;
                clock_gettime(CLOCK_REALTIME, &t);

                if ((t.tv_sec > target.tv_sec) ||
                    (t.tv_sec == target.tv_sec && t.tv_nsec >= target.tv_nsec))
                    break;
            }

            // GPZDA 생성 (next_sec 기준!)
            struct timespec ts;
            ts.tv_sec = next_sec;
            ts.tv_nsec = 0;

            std::string zda = make_gpzda(ts);

            write(fd_, zda.c_str(), zda.size());
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GPZDASender>());
    rclcpp::shutdown();
    return 0;
}