#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <opencv2/opencv.hpp>

#include <deque>
#include <cmath>
#include <map>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <ctime>

struct PoseData {
    double x;
    double y;
    double yaw;
};

struct StatisticsData {
    PoseData current;
    PoseData mean;
    PoseData stddev;
    std::deque<PoseData> history;
};

class PoseMonitor : public rclcpp::Node {
private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub1_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub2_;
    
    StatisticsData stats1_;
    StatisticsData stats2_;
    
    std::string topic1_;
    std::string topic2_;
    int sample_size_;
    
    std::vector<std::string> available_topics_;
    int selected_topic1_idx_;
    int selected_topic2_idx_;
    
    cv::Mat display_;
    const int WINDOW_WIDTH = 1200;
    const int WINDOW_HEIGHT = 900;
    
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::ofstream csv_file_;
    std::string csv_filename_;
    cv::Rect save_button_rect_;
    bool mouse_over_button_;
    bool csv_file_created_;

    std::deque<std::string> log_history_;
    const int MAX_LOG_LINES = 8;  
    int log_count_;    
    
public:
    PoseMonitor() : Node("pose_monitor_node"), 
                    sample_size_(10), 
                    selected_topic1_idx_(0), 
                    selected_topic2_idx_(1),
                    mouse_over_button_(false),
                    csv_file_created_(false),
                    log_count_(0)  
    {
        
        topic1_ = "/vloc/pose";
        topic2_ = "/gls100/pose";
        
        // Generate CSV filename (but don't create file yet)
        generateCSVFilename();
        
        display_ = cv::Mat(WINDOW_HEIGHT, WINDOW_WIDTH, CV_8UC3, cv::Scalar(240, 240, 240));
        cv::namedWindow("Pose Monitor", cv::WINDOW_AUTOSIZE);
        
        cv::setMouseCallback("Pose Monitor", mouseCallback, this);
        
        cv::createTrackbar("Sample Size", "Pose Monitor", &sample_size_, 100);
        cv::setTrackbarMin("Sample Size", "Pose Monitor", 1);
        cv::setTrackbarPos("Sample Size", "Pose Monitor", 10);
        
        save_button_rect_ = cv::Rect(WINDOW_WIDTH - 250, 10, 230, 40);
        
        getAvailableTopics();
        subscribePoseTopics();  

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&PoseMonitor::displayCallback, this));
    }
    
    ~PoseMonitor() 
    {
        if (csv_file_.is_open()) 
        {
            csv_file_.close();
        }
    }
    
    void generateCSVFilename() 
    {
        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);
        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        
        std::stringstream ss;
        ss << "pose_log_" 
           << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S")
           << "_" << std::setfill('0') << std::setw(3) << now_ms.count()
           << ".csv";
        
        csv_filename_ = ss.str();
        RCLCPP_INFO(this->get_logger(), "CSV filename ready: %s (will be created on first save)", 
                    csv_filename_.c_str());
    }
    
    void createCSVFile() 
    {
        csv_file_.open(csv_filename_, std::ios::app);
        
        if (csv_file_.is_open()) 
        {
            csv_file_created_ = true;
            RCLCPP_INFO(this->get_logger(), "CSV file created: %s", csv_filename_.c_str());
        } 
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create CSV file: %s", csv_filename_.c_str());
        }
    }
    
    void saveToCSV() 
    {
        if (!csv_file_created_) 
        {
            createCSVFile();
        }
        
        if (!csv_file_.is_open()) 
        {
            RCLCPP_WARN(this->get_logger(), "CSV file is not open!");
            return;
        }

        log_count_++;        
        
        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);
        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        
        std::stringstream timestamp;
        timestamp << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S")
                 << "." << std::setfill('0') << std::setw(3) << now_ms.count();

        csv_file_<< std::fixed << std::setprecision(6)
                 << stats1_.mean.x << ","
                 << stats1_.mean.y << ","
                 << stats1_.mean.yaw << ","
                 << stats2_.mean.x << ","
                 << stats2_.mean.y << ","
                 << stats2_.mean.yaw << ","
                 << stats1_.stddev.x << ","
                 << stats1_.stddev.y << ","
                 << stats1_.stddev.yaw << ","
                 << stats2_.stddev.x << ","
                 << stats2_.stddev.y << ","
                 << stats2_.stddev.yaw << ","
                 << timestamp.str() << "\n";
        
        csv_file_.flush();

        std::stringstream log_entry;
        log_entry << "#" << log_count_ << " | " << timestamp.str() 
                 << " | V:(" << std::fixed << std::setprecision(2)
                 << stats1_.mean.x << "," << stats1_.mean.y << "," << stats1_.mean.yaw << ","
                 << stats1_.stddev.x << "," << stats1_.stddev.y << "," << stats1_.stddev.yaw 
                 << ") G:(" 
                 << stats2_.current.x << "," << stats2_.current.y << "," << stats2_.current.yaw << ","
                 << stats2_.stddev.x << "," << stats2_.stddev.y << "," << stats2_.stddev.yaw << ")";
        
        log_history_.push_back(log_entry.str());
        
        while (log_history_.size() > MAX_LOG_LINES) 
        {
            log_history_.pop_front();
        }        

        RCLCPP_INFO(this->get_logger(), "Data saved to CSV at %s", timestamp.str().c_str());
    }
    
    static void mouseCallback(int event, int x, int y, int flags, void* userdata) 
    {
        PoseMonitor* monitor = static_cast<PoseMonitor*>(userdata);
        
        if (event == cv::EVENT_LBUTTONDOWN) 
        {
            if (monitor->save_button_rect_.contains(cv::Point(x, y))) 
            {
                monitor->saveToCSV();
            }
        } 
        else if (event == cv::EVENT_MOUSEMOVE) 
        {
            monitor->mouse_over_button_ = monitor->save_button_rect_.contains(cv::Point(x, y));
        }
    }
    
    void getAvailableTopics() 
    {
        auto topic_names_and_types = this->get_topic_names_and_types();
        
        available_topics_.clear();
        for (const auto& topic : topic_names_and_types) 
        {
            for (const auto& type : topic.second) 
            {
                if (type == "geometry_msgs/msg/PoseStamped" || 
                    type == "geometry_msgs/msg/PoseWithCovarianceStamped") 
                    {
                    available_topics_.push_back(topic.first);
                    break;
                }
            }
        }
        
        if (available_topics_.empty()) 
        {
            available_topics_.push_back("/vloc/pose");
            available_topics_.push_back("/gls100/pose");
        }
        
        // Set default topics if they exist
        auto it1 = std::find(available_topics_.begin(), available_topics_.end(), "/vloc/pose");
        if (it1 != available_topics_.end()) 
        {
            selected_topic1_idx_ = std::distance(available_topics_.begin(), it1);
            topic1_ = "/vloc/pose";
        }
        
        auto it2 = std::find(available_topics_.begin(), available_topics_.end(), "/gls100/pose");
        if (it2 != available_topics_.end()) 
        {
            selected_topic2_idx_ = std::distance(available_topics_.begin(), it2);
            topic2_ = "/gls100/pose";
        }
    }
    
    void subscribePoseTopics() 
    {
        sub1_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            topic1_, 10,
            std::bind(&PoseMonitor::poseCallback1, this, std::placeholders::_1));
            
        sub2_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            topic2_, 10,
            std::bind(&PoseMonitor::poseCallback2, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Subscribed to: %s and %s", topic1_.c_str(), topic2_.c_str());
    }
    
    void poseCallback1(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
    {
        updateStatistics(stats1_, msg);
    }
    
    void poseCallback2(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
    {
        updateStatistics(stats2_, msg);
    }
    

    void updateStatistics(StatisticsData& stats, const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
    {
        PoseData pose;
        pose.x = msg->pose.position.x;
        pose.y = msg->pose.position.y;
        
        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w
        );
        
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        pose.yaw = yaw * 180.0 / M_PI; // Convert to degrees
        
        stats.current = pose;
        stats.history.push_back(pose);
        
        while (static_cast<int>(stats.history.size()) > sample_size_) 
        {
            stats.history.pop_front();
        }
        
        calculateStatistics(stats);
    }
    
    void calculateStatistics(StatisticsData& stats) 
    {
        if (stats.history.empty()) return;
        
        double sum_x = 0, sum_y = 0, sum_yaw = 0;
        for (const auto& pose : stats.history) {
            sum_x += pose.x;
            sum_y += pose.y;
            sum_yaw += pose.yaw;
        }
        
        int n = stats.history.size();
        stats.mean.x = sum_x / n;
        stats.mean.y = sum_y / n;
        stats.mean.yaw = sum_yaw / n;
        
        double var_x = 0, var_y = 0, var_yaw = 0;
        for (const auto& pose : stats.history) {
            var_x += std::pow(pose.x - stats.mean.x, 2);
            var_y += std::pow(pose.y - stats.mean.y, 2);
            var_yaw += std::pow(pose.yaw - stats.mean.yaw, 2);
        }
        
        stats.stddev.x = std::sqrt(var_x / n);
        stats.stddev.y = std::sqrt(var_y / n);
        stats.stddev.yaw = std::sqrt(var_yaw / n);
    }
    
    void drawText(const std::string& text, int x, int y, double scale = 0.6, 
                  cv::Scalar color = cv::Scalar(0, 0, 0), int thickness = 1) 
    {
        cv::putText(display_, text, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 
                   scale, color, thickness);
    }
    
    void drawButton() 
    {
        cv::Scalar button_color = mouse_over_button_ ? 
            cv::Scalar(100, 180, 100) : cv::Scalar(80, 150, 80);
        cv::Scalar border_color = cv::Scalar(50, 100, 50);
        
        // Draw button background
        cv::rectangle(display_, save_button_rect_, button_color, cv::FILLED);
        cv::rectangle(display_, save_button_rect_, border_color, 2);
        
        // Draw button text
        std::string button_text = "Save to CSV";
        int baseline = 0;
        cv::Size text_size = cv::getTextSize(button_text, cv::FONT_HERSHEY_SIMPLEX, 
                                             0.7, 2, &baseline);
        
        cv::Point text_pos(
            save_button_rect_.x + (save_button_rect_.width - text_size.width) / 2,
            save_button_rect_.y + (save_button_rect_.height + text_size.height) / 2
        );
        
        drawText(button_text, text_pos.x, text_pos.y, 0.7, cv::Scalar(255, 255, 255), 2);
    }
    
    void drawPoseData(const std::string& title, const StatisticsData& stats, 
                     int x_offset, int y_offset, cv::Scalar color) 
    {
        // Title
        drawText(title, x_offset, y_offset, 0.8, color, 2);
        
        int y = y_offset + 40;
        int line_height = 30;
        
        // Current Data
        drawText("[ Current Data ]", x_offset, y, 0.7, cv::Scalar(0, 0, 0), 2);
        y += line_height;
        drawText(cv::format("X: %.4f m", stats.current.x), x_offset + 10, y);
        y += line_height;
        drawText(cv::format("Y: %.4f m", stats.current.y), x_offset + 10, y);
        y += line_height;
        drawText(cv::format("Yaw: %.2f deg", stats.current.yaw), x_offset + 10, y);
        y += line_height + 10;
        
        // Mean
        drawText("[ Mean ]", x_offset, y, 0.7, cv::Scalar(0, 100, 0), 2);
        y += line_height;
        drawText(cv::format("X: %.4f m", stats.mean.x), x_offset + 10, y);
        y += line_height;
        drawText(cv::format("Y: %.4f m", stats.mean.y), x_offset + 10, y);
        y += line_height;
        drawText(cv::format("Yaw: %.2f deg", stats.mean.yaw), x_offset + 10, y);
        y += line_height + 10;
        
        // Standard Deviation
        drawText("[ Std Dev ]", x_offset, y, 0.7, cv::Scalar(0, 0, 200), 2);
        y += line_height;
        drawText(cv::format("X: %.6f m", stats.stddev.x), x_offset + 10, y);
        y += line_height;
        drawText(cv::format("Y: %.6f m", stats.stddev.y), x_offset + 10, y);
        y += line_height;
        drawText(cv::format("Yaw: %.4f deg", stats.stddev.yaw), x_offset + 10, y);
        y += line_height;
        
        drawText(cv::format("Samples: %d / %d", (int)stats.history.size(), sample_size_), 
                x_offset + 10, y, 0.5, cv::Scalar(100, 100, 100));
    }
    
    void drawTopicList(int x_offset, int y_offset) 
    {
        drawText("[ Available Topics ]", x_offset, y_offset, 0.7, cv::Scalar(0, 0, 0), 2);
        int y = y_offset + 30;
        
        for (size_t i = 0; i < available_topics_.size() && i < 15; ++i) 
        {
            cv::Scalar color = cv::Scalar(100, 100, 100);
            std::string marker = "  ";
            
            if (static_cast<int>(i) == selected_topic1_idx_) 
            {
                color = cv::Scalar(200, 0, 0);
                marker = "1>";
            } 
            else if (static_cast<int>(i) == selected_topic2_idx_) 
            {
                color = cv::Scalar(0, 150, 0);
                marker = "2>";
            }
            
            drawText(marker + available_topics_[i], x_offset, y, 0.45, color);
            y += 22;
        }
        
        if (available_topics_.size() > 15) 
        {
            drawText(cv::format("... and %d more", (int)available_topics_.size() - 15), 
                    x_offset, y, 0.4, cv::Scalar(100, 100, 100));
        }
    }

  
    // *** 로그 히스토리 에디트박스 그리기 ***
    void drawLogHistory(int x_offset, int y_offset, int width, int height) 
    {
        // 에디트박스 배경
        cv::Rect log_box(x_offset, y_offset, width, height);
        cv::rectangle(display_, log_box, cv::Scalar(255, 255, 255), cv::FILLED);
        cv::rectangle(display_, log_box, cv::Scalar(100, 100, 100), 2);
        
        // 타이틀
        drawText(cv::format("[Save History - Total: %d]", log_count_), 
                x_offset + 0, y_offset - 8, 0.6, cv::Scalar(0, 0, 0), 2);
        
        // 로그 내용 표시
        int y = y_offset + 25;
        int line_spacing = 20;
        
        if (log_history_.empty()) 
        {
            drawText("No data saved yet. Click 'Save to CSV' button.", 
                    x_offset + 10, y, 0.45, cv::Scalar(150, 150, 150));
        } 
        else 
        {
            for (const auto& log : log_history_) 
            {
                drawText(log, x_offset + 10, y, 0.4, cv::Scalar(0, 0, 0));
                y += line_spacing;
            }
        }
    }

    void displayCallback() 
    {
        sample_size_ = cv::getTrackbarPos("Sample Size", "Pose Monitor");
        if (sample_size_ < 1) sample_size_ = 1;
        
        display_ = cv::Mat(WINDOW_HEIGHT, WINDOW_WIDTH, CV_8UC3, cv::Scalar(240, 240, 240));
        
        // Title
        drawText("ROS2 Pose Monitor", 20, 40, 1.2, cv::Scalar(0, 0, 0), 2);
        
        // Draw save button
        drawButton();
        
        // CSV filename display
        std::string csv_status = csv_file_created_ ? 
            ("Logging to: " + csv_filename_) : 
            ("Ready: " + csv_filename_ + " (click Save to start)");
        drawText(csv_status, 20, WINDOW_HEIGHT - 40, 0.45, 
                csv_file_created_ ? cv::Scalar(0, 100, 0) : cv::Scalar(100, 100, 0));
               
        // Left panel - Topic 1
        drawPoseData(topic1_, stats1_, 30, 90, cv::Scalar(200, 0, 0));
        
        // Right panel - Topic 2
        drawPoseData(topic2_, stats2_, WINDOW_WIDTH/2 + 30, 90, cv::Scalar(0, 150, 0));
        
        // Bottom section - Topic list
        int topic_list_y = 500;

        // *** 로그 히스토리 에디트박스 (우측 하단) ***
        int log_box_x = 10;
        int log_box_y = topic_list_y + 140;
        int log_box_width = WINDOW_WIDTH - log_box_x - 30;
        int log_box_height = 200;
        drawLogHistory(log_box_x, log_box_y, log_box_width, log_box_height);        

        // Instructions
        drawText("Press 'r' to refresh topics | 'q' to quit", 20, WINDOW_HEIGHT - 20, 
                0.5, cv::Scalar(100, 100, 100));
        
        cv::imshow("Pose Monitor", display_);
        
        int key = cv::waitKey(1);
        if (key == 'q' || key == 27) 
        { // 'q' or ESC
            rclcpp::shutdown();
        } 
        else if (key == 'r') 
        { // Refresh topics
            getAvailableTopics();
            RCLCPP_INFO(this->get_logger(), "Topics refreshed");
        }
    }
};

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<PoseMonitor>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
