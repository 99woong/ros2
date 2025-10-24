#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/u_int16.hpp> 

#include <deque>
#include <cmath>
#include <map>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <ctime>
#include <cstdlib> // for std::exit
#include <filesystem> 
namespace fs = std::filesystem; 

struct PoseData {
    double x;
    double y;
    double yaw;
};

struct StatisticsData {
    PoseData current;
    PoseData mean;
    PoseData stddev;
    PoseData max_stddev;
    std::deque<PoseData> history;

    double rate_hz = 0.0;        // 현재 계산된 패킷 레이트 (Hz)    
};

class PoseMonitor : public rclcpp::Node {
private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub1_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub2_;
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr status_sub_; 
    
    rclcpp::TimerBase::SharedPtr rate_timer_;
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
    
    // 각 토픽별 1초간 메시지 수신 카운터
    std::atomic<int> msg_count1_ = 0; 
    std::atomic<int> msg_count2_ = 0;

    StatisticsData stats1_;
    StatisticsData stats2_;
    
    std::string topic1_;
    std::string topic2_;
    uint16_t current_status_ = 0; 

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
    
    // 자동 저장 기능 변수
    bool auto_save_enabled_ = false;  
    bool is_stable_ = false;          
    bool was_stable_ = false;         
    int stable_count = 0;          

    // 안정성 카운터
    int stable_count_ = 0; // 안정 상태가 지속된 프레임 수
    const int STABILITY_FRAMES_THRESHOLD = 30; // 약 1초 (30fps 기준)

    //이전 프레임의 포즈 값 저장 (Topic 1과 Topic 2 각각 필요)
    PoseData last_pose1_; 
    PoseData last_pose2_; 
    
    //1초 카운트 동안 데이터 변화가 있었는지 추적
    bool data_changed_during_stable_count_ = false; 

    // 변화를 감지할 미세 임계값 (예: 1 마이크로미터 및 1 마이크로도)
    const double MIN_CHANGE_M = 1e-6;   
    const double MIN_CHANGE_YAW = 1e-6;

    // 추가: 저장 이벤트가 발생했는지 추적하는 플래그
    bool save_fired_ = false;    
    
    // 안정성 기준 정의
    // const double STDD_M_THRESHOLD = 0.005;
    // const double STDD_YAW_THRESHOLD = 0.1;

    double std_m_threshold_;
    double std_yaw_threshold_;

    int grid_x_offset; 
    int grid_y_offset; 

    // const int GRID_SIZE = 250;     // 그리드 UI의 크기 (픽셀)
    // const int GRID_SCALE = 100;    // 1m당 픽셀 수 (예: 100 픽셀/m)
    
    const int GRID_SIZE = 250;       // 그리드 UI의 크기는 유지 (픽셀)
    
    // 30mm를 표시하기 위해 스케일 조정 (0.03m을 100~125픽셀로 표시)
    // 0.03m -> 125 픽셀로 설정하면 1m당 약 4166 픽셀이 됨 (4166 pixels/m)
    const double GRID_M_RANGE = 0.06;  // 표시할 X, Y 범위 (m). 즉, [-0.03m, 0.03m]
    const double GRID_SCALE = GRID_SIZE / (GRID_M_RANGE * 2.0); // 250 / 0.06 = 4166.67 pixels/m
    
    int map_rotation_deg_ = 0; // 0, 90, 180, 270
    cv::Rect rotate_button_rect_;
    
    const std::string LOG_FOLDER = "/home/zenix/pose_logs/"; 
    
    void rateTimerCallback();
    void updateRate(StatisticsData& stats, int count);

    // PRIVATE 멤버 함수 선언 (thickness 기본 인자 추가)
    void updateStatistics(StatisticsData& stats, const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void calculateStatistics(StatisticsData& stats);
    void checkStability(); 

    // 두께(thickness)의 기본값 1을 명시적으로 추가
    void drawText(const std::string& text, int x, int y, double scale = 0.6, 
                  cv::Scalar color = cv::Scalar(0, 0, 0), int thickness = 1);
    
    void drawButton();
    void drawPoseData(const std::string& title, const StatisticsData& stats, 
                      int x_offset, int y_offset, cv::Scalar color, double std_m, double std_deg);
    void drawTopicList(int x_offset, int y_offset);
    void drawLogHistory(int x_offset, int y_offset, int width, int height); 
    void drawGridUI(int x_offset, int y_offset);
    
public:
    // PUBLIC 멤버 함수 선언
    PoseMonitor();
    ~PoseMonitor();
    
    void generateCSVFilename();
    void createCSVFile();
    void saveToCSV();
    void closeAndMoveCSV(); 
    void prepareNewCSV(); 

    void getAvailableTopics();
    void subscribePoseTopics();
    void subscribeStatusTopic();

    void poseCallback1(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void poseCallback2(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void statusCallback(const std_msgs::msg::UInt16::SharedPtr msg);
    void displayCallback();
    void drawRotateButton(int x_offset, int y_offset);
    
    static void mouseCallback(int event, int x, int y, int flags, void* userdata);
};

// ====================================================================
// PoseMonitor 구현
// ====================================================================

PoseMonitor::PoseMonitor() : Node("pose_monitor_node"), 
                    sample_size_(10), 
                    selected_topic1_idx_(0), 
                    selected_topic2_idx_(1),
                    mouse_over_button_(false),
                    csv_file_created_(false),
                    log_count_(0),
                    stable_count_(0),
                    save_fired_(false) // save_fired_ 추가  
{
    this->declare_parameter<double>("stability_threshold.std_m", 0.005);
    this->declare_parameter<double>("stability_threshold.std_yaw", 0.1);

    // ----------------------------------------------------
    // 수정: 매개변수 값 읽기
    // ----------------------------------------------------
    this->get_parameter("stability_threshold.std_m", std_m_threshold_);
    this->get_parameter("stability_threshold.std_yaw", std_yaw_threshold_);

    std::cout << " [std_m_threshold] : " << std_m_threshold_<< std::endl;
    std::cout << " [std_yaw_threshold] : " << std_yaw_threshold_<< std::endl;

    RCLCPP_INFO(this->get_logger(), "Loaded STDD_M_THRESHOLD: %.4f m", std_m_threshold_);
    RCLCPP_INFO(this->get_logger(), "Loaded STDD_YAW_THRESHOLD: %.4f deg", std_yaw_threshold_);    
    topic1_ = "/vslam/pose";
    topic2_ = "/gls100/pose";
    
    generateCSVFilename();
    
    rotate_button_rect_ = cv::Rect(WINDOW_WIDTH / 2 + 300, 
                                   100 + GRID_SIZE + 80, // y_offset (400) + GRID_SIZE (250) + 여백 (40)
                                   GRID_SIZE, 30);      // 250x30 픽셀 크기

    // 🚀 WINDOW_WIDTH, WINDOW_HEIGHT 순서 변경 (OpenCV Mat 생성 시 행, 열 순서)
    display_ = cv::Mat(WINDOW_HEIGHT, WINDOW_WIDTH, CV_8UC3, cv::Scalar(240, 240, 240)); 
    cv::namedWindow("Pose Monitor", cv::WINDOW_AUTOSIZE);
    
    cv::setMouseCallback("Pose Monitor", mouseCallback, this);
    
    cv::createTrackbar("Sample Size", "Pose Monitor", &sample_size_, 100);
    cv::setTrackbarMin("Sample Size", "Pose Monitor", 1);
    cv::setTrackbarPos("Sample Size", "Pose Monitor", 10);
    
    save_button_rect_ = cv::Rect(WINDOW_WIDTH - 250, 10, 230, 40);
    
    timer_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
        
    rate_timer_ = this->create_wall_timer(
        std::chrono::seconds(1), 
        std::bind(&PoseMonitor::rateTimerCallback, this), 
        timer_callback_group_);

    getAvailableTopics();
    subscribePoseTopics();  
    subscribeStatusTopic(); 

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33),
        std::bind(&PoseMonitor::displayCallback, this));
}

PoseMonitor::~PoseMonitor() 
{
    if (csv_file_.is_open()) 
    {
        csv_file_.close();
    }
}

void PoseMonitor::drawRotateButton(int x_offset, int y_offset)
{
    std::string button_text = cv::format("Rotate Map (%d deg)", map_rotation_deg_);
    
    // 버튼 색상 설정
    cv::Scalar button_color = mouse_over_button_ ? 
        cv::Scalar(100, 100, 200) : cv::Scalar(150, 150, 200); 
    
    cv::rectangle(display_, rotate_button_rect_, button_color, cv::FILLED);
    cv::rectangle(display_, rotate_button_rect_, cv::Scalar(50, 50, 50), 2);
    
    int baseline = 0;
    cv::Size text_size = cv::getTextSize(button_text, cv::FONT_HERSHEY_SIMPLEX, 
                                         0.6, 1, &baseline);
    
    cv::Point text_pos(
        rotate_button_rect_.x + (rotate_button_rect_.width - text_size.width) / 2,
        rotate_button_rect_.y + (rotate_button_rect_.height + text_size.height) / 2
    );
    
    drawText(button_text, text_pos.x, text_pos.y, 0.6, cv::Scalar(255, 255, 255), 1);
}

void PoseMonitor::generateCSVFilename() 
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

void PoseMonitor::createCSVFile() 
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

void PoseMonitor::closeAndMoveCSV() 
{
    if (!csv_file_created_) {
        RCLCPP_WARN(this->get_logger(), "No CSV file was created to save.");
        return;
    }
    
    if (csv_file_.is_open()) {
        csv_file_.close();
    }

    fs::path current_path = csv_filename_;
    fs::path target_folder = LOG_FOLDER;
    
    try {
        if (!fs::exists(target_folder)) {
            fs::create_directories(target_folder);
            RCLCPP_INFO(this->get_logger(), "Created log directory: %s", target_folder.c_str());
        }
        
        fs::path target_path = target_folder / current_path.filename();
        
        fs::rename(current_path, target_path);
        
        RCLCPP_INFO(this->get_logger(), "Successfully saved and moved log to: %s", target_path.c_str());
        
        csv_file_created_ = false;
        csv_filename_ = "--- LOG SAVED ---"; 
        
    } catch (const fs::filesystem_error& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to move CSV file: %s", e.what());
    }
}

void PoseMonitor::prepareNewCSV() 
{
    if (csv_file_created_ && csv_file_.is_open()) {
        csv_file_.close();
        RCLCPP_WARN(this->get_logger(), "Discarded current log file: %s", csv_filename_.c_str());
    }
    
    generateCSVFilename(); 
    csv_file_created_ = false;
    log_history_.clear();
    log_count_ = 0;
    RCLCPP_INFO(this->get_logger(), "Ready for new log file recording.");
}


void PoseMonitor::saveToCSV() 
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

    if (log_count_ == 0) {
         csv_file_ << "vloc_x,vloc_y,vloc_yaw,gls_x,gls_y,gls_yaw,vloc_stddev_x,vloc_stddev_y,vloc_stddev_yaw,gls_stddev_x,gls_stddev_y,gls_stddev_yaw,timestamp,status\n";
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
             << timestamp.str() << ","
             << current_status_ << "\n";
    
    csv_file_.flush();

    std::stringstream log_entry;
    log_entry << "#" << log_count_ << " | " << timestamp.str() 
             << " | S:" << current_status_ 
             << " | V:(" << std::fixed << std::setprecision(2)
             << stats1_.mean.x << "," << stats1_.mean.y << "," << stats1_.mean.yaw << ")"
             << " G:(" 
             << stats2_.mean.x << "," << stats2_.mean.y << "," << stats2_.mean.yaw << ")";
    
    log_history_.push_back(log_entry.str());
    
    while (log_history_.size() > MAX_LOG_LINES) 
    {
        log_history_.pop_front();
    }        

    RCLCPP_INFO(this->get_logger(), "Data saved to CSV at %s, Status: %u (Total: %d)", 
                timestamp.str().c_str(), current_status_, log_count_);
}

void PoseMonitor::checkStability() 
{
    // Topic 1 안정성 검사
    // bool stable1 = (stats1_.stddev.x < STDD_M_THRESHOLD &&
    //                 stats1_.stddev.y < STDD_M_THRESHOLD &&
    //                 stats1_.stddev.yaw < STDD_YAW_THRESHOLD);

    // // Topic 2 안정성 검사
    // bool stable2 = (stats2_.stddev.x < STDD_M_THRESHOLD &&
    //                 stats2_.stddev.y < STDD_M_THRESHOLD &&
    //                 stats2_.stddev.yaw < STDD_YAW_THRESHOLD);
    bool stable1 = (stats1_.stddev.x < std_m_threshold_ &&
                    stats1_.stddev.y < std_m_threshold_ &&
                    stats1_.stddev.yaw < std_yaw_threshold_);

    // Topic 2 안정성 검사
    bool stable2 = (stats2_.stddev.x < std_m_threshold_ &&
                    stats2_.stddev.y < std_m_threshold_ &&
                    stats2_.stddev.yaw < std_yaw_threshold_);
    
    // std::cout << "stable1 : " << stable1 << " " << stable2 <<std::endl;

    // 두 토픽 모두 안정적일 때만 전체 시스템 안정적
    is_stable_ = stable1 && stable2;
}

void PoseMonitor::mouseCallback(int event, int x, int y, int flags, void* userdata) 
{
    PoseMonitor* monitor = static_cast<PoseMonitor*>(userdata);
    
    if (event == cv::EVENT_LBUTTONDOWN) 
    {
        if (monitor->save_button_rect_.contains(cv::Point(x, y))) 
        {
            // 버튼 클릭 시 자동 저장 상태 토글
            monitor->auto_save_enabled_ = !monitor->auto_save_enabled_;
            RCLCPP_INFO(monitor->get_logger(), "AutoSave toggled: %s", 
                        monitor->auto_save_enabled_ ? "ON" : "OFF");
        }
        else if (monitor->rotate_button_rect_.contains(cv::Point(x, y))) 
        {
            monitor->map_rotation_deg_ = (monitor->map_rotation_deg_ + 90) % 360;
            RCLCPP_INFO(monitor->get_logger(), "Map Rotated to: %d degrees", monitor->map_rotation_deg_);
        }    
    } 
    else if (event == cv::EVENT_MOUSEMOVE) 
    {
        monitor->mouse_over_button_ = monitor->save_button_rect_.contains(cv::Point(x, y)) ||
                                       monitor->rotate_button_rect_.contains(cv::Point(x, y));
    }
    // flags는 사용되지 않음
    (void)flags; 
}

void PoseMonitor::getAvailableTopics() 
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
        available_topics_.push_back("/vslam/pose");
        available_topics_.push_back("/gls100/pose");
    }
    
    auto it1 = std::find(available_topics_.begin(), available_topics_.end(), "/vslam/pose");
    if (it1 != available_topics_.end()) 
    {
        selected_topic1_idx_ = std::distance(available_topics_.begin(), it1);
        topic1_ = "/vslam/pose";
    }
    
    auto it2 = std::find(available_topics_.begin(), available_topics_.end(), "/gls100/pose");
    if (it2 != available_topics_.end()) 
    {
        selected_topic2_idx_ = std::distance(available_topics_.begin(), it2);
        topic2_ = "/gls100/pose";
    }
}

void PoseMonitor::subscribePoseTopics() 
{
    sub1_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        topic1_, 10,
        std::bind(&PoseMonitor::poseCallback1, this, std::placeholders::_1));
        
    sub2_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        topic2_, 10,
        std::bind(&PoseMonitor::poseCallback2, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribed to: %s and %s", topic1_.c_str(), topic2_.c_str());
}

void PoseMonitor::subscribeStatusTopic() 
{
    status_sub_ = this->create_subscription<std_msgs::msg::UInt16>(
        "gls100/status", 10,
        std::bind(&PoseMonitor::statusCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribed to status topic: gls100/status");
}

void PoseMonitor::rateTimerCallback()
{
    // Topic 1 레이트 업데이트
    updateRate(stats1_, msg_count1_.exchange(0)); // 현재 카운트를 읽고 0으로 리셋
    
    // Topic 2 레이트 업데이트
    updateRate(stats2_, msg_count2_.exchange(0)); // 현재 카운트를 읽고 0으로 리셋
}

// StatisticsData에 레이트 값을 적용하는 함수
void PoseMonitor::updateRate(StatisticsData& stats, int count)
{
    // 타이머 주기가 정확히 1초이므로, 카운트 수 = 레이트 (Hz)
    stats.rate_hz = (double)count;
}

void PoseMonitor::poseCallback1(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
{
    updateStatistics(stats1_, msg);
    msg_count1_++;
}

void PoseMonitor::poseCallback2(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
{
    updateStatistics(stats2_, msg);
    msg_count2_++;
}

void PoseMonitor::statusCallback(const std_msgs::msg::UInt16::SharedPtr msg)
{
    current_status_ = msg->data;
}


void PoseMonitor::updateStatistics(StatisticsData& stats, const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
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
    pose.yaw = yaw * 180.0 / M_PI; 
    
    stats.current = pose;
    stats.history.push_back(pose);
    
    while (static_cast<int>(stats.history.size()) > sample_size_) 
    {
        stats.history.pop_front();
    }
    
    calculateStatistics(stats);

    stats.max_stddev.x = std::max(stats.max_stddev.x, stats.stddev.x);
    stats.max_stddev.y = std::max(stats.max_stddev.y, stats.stddev.y);
    stats.max_stddev.yaw = std::max(stats.max_stddev.yaw, stats.stddev.yaw);
    
    checkStability();
}

void PoseMonitor::calculateStatistics(StatisticsData& stats) 
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

// 🚀 drawText 정의: thickness 인자를 제거하면 안 됨.
void PoseMonitor::drawText(const std::string& text, int x, int y, double scale, 
              cv::Scalar color, int thickness) 
{
    cv::putText(display_, text, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 
               scale, color, thickness);
}

void PoseMonitor::drawButton() 
{
    cv::Scalar button_color;
    std::string button_text;
    
    if (auto_save_enabled_) {
        button_color = mouse_over_button_ ? 
            cv::Scalar(0, 200, 0) : cv::Scalar(0, 150, 0); 
        button_text = "AutoSave: ON";
    } else {
        button_color = mouse_over_button_ ? 
            cv::Scalar(180, 180, 180) : cv::Scalar(150, 150, 150); 
        button_text = "AutoSave: OFF";
    }
    
    cv::Scalar border_color = cv::Scalar(50, 100, 50);
    
    cv::rectangle(display_, save_button_rect_, button_color, cv::FILLED);
    cv::rectangle(display_, save_button_rect_, border_color, 2);
    
    int baseline = 0;
    cv::Size text_size = cv::getTextSize(button_text, cv::FONT_HERSHEY_SIMPLEX, 
                                         0.7, 2, &baseline);
    
    cv::Point text_pos(
        save_button_rect_.x + (save_button_rect_.width - text_size.width) / 2,
        save_button_rect_.y + (save_button_rect_.height + text_size.height) / 2
    );
    
    drawText(button_text, text_pos.x, text_pos.y, 0.7, cv::Scalar(255, 255, 255), 2);
    
    cv::Scalar status_color = is_stable_ ? cv::Scalar(0, 150, 0) : cv::Scalar(0, 0, 200);
    std::string status_text = is_stable_ ? "SYSTEM STABLE" : "SYSTEM MOVING";
    drawText(status_text, save_button_rect_.x + 10, save_button_rect_.y + 15, 0.5, status_color, 1);
}

void PoseMonitor::drawPoseData(const std::string& title, const StatisticsData& stats, 
                 int x_offset, int y_offset, cv::Scalar color, double std_m, double std_deg) 
{
    drawText(title, x_offset, y_offset, 0.8, color, 2);
    
    int y = y_offset + 40;
    int line_height = 30;
    
    // Current Data
    drawText("[ Current Data ]", x_offset, y, 0.7, cv::Scalar(0, 0, 0), 2);
    y += line_height;
    // 🚀 drawText 인자 수정 (thickness = 1 추가)
    drawText(cv::format("X: %.4f m", stats.current.x), x_offset + 10, y, 0.6, cv::Scalar(0, 0, 0), 1); 
    y += line_height;
    drawText(cv::format("Y: %.4f m", stats.current.y), x_offset + 10, y, 0.6, cv::Scalar(0, 0, 0), 1);
    y += line_height;
    drawText(cv::format("Yaw: %.2f deg", stats.current.yaw), x_offset + 10, y, 0.6, cv::Scalar(0, 0, 0), 1);
    y += line_height + 10;

    std::string rate_text = cv::format("Rate: %.2f Hz", stats.rate_hz);
    drawText(rate_text, x_offset + 10, y, 0.6, cv::Scalar(150, 0, 0), 1);
    y += line_height + 10;    
    
    // Mean
    drawText("[ Mean ]", x_offset, y, 0.7, cv::Scalar(0, 100, 0), 2);
    y += line_height;
    // 🚀 drawText 인자 수정 (thickness = 1 추가)
    drawText(cv::format("X: %.4f m", stats.mean.x), x_offset + 10, y, 0.6, cv::Scalar(0, 0, 0), 1);
    y += line_height;
    drawText(cv::format("Y: %.4f m", stats.mean.y), x_offset + 10, y, 0.6, cv::Scalar(0, 0, 0), 1);
    y += line_height;
    drawText(cv::format("Yaw: %.2f deg", stats.mean.yaw), x_offset + 10, y, 0.6, cv::Scalar(0, 0, 0), 1);
    y += line_height + 10;
    
    // Std Dev
    bool data_stable = stats.stddev.x < std_m && stats.stddev.y < std_m && stats.stddev.yaw < std_deg;

    if(data_stable)
    {
        drawText("[ Std Dev ]", x_offset, y, 0.7, cv::Scalar(0, 200, 0), 2);
    }
    else
    {
        drawText("[ Std Dev ]", x_offset, y, 0.7, cv::Scalar(0, 0, 200), 2);
    }

    y += line_height;
    std::string x_text = cv::format("X: %.6f m (Max: %.6f m)", stats.stddev.x, stats.max_stddev.x);
    drawText(x_text, x_offset + 10, y, 0.6, cv::Scalar(0, 0, 0), 1);
    y += line_height;
    
    // Y Std Dev + Max
    std::string y_text = cv::format("Y: %.6f m (Max: %.6f m)", stats.stddev.y, stats.max_stddev.y);
    drawText(y_text, x_offset + 10, y, 0.6, cv::Scalar(0, 0, 0), 1);
    y += line_height;
    
    // Yaw Std Dev + Max
    std::string yaw_text = cv::format("Yaw: %.4f deg (Max: %.4f deg)", stats.stddev.yaw, stats.max_stddev.yaw);
    drawText(yaw_text, x_offset + 10, y, 0.6, cv::Scalar(0, 0, 0), 1);
    y += line_height;    
    // 🚀 drawText 인자 수정 (thickness = 1 추가)
    drawText(cv::format("Samples: %d / %d", (int)stats.history.size(), sample_size_), 
            x_offset + 10, y, 0.5, cv::Scalar(100, 100, 100), 1);
}

void PoseMonitor::drawTopicList(int x_offset, int y_offset) 
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
        
        // 🚀 drawText 인자 수정 (thickness = 1 추가)
        drawText(marker + available_topics_[i], x_offset, y, 0.45, color, 1);
        y += 22;
    }
    
    if (available_topics_.size() > 15) 
    {
        // 🚀 drawText 인자 수정 (thickness = 1 추가)
        drawText(cv::format("... and %d more", (int)available_topics_.size() - 15), 
                x_offset, y, 0.4, cv::Scalar(100, 100, 100), 1);
    }
}

void PoseMonitor::drawLogHistory(int x_offset, int y_offset, int width, int height) 
{
    cv::Rect log_box(x_offset, y_offset, width, height);
    cv::rectangle(display_, log_box, cv::Scalar(255, 255, 255), cv::FILLED);
    cv::rectangle(display_, log_box, cv::Scalar(100, 100, 100), 2);
    
    drawText(cv::format("[Save History - Total: %d]", log_count_), 
            x_offset + 0, y_offset - 8, 0.6, cv::Scalar(0, 0, 0), 2);
    
    int y = y_offset + 25;
    int line_spacing = 20;
    
    if (log_history_.empty()) 
    {
        // 🚀 drawText 인자 수정 (thickness = 1 추가)
        drawText("No data saved yet. Click 'AutoSave: OFF' to start logging.", 
                x_offset + 10, y, 0.45, cv::Scalar(150, 150, 150), 1);
    } 
    else 
    {
        for (const auto& log : log_history_) 
        {
            // 🚀 drawText 인자 수정 (thickness = 1 추가)
            drawText(log, x_offset + 10, y, 0.4, cv::Scalar(0, 0, 0), 1);
            y += line_spacing;
        }
    }
}

void PoseMonitor::drawGridUI(int x_offset, int y_offset)
{
    // ----------------------------------------------------
    // 그리드 박스 배경 및 테두리
    // ----------------------------------------------------
    cv::Rect grid_box(x_offset, y_offset, GRID_SIZE, GRID_SIZE);
    cv::rectangle(display_, grid_box, cv::Scalar(255, 255, 255), cv::FILLED);
    cv::rectangle(display_, grid_box, cv::Scalar(50, 50, 50), 2);
    
    // 타이틀
    drawText("[ GLS Position(Mean) Map ]", x_offset, y_offset - 8, 0.65, cv::Scalar(0, 0, 0), 2);
    
    cv::Point origin(x_offset + GRID_SIZE / 2, y_offset + GRID_SIZE / 2);
    
    // ----------------------------------------------------
    // 좌표축 및 그리드 선 (0.01m = 10mm 간격으로 변경)
    // ----------------------------------------------------
    cv::Scalar axis_color(100, 100, 100);
    cv::Scalar grid_color(220, 220, 220);
    int center_thickness = 1;
    
    // X축 (가로) 및 Y축 (세로)
    cv::line(display_, cv::Point(x_offset, origin.y), cv::Point(x_offset + GRID_SIZE, origin.y), axis_color, center_thickness); 
    cv::line(display_, cv::Point(origin.x, y_offset), cv::Point(origin.x, y_offset + GRID_SIZE), axis_color, center_thickness); 
    
    // 보조 그리드 선 (10mm = 0.01m 간격)
    double step_m = 0.01; // 10mm
    
    // -0.02m, -0.01m, 0.01m, 0.02m 위치에 보조선 그리기 (0.03m는 테두리이므로 제외)
    for (int i = -5; i <= 5; ++i) { 
        if (i == 0) continue;
        
        // 픽셀 거리 계산: i * step_m * GRID_SCALE
        int pixel_step = (int)(i * step_m * GRID_SCALE); 
        
        // 수직선 (X)
        cv::line(display_, cv::Point(origin.x + pixel_step, y_offset), cv::Point(origin.x + pixel_step, y_offset + GRID_SIZE), grid_color, 1);
        // 수평선 (Y)
        cv::line(display_, cv::Point(x_offset, origin.y - pixel_step), cv::Point(x_offset + GRID_SIZE, origin.y - pixel_step), grid_color, 1); 
    }

    // ----------------------------------------------------
    // GLS 평균 위치 표시 (GRID_SCALE 사용)
    // ----------------------------------------------------
    double mean_x = stats2_.mean.x;
    double mean_y = stats2_.mean.y;
    double mean_yaw_deg = stats2_.mean.yaw;

    double rotated_x = mean_x;
    double rotated_y = mean_y;

    // 🚀 추가: 90도 단위 회전 변환 적용
    if (map_rotation_deg_ == 90) {
        rotated_x = -mean_y;
        rotated_y = mean_x;
    } else if (map_rotation_deg_ == 180) {
        rotated_x = -mean_x;
        rotated_y = -mean_y;
    } else if (map_rotation_deg_ == 270) {
        rotated_x = mean_y;
        rotated_y = -mean_x;
    }    
    
    // 픽셀 좌표로 변환 (Y는 화면 좌표계에서 아래로 증가하므로 부호 반전)
    int pos_x = origin.x + (int)(rotated_x * GRID_SCALE);
    int pos_y = origin.y - (int)(rotated_y * GRID_SCALE); 
    
    // 위치를 그리드 박스 안에 클리핑 (여전히 그리드 범위 내에 위치)
    pos_x = std::max(x_offset, std::min(x_offset + GRID_SIZE, pos_x));
    pos_y = std::max(y_offset, std::min(y_offset + GRID_SIZE, pos_y));
    
    // 포인트 그리기 (파란색)
    cv::circle(display_, cv::Point(pos_x, pos_y), 5, cv::Scalar(255, 0, 0), cv::FILLED);

    double yaw_rad = mean_yaw_deg * M_PI / 180.0; // Yaw을 라디안으로 변환
    // 회전 변환 행렬은 (cosθ, -sinθ), (sinθ, cosθ)를 사용하지만,
    // 각도 변환이 90도 단위이므로 간단하게 처리
    double rotated_yaw_rad = yaw_rad;

    if (map_rotation_deg_ == 90) {
        rotated_yaw_rad = yaw_rad + M_PI / 2.0;
    } else if (map_rotation_deg_ == 180) {
        rotated_yaw_rad = yaw_rad + M_PI;
    } else if (map_rotation_deg_ == 270) {
        rotated_yaw_rad = yaw_rad - M_PI / 2.0; // 또는 + 3*M_PI/2.0
    }
    
    // Yaw 라디안을 0 ~ 2*PI 범위로 정규화 (선택 사항이지만 안전함)
    rotated_yaw_rad = std::fmod(rotated_yaw_rad, 2.0 * M_PI);
    if (rotated_yaw_rad < 0) rotated_yaw_rad += 2.0 * M_PI;    

    const int HEADING_LINE_LENGTH = 15;
    
    int end_x = pos_x + (int)(HEADING_LINE_LENGTH * std::cos(rotated_yaw_rad));
    int end_y = pos_y - (int)(HEADING_LINE_LENGTH * std::sin(rotated_yaw_rad)); // 화면 Y축은 아래로 증가하므로 (-)

    cv::line(display_, cv::Point(pos_x, pos_y), cv::Point(end_x, end_y), cv::Scalar(0, 0, 255), 2);

    // 원점 표시 (녹색)
    cv::circle(display_, origin, 3, cv::Scalar(0, 150, 0), cv::FILLED); 
    
    // ----------------------------------------------------
    // 좌표 라벨 (0.03m 경계 표시)
    // ----------------------------------------------------
    // int boundary_pixel = (int)(GRID_M_RANGE * GRID_SCALE); // 0.03m를 픽셀로 변환 (125 픽셀)

    // // X = +30mm 라벨
    // drawText("+60mm", origin.x + boundary_pixel - 45, origin.y + 15, 0.4, cv::Scalar(0, 0, 0), 1);
    // // X = -30mm 라벨
    // drawText("-60mm", origin.x - boundary_pixel + 5, origin.y + 15, 0.4, cv::Scalar(0, 0, 0), 1);
    
    // // Y = +30mm 라벨 (화면상으로는 위쪽)
    // drawText("+60mm", origin.x + 5, origin.y - boundary_pixel + 15, 0.4, cv::Scalar(0, 0, 0), 1);
    // // Y = -30mm 라벨 (화면상으로는 아래쪽)
    // drawText("-60mm", origin.x + 5, origin.y + boundary_pixel - 5, 0.4, cv::Scalar(0, 0, 0), 1);

    // 현재 위치 좌표 텍스트
    // std::string pos_text = cv::format("(X: %.3f, Y: %.3f m)", mean_x, mean_y);
    // drawText(pos_text, x_offset + 5, y_offset + GRID_SIZE + 20, 0.6, cv::Scalar(255, 0, 0), 1);
    std::string pos_text = cv::format("(X: %.3f, Y: %.3f m) Yaw: %.2f deg | Rot: %d deg", 
                                        rotated_x, rotated_y, mean_yaw_deg, map_rotation_deg_); // 🚀 Yaw 추가
    drawText(pos_text, x_offset + 5, y_offset + GRID_SIZE + 20, 0.6, cv::Scalar(255, 0, 0), 1);}

// void PoseMonitor::drawGridUI(int x_offset, int y_offset)
// {
//     // ----------------------------------------------------
//     // 그리드 박스 배경 및 테두리
//     // ----------------------------------------------------
//     cv::Rect grid_box(x_offset, y_offset, GRID_SIZE, GRID_SIZE);
//     cv::rectangle(display_, grid_box, cv::Scalar(255, 255, 255), cv::FILLED);
//     cv::rectangle(display_, grid_box, cv::Scalar(50, 50, 50), 2);
    
//     // 타이틀
//     drawText("[ GLS Position(Mean) Map ]", x_offset, y_offset - 8, 0.65, cv::Scalar(0, 0, 0), 2);
    
//     // 원점 (그리드 중앙)
//     cv::Point origin(x_offset + GRID_SIZE / 2, y_offset + GRID_SIZE / 2);
    
//     // ----------------------------------------------------
//     // 좌표축 및 그리드 선
//     // ----------------------------------------------------
//     cv::Scalar axis_color(100, 100, 100);
//     cv::Scalar grid_color(220, 220, 220);
//     int center_thickness = 1;
    
//     // X축 (빨간색) 및 Y축 (녹색)
//     cv::line(display_, cv::Point(x_offset, origin.y), cv::Point(x_offset + GRID_SIZE, origin.y), axis_color, center_thickness); // X축 (가로)
//     cv::line(display_, cv::Point(origin.x, y_offset), cv::Point(origin.x, y_offset + GRID_SIZE), axis_color, center_thickness); // Y축 (세로)
    
//     // 보조 그리드 선 (1m 간격)
//     int grid_step = GRID_SCALE; // 100 픽셀/m
    
//     for (int i = -1; i <= 1; ++i) { // -1m, 1m 위치에 보조선
//         if (i == 0) continue;
        
//         // 수직선 (X=1m, X=-1m)
//         cv::line(display_, cv::Point(origin.x + i * grid_step, y_offset), cv::Point(origin.x + i * grid_step, y_offset + GRID_SIZE), grid_color, 1);
//         // 수평선 (Y=1m, Y=-1m)
//         cv::line(display_, cv::Point(x_offset, origin.y + i * grid_step), cv::Point(x_offset + GRID_SIZE, origin.y + i * grid_step), grid_color, 1);
//     }

//     // ----------------------------------------------------
//     // GLS 평균 위치 표시
//     // ----------------------------------------------------
//     double mean_x = stats2_.mean.x;
//     double mean_y = stats2_.mean.y;
    
//     // 픽셀 좌표로 변환 (Y는 화면 좌표계에서 아래로 증가하므로 부호 반전)
//     int pos_x = origin.x + (int)(mean_x * GRID_SCALE);
//     int pos_y = origin.y - (int)(mean_y * GRID_SCALE); 
    
//     // 위치를 그리드 박스 안에 클리핑
//     pos_x = std::max(x_offset, std::min(x_offset + GRID_SIZE, pos_x));
//     pos_y = std::max(y_offset, std::min(y_offset + GRID_SIZE, pos_y));
    
//     // 포인트 그리기 (파란색)
//     cv::circle(display_, cv::Point(pos_x, pos_y), 5, cv::Scalar(255, 0, 0), cv::FILLED);
    
//     // 원점 표시 (녹색)
//     cv::circle(display_, origin, 3, cv::Scalar(0, 150, 0), cv::FILLED); 
    
//     // ----------------------------------------------------
//     // 좌표 라벨
//     // ----------------------------------------------------
//     drawText("X", x_offset + GRID_SIZE, origin.y - 5, 0.4, cv::Scalar(0, 0, 0), 1);
//     drawText("Y", origin.x + 5, y_offset + 15, 0.4, cv::Scalar(0, 0, 0), 1);
//     drawText("(0,0)", origin.x + 5, origin.y - 5, 0.4, cv::Scalar(0, 0, 0), 1);
    
//     // 현재 위치 좌표 텍스트
//     std::string pos_text = cv::format("(X: %.3f, Y: %.3f)", mean_x, mean_y);
//     drawText(pos_text, x_offset + 5, y_offset + GRID_SIZE + 20, 0.6, cv::Scalar(255, 0, 0), 1);
// }

void PoseMonitor::displayCallback() 
{
    sample_size_ = cv::getTrackbarPos("Sample Size", "Pose Monitor");
    if (sample_size_ < 1) sample_size_ = 1;
    
    display_ = cv::Mat(WINDOW_HEIGHT, WINDOW_WIDTH, CV_8UC3, cv::Scalar(240, 240, 240));
    
    drawText("ROS2 Pose Monitor", 20, 40, 1.2, cv::Scalar(0, 0, 0), 2);
    

    if(current_status_)
    {
        drawText(cv::format("GLS100 dection : Success"), 
                 WINDOW_WIDTH/2 + 30, 70, 0.8, cv::Scalar(0, 128, 0), 2);
    }
    else
    {
        drawText(cv::format("GLS100 dection : Fail"), 
                 WINDOW_WIDTH/2 + 30, 70, 0.8, cv::Scalar(0, 0, 255), 2);
    }

    drawButton();
    
    std::string csv_status = csv_file_created_ ? 
        ("Logging to: " + csv_filename_) : 
        ("Ready: " + csv_filename_ + " (Save/New/Start logging)");
        
    // 🚀 drawText 인자 수정 (thickness = 1 추가)
    drawText(csv_status, 20, WINDOW_HEIGHT - 40, 0.45,
            csv_file_created_ ? cv::Scalar(0, 100, 0) : cv::Scalar(100, 100, 0), 1);
           
    drawPoseData(topic1_, stats1_, 30, 90, cv::Scalar(200, 0, 0), std_m_threshold_, std_yaw_threshold_);
    
    drawPoseData(topic2_, stats2_, WINDOW_WIDTH/2 + 30, 140, cv::Scalar(0, 150, 0), std_m_threshold_, std_yaw_threshold_); 
    
    grid_x_offset = WINDOW_WIDTH / 2 + 300; 
    grid_y_offset = 150; 
    drawGridUI(grid_x_offset, grid_y_offset);

    int button_y_offset = grid_y_offset + GRID_SIZE + 40; 
    drawRotateButton(grid_x_offset+100, button_y_offset-200);    

    int log_box_x = 10;
    int log_box_y = 700; 
    int log_box_width = WINDOW_WIDTH - log_box_x - 30;
    int log_box_height = 200;
    drawLogHistory(log_box_x, log_box_y, log_box_width, log_box_height);        

    // 🚀 drawText 인자 수정 (thickness = 1 추가)
    drawText("Press 's' to Save & Move CSV | 'n' for New File | 'r' to refresh | 'q' to quit", 
        20, WINDOW_HEIGHT - 20, 0.5, cv::Scalar(100, 100, 100), 1);
    
    cv::imshow("Pose Monitor", display_);
    
    // std::cout << "auto save : " << auto_save_enabled_ << " " << was_stable_ << " " << is_stable_ << std::endl;
    // // ----------------------------------------------------
    // // 자동 저장 로직 실행
    // // ----------------------------------------------------
    // if (auto_save_enabled_) 
    // {
    //     if (!was_stable_ && is_stable_) 
    //     {
    //         stable_count = 0;
    //         saveToCSV(); 
    //         RCLCPP_INFO(this->get_logger(), "AutoSave Triggered: Stable state achieved.");
    //     }
    // }
    
    // was_stable_ = is_stable_; 
    // ----------------------------------------------------

    bool topic1_changed = 
        std::abs(stats1_.current.x - last_pose1_.x) > MIN_CHANGE_M ||
        std::abs(stats1_.current.y - last_pose1_.y) > MIN_CHANGE_M ||
        std::abs(stats1_.current.yaw - last_pose1_.yaw) > MIN_CHANGE_YAW;

    bool topic2_changed = 
        std::abs(stats2_.current.x - last_pose2_.x) > MIN_CHANGE_M ||
        std::abs(stats2_.current.y - last_pose2_.y) > MIN_CHANGE_M ||
        std::abs(stats2_.current.yaw - last_pose2_.yaw) > MIN_CHANGE_YAW;

    bool data_has_flow = topic1_changed && topic2_changed;


    // 🚀 1초 카운트 기간 동안 데이터 흐름(변화)이 한 번이라도 있었는지 누적 추적
    if (data_has_flow) {
        data_changed_during_stable_count_ = true;
    }    

    // std::cout << "change : " << topic1_changed << " " << topic2_changed << " " << data_has_flow << std::endl;

    // ----------------------------------------------------
    // 자동 저장 로직 실행 (1초 지연 및 반복 저장 방지)
    // ----------------------------------------------------
    if (!is_stable_) 
    {
        // 🚀 이동/불안정 상태: 카운터 및 저장 플래그 리셋
        stable_count_ = 0;
        save_fired_ = false; // 이동이 감지되면 다음 저장을 허용
        data_changed_during_stable_count_ = false;
    }
    else 
    {
        // 🚀 안정 상태: save_fired_가 false일 때만 카운트 증가
        if (save_fired_ == false) 
        {
            if (stable_count_ < STABILITY_FRAMES_THRESHOLD) 
            {
                stable_count_++;
            }
        }
    }
    
    // if (auto_save_enabled_) 
    // {
    //     // 🚀 임계값에 도달했고, 아직 저장이 되지 않았다면 (save_fired_가 false)
    //     if (stable_count_ == STABILITY_FRAMES_THRESHOLD && save_fired_ == false) 
    //     {
    //         saveToCSV(); 
    //         RCLCPP_INFO(this->get_logger(), "AutoSave Triggered: Stable state achieved and held for 1 second.");
            
    //         // 🚀 저장 완료 플래그 설정: 다음 이동이 있을 때까지 저장 방지
    //         save_fired_ = true; 
            
    //         // stable_count_는 이미 임계값에 도달했으므로 더 이상 증가하지 않음
    //         // (is_stable_이 계속 true이고 save_fired_가 true이면 stable_count_는 증가하지 않음)
    //     }
    // }    
    if (auto_save_enabled_) 
        {
        // 🚀 저장 조건 확인: 
        // 1. 임계값에 도달했고 (1초 유지)
        // 2. 아직 저장이 되지 않았으며
        // 3. AND (추가 조건) 1초 동안 포즈 값이 변화했을 때 (데이터 정체 방지)
        if (stable_count_ == STABILITY_FRAMES_THRESHOLD && 
            save_fired_ == false &&
            data_changed_during_stable_count_ == true) 
        {
            saveToCSV(); 
            RCLCPP_INFO(this->get_logger(), "AutoSave Triggered: Stable state achieved and held for 1 second with data flow.");
            
            // 저장 완료 플래그 설정
            save_fired_ = true; 
        }
        else if (stable_count_ == STABILITY_FRAMES_THRESHOLD && save_fired_ == false)
        {
            // 1초 안정 상태는 달성했으나, 데이터 정체로 인해 저장을 건너뛴 경우
             RCLCPP_WARN(this->get_logger(), "AutoSave Skipped: Stable state achieved but no data updated during 1 second count.");
             
             // 다음 이동이 있을 때까지 저장을 시도하지 않도록 플래그 설정
             save_fired_ = true; 
        }
    }
    
    last_pose1_ = stats1_.current;
    last_pose2_ = stats2_.current;

    int key = cv::waitKey(1);
    if (key == 'q' || key == 27) 
    { 
        std::exit(0);
    } 
    else if (key == 'r') 
    { 
        getAvailableTopics();
        RCLCPP_INFO(this->get_logger(), "Topics refreshed");
    }
    else if (key == 's') 
    { 
        closeAndMoveCSV();
    }
    else if (key == 'n') 
    { 
        prepareNewCSV();
    }
    else if (key == 'c') 
    {
        // Topic 1 (VLOC) 통계 리셋
        stats1_.history.clear();
        stats1_.mean = {0.0, 0.0, 0.0};
        stats1_.stddev = {0.0, 0.0, 0.0};
        stats1_.max_stddev = {0.0, 0.0, 0.0}; // 최대 표준 편차도 리셋

        // Topic 2 (GLS) 통계 리셋
        stats2_.history.clear();
        stats2_.mean = {0.0, 0.0, 0.0};
        stats2_.stddev = {0.0, 0.0, 0.0};
        stats2_.max_stddev = {0.0, 0.0, 0.0}; // 최대 표준 편차도 리셋
        
        // 안정성 관련 플래그도 리셋
        stable_count_ = 0;
        save_fired_ = false;
        data_changed_during_stable_count_ = false;

        RCLCPP_INFO(this->get_logger(), "Statistics Reset: 'c' key pressed. History and Max StdDev cleared.");
    }
}

// ====================================================================
// main 함수
// ====================================================================

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<PoseMonitor>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown(); 
    return 0;
}