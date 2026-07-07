#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/empty.hpp>
#include <Eigen/Dense>
#include <random>
#include <string>
#include <vector>

// rosidl 생성 헤더
#include "rtk_lio/msg/pose_update.hpp"
#include "rtk_lio/srv/set_origin.hpp"
#include "rtk_lio/srv/set_datum.hpp"

namespace rtk_lio
{
class GnssCorrectorNode : public rclcpp::Node
{
public:
    GnssCorrectorNode();

private:
    //  내부 초기화 상태머신 
    enum class InitState { WAIT_LIO, WAIT_GNSS_STABLE, RUNNING };

    //  NCU 노출 시스템 상태머신 
    enum class SystemState    { INITIALIZING, LOCALIZED, DEGRADED, ERROR, SHUTTING_DOWN };
    enum class SystemSubstate { GNSS_WAIT, TRACKING, LIO_ONLY, SENSOR_FAULT, LOG_SYNC };

    //  RTK 품질 
    enum class RtkQuality { NONE, SINGLE, FLOAT, FIX };

    //  센서 콜백 
    void lioCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void headingCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg);
    void onKeyOff(const std_msgs::msg::Empty::SharedPtr msg);

    //  NCU 서비스 핸들러 
    void handleSetOrigin(
        const std::shared_ptr<rtk_lio::srv::SetOrigin::Request>  req,
        std::shared_ptr<rtk_lio::srv::SetOrigin::Response>        res);
    void handleSetDatum(
        const std::shared_ptr<rtk_lio::srv::SetDatum::Request>   req,
        std::shared_ptr<rtk_lio::srv::SetDatum::Response>         res);

    //  초기화 
    void tryGnssInit();

    //  EKF 연산 
    void ekfPredict(double dt);
    bool ekfUpdatePosition(double mx_meas, double my_meas,
                           const Eigen::Matrix2d & R, double max_correction);
    void ekfUpdateHeading(double yaw_map_meas, double yaw_r);

    //  Datum 관리 
    void reinitDrift();
    bool saveDatumToFile(double map_x, double map_y, double map_heading) const;
    void saveStateToFile() const;

    //  발행 
    void publishMapOdom(const nav_msgs::msg::Odometry::SharedPtr & lio_msg);
    void publishGnssOdom(double mx, double my, double r_pos, const rclcpp::Time & stamp);
    void publishPoseUpdate(const nav_msgs::msg::Odometry::SharedPtr & lio_msg);
    void publishDiagnostics();
    void broadcastMapOdomTf(const rclcpp::Time & stamp);
    void appendPath(nav_msgs::msg::Path & path,
                    const geometry_msgs::msg::PoseStamped & pose,
                    std::size_t max_poses);

    //  상태 계산 
    uint8_t        computeQuality()        const;
    SystemState    computeSystemState()    const;
    SystemSubstate computeSystemSubstate() const;
    static const char * systemStateStr(SystemState s);
    static const char * systemSubstateStr(SystemSubstate s);

    //  좌표 변환 헬퍼 
    void latLonToEnu(double lat, double lon, double & e, double & n) const;
    void latLonToMap(double lat, double lon, double & mx, double & my) const;
    void enuFromRefToLatLon(double ref_lat, double ref_lon,
                             double e_offset, double n_offset,
                             double & lat, double & lon) const;
    double quaternionToYaw(const geometry_msgs::msg::Quaternion & q) const;

    //  통계 헬퍼 
    static double sampleMean(const std::vector<double> & v);
    static double sampleStd(const std::vector<double> & v, double mean);
    static double circularMean(const std::vector<double> & angles);
    static double circularStd(const std::vector<double> & angles, double mean);

    
    // 구독
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr              sub_lio_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr          sub_fix_;
    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr sub_heading_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr                 sub_key_off_;

    // 발행 — 디버깅/시각화
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_map_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_gnss_odom_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr     pub_path_map_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr     pub_path_gnss_;

    // 발행 — NCU 인터페이스
    rclcpp::Publisher<rtk_lio::msg::PoseUpdate>::SharedPtr              pub_pose_update_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;

    // 서비스 서버 — NCU → LPC
    rclcpp::Service<rtk_lio::srv::SetOrigin>::SharedPtr srv_set_origin_;
    rclcpp::Service<rtk_lio::srv::SetDatum>::SharedPtr  srv_set_datum_;

    // 타이머
    rclcpp::TimerBase::SharedPtr diag_timer_;

    // TF 브로드캐스터
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // 누적 경로 (ring-buffer)
    nav_msgs::msg::Path path_map_;
    nav_msgs::msg::Path path_gnss_;
    static constexpr std::size_t PATH_MAX_POSES = 100000;

    
    // 파라미터
    
    std::string frame_map_;
    std::string frame_odom_;
    std::string frame_body_;
    std::string topic_lio_;
    std::string topic_fix_;
    std::string topic_heading_;
    std::string datum_file_path_;
    std::string state_file_path_;

    double yaw_offset_;
    double pos_process_noise_;
    double yaw_process_noise_;
    double pos_r_fix_;
    double pos_r_float_;
    double pos_r_single_;
    double yaw_r_;
    double max_pos_correction_;
    double max_pos_correction_resume_;
    double max_yaw_correction_;
    double max_heading_innovation_;
    bool   use_rtk_covariance_;
    double restart_gap_s_;
    double pos_innovation_gate_;
    int    gnss_freeze_count_;
    int    gnss_recover_count_;
    int    gnss_init_samples_;
    double gnss_init_pos_std_;
    double gnss_init_yaw_std_;
    double antenna_offset_x_;
    double antenna_offset_y_;
    double body_to_base_x_;   // body → base_link 오프셋 (body 프레임, 전방)
    double body_to_base_y_;   // body → base_link 오프셋 (body 프레임, 좌방)
    double body_to_base_z_;   // body → base_link 오프셋 (body 프레임, 상방)
    bool   enable_noise_sim_;
    double multipath_prob_;
    double multipath_magnitude_;
    double multipath_duration_s_;
    double shadow_prob_;
    double shadow_duration_s_;
    double pos_converge_threshold_;
    double relocalize_threshold_;
    double lidar_timeout_s_;

    
    // EKF 상태: [dx, dy, dyaw]  (map 프레임 기준)
    
    Eigen::Vector3d drift_;
    Eigen::Matrix3d P_;

    
    // Datum — map 프레임 원점 및 회전
    
    double datum_lat_;
    double datum_lon_;
    double datum_yaw_;  // map X축의 ENU 기준 방위각 [rad, CCW from East]

    // WGS84 상수
    static constexpr double WGS84_A  = 6378137.0;
    static constexpr double WGS84_E2 = 0.00669437999014;
    static constexpr double DEG2RAD  = M_PI / 180.0;

    
    // 상태머신
    
    InitState init_state_;
    bool      shutting_down_;

    
    // 센서 캐시
    
    nav_msgs::msg::Odometry::SharedPtr last_lio_;
    rclcpp::Time last_lio_stamp_;      // msg->header.stamp (하드웨어 클럭, dt 계산용)
    rclcpp::Time last_lio_recv_time_;  // this->now() 수신 시점 (timeout 판정용)
    bool         lio_received_;

    double last_heading_yaw_;  // ENU 기준 GPS heading [rad]
    bool   heading_received_;

    double last_fix_lat_;      // 마지막 raw GNSS 위도
    double last_fix_lon_;      // 마지막 raw GNSS 경도
    bool   fix_received_;

    double last_map_x_;
    double last_map_y_;
    bool   last_map_valid_;

    RtkQuality rtk_quality_;

    
    // NCU 인터페이스 상태
    
    uint8_t      quality_;
    bool         relocalization_;
    rclcpp::Time last_ncu_contact_time_;
    bool         ncu_contacted_;

    
    // GNSS freeze / soft-resume
    
    int          gnss_reject_count_;
    int          gnss_good_count_;
    bool         gnss_frozen_;
    rclcpp::Time gnss_freeze_start_time_;
    bool         gnss_freeze_time_valid_;

    
    // GNSS 초기화 샘플 버퍼
    
    std::vector<double> init_e_samples_;  // map frame x
    std::vector<double> init_n_samples_;  // map frame y
    std::vector<double> init_yaw_samples_;  // ENU heading

    
    // 노이즈 시뮬레이션
    
    bool   in_multipath_;
    double multipath_end_sim_s_;
    double multipath_e_offset_;
    double multipath_n_offset_;
    bool   in_shadow_;
    double shadow_end_sim_s_;
    std::mt19937 rng_;
    std::uniform_real_distribution<double> uniform01_;
    std::uniform_real_distribution<double> uniform_angle_;
};

}  // namespace rtk_lio
