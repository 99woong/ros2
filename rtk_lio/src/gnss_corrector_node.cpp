#include "rtk_lio/gnss_corrector_node.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include <cmath>
#include <algorithm>
#include <numeric>
#include <fstream>
#include <sstream>
#include <iomanip>

namespace rtk_lio
{

GnssCorrectorNode::GnssCorrectorNode()
: Node("gnss_corrector_node"),
  drift_(Eigen::Vector3d::Zero()),
  P_(Eigen::Matrix3d::Zero()),
  datum_lat_(0.0), datum_lon_(0.0), datum_yaw_(0.0),
  init_state_(InitState::WAIT_LIO),
  shutting_down_(false),
  last_lio_recv_time_(0, 0, RCL_ROS_TIME),
  lio_received_(false),
  last_heading_yaw_(0.0),
  heading_received_(false),
  last_fix_lat_(0.0), last_fix_lon_(0.0), fix_received_(false),
  last_map_x_(0.0), last_map_y_(0.0), last_map_valid_(false),
  rtk_quality_(RtkQuality::NONE),
  quality_(0),
  relocalization_(false),
  ncu_contacted_(false),
  gnss_reject_count_(0),
  gnss_good_count_(0),
  gnss_frozen_(false),
  gnss_freeze_time_valid_(false),
  in_multipath_(false), multipath_end_sim_s_(0.0),
  multipath_e_offset_(0.0), multipath_n_offset_(0.0),
  in_shadow_(false), shadow_end_sim_s_(0.0),
  rng_(std::random_device{}()),
  uniform01_(0.0, 1.0),
  uniform_angle_(0.0, 2.0 * M_PI)
{
  // 프레임,토픽
  declare_parameter<std::string>("frame_map",      "map");
  declare_parameter<std::string>("frame_odom",     "odom");
  declare_parameter<std::string>("frame_body",     "body");
  declare_parameter<std::string>("topic_lio",      "/Odometry");
  declare_parameter<std::string>("topic_fix",      "/fix");
  declare_parameter<std::string>("topic_heading",  "/heading");

  //Datum
  declare_parameter<double>("datum_lat", 0.0);
  declare_parameter<double>("datum_lon", 0.0);
  declare_parameter<double>("datum_yaw", 0.0);

  //안테나 레버암
  declare_parameter<double>("antenna_offset_x", 0.0);
  declare_parameter<double>("antenna_offset_y", 0.0);
  declare_parameter<double>("body_to_base_link_x", 0.0);
  declare_parameter<double>("body_to_base_link_y", 0.0);
  declare_parameter<double>("body_to_base_link_z", 0.0);

  //EKF 튜닝
  declare_parameter<double>("yaw_offset",         3.20);
  declare_parameter<double>("pos_process_noise",  0.001);
  declare_parameter<double>("yaw_process_noise",  0.0001);
  declare_parameter<double>("pos_r_fix",          0.5);
  declare_parameter<double>("pos_r_float",        5.0);
  declare_parameter<double>("pos_r_single",       100.0);
  declare_parameter<double>("yaw_r",              0.01);
  declare_parameter<double>("max_pos_correction",        0.3);
  declare_parameter<double>("max_pos_correction_resume", 0.15);
  declare_parameter<double>("max_yaw_correction",        0.3);
  declare_parameter<double>("max_heading_innovation",    0.3);
  declare_parameter<double>("pos_innovation_gate",       9.21);
  declare_parameter<int>   ("gnss_freeze_count",         5);
  declare_parameter<int>   ("gnss_recover_count",        3);
  declare_parameter<bool>  ("use_rtk_covariance", false);
  declare_parameter<double>("restart_gap_s",      3.0);

  //GNSS 초기화 안정성 임계값
  declare_parameter<int>   ("gnss_init_samples",  10);
  declare_parameter<double>("gnss_init_pos_std",   0.3);
  declare_parameter<double>("gnss_init_yaw_std",   0.02);

  // quality / relocalization 임계값
  declare_parameter<double>("pos_converge_threshold", 0.1);
  declare_parameter<double>("relocalize_threshold",   0.03);
  declare_parameter<double>("lidar_timeout_s",        2.0);

  //상태 저장 파일 경로
  declare_parameter<std::string>("datum_file_path", "/tmp/lpc_datum.yaml");
  declare_parameter<std::string>("state_file_path", "/tmp/lpc_state.yaml");

  // 노이즈 시뮬레이션
  declare_parameter<bool>  ("enable_noise_sim",     false);
  declare_parameter<double>("multipath_prob",        0.05);
  declare_parameter<double>("multipath_magnitude",   8.0);
  declare_parameter<double>("multipath_duration_s",  4.0);
  declare_parameter<double>("shadow_prob",           0.03);
  declare_parameter<double>("shadow_duration_s",     10.0);

  //파라미터 로드
  frame_map_          = get_parameter("frame_map").as_string();
  frame_odom_         = get_parameter("frame_odom").as_string();
  frame_body_         = get_parameter("frame_body").as_string();
  topic_lio_          = get_parameter("topic_lio").as_string();
  topic_fix_          = get_parameter("topic_fix").as_string();
  topic_heading_      = get_parameter("topic_heading").as_string();
  datum_lat_          = get_parameter("datum_lat").as_double();
  datum_lon_          = get_parameter("datum_lon").as_double();
  datum_yaw_          = get_parameter("datum_yaw").as_double();
  yaw_offset_         = get_parameter("yaw_offset").as_double();
  pos_process_noise_  = get_parameter("pos_process_noise").as_double();
  yaw_process_noise_  = get_parameter("yaw_process_noise").as_double();
  pos_r_fix_          = get_parameter("pos_r_fix").as_double();
  pos_r_float_        = get_parameter("pos_r_float").as_double();
  pos_r_single_       = get_parameter("pos_r_single").as_double();
  yaw_r_              = get_parameter("yaw_r").as_double();
  max_pos_correction_        = get_parameter("max_pos_correction").as_double();
  max_pos_correction_resume_ = get_parameter("max_pos_correction_resume").as_double();
  max_yaw_correction_        = get_parameter("max_yaw_correction").as_double();
  max_heading_innovation_    = get_parameter("max_heading_innovation").as_double();
  use_rtk_covariance_        = get_parameter("use_rtk_covariance").as_bool();
  pos_innovation_gate_       = get_parameter("pos_innovation_gate").as_double();
  gnss_freeze_count_         = get_parameter("gnss_freeze_count").as_int();
  gnss_recover_count_        = get_parameter("gnss_recover_count").as_int();
  restart_gap_s_      = get_parameter("restart_gap_s").as_double();
  gnss_init_samples_  = get_parameter("gnss_init_samples").as_int();
  gnss_init_pos_std_  = get_parameter("gnss_init_pos_std").as_double();
  gnss_init_yaw_std_  = get_parameter("gnss_init_yaw_std").as_double();
  pos_converge_threshold_ = get_parameter("pos_converge_threshold").as_double();
  relocalize_threshold_   = get_parameter("relocalize_threshold").as_double();
  lidar_timeout_s_        = get_parameter("lidar_timeout_s").as_double();
  datum_file_path_    = get_parameter("datum_file_path").as_string();
  state_file_path_    = get_parameter("state_file_path").as_string();
  enable_noise_sim_   = get_parameter("enable_noise_sim").as_bool();
  multipath_prob_     = get_parameter("multipath_prob").as_double();
  multipath_magnitude_= get_parameter("multipath_magnitude").as_double();
  multipath_duration_s_= get_parameter("multipath_duration_s").as_double();
  shadow_prob_        = get_parameter("shadow_prob").as_double();
  shadow_duration_s_  = get_parameter("shadow_duration_s").as_double();
  antenna_offset_x_   = get_parameter("antenna_offset_x").as_double();
  antenna_offset_y_   = get_parameter("antenna_offset_y").as_double();
  body_to_base_x_     = get_parameter("body_to_base_link_x").as_double();
  body_to_base_y_     = get_parameter("body_to_base_link_y").as_double();
  body_to_base_z_     = get_parameter("body_to_base_link_z").as_double();

  //구독
  sub_lio_ = create_subscription<nav_msgs::msg::Odometry>(
    topic_lio_, rclcpp::SensorDataQoS(),
    std::bind(&GnssCorrectorNode::lioCallback, this, std::placeholders::_1));

  sub_fix_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    topic_fix_, rclcpp::SensorDataQoS(),
    std::bind(&GnssCorrectorNode::fixCallback, this, std::placeholders::_1));

  sub_heading_ = create_subscription<geometry_msgs::msg::QuaternionStamped>(
    topic_heading_, rclcpp::SensorDataQoS(),
    std::bind(&GnssCorrectorNode::headingCallback, this, std::placeholders::_1));

  {
    rclcpp::QoS qos_rl(1);
    qos_rl.reliable();
    qos_rl.transient_local();
    sub_key_off_ = create_subscription<std_msgs::msg::Empty>(
      "/ncu/system/key_off", qos_rl,
      std::bind(&GnssCorrectorNode::onKeyOff, this, std::placeholders::_1));
  }

  //  발행 — 디버깅 
  pub_odom_map_  = create_publisher<nav_msgs::msg::Odometry>("/odometry/map",  rclcpp::QoS(1000));
  pub_gnss_odom_ = create_publisher<nav_msgs::msg::Odometry>("/odometry/gnss", rclcpp::QoS(1000));
  pub_path_map_  = create_publisher<nav_msgs::msg::Path>("/path/map",  rclcpp::QoS(10));
  pub_path_gnss_ = create_publisher<nav_msgs::msg::Path>("/path/gnss", rclcpp::QoS(10));

  //  발행 — NCU 인터페이스 
  pub_pose_update_ = create_publisher<rtk_lio::msg::PoseUpdate>(
    "/lpc/localization/pose_update",
    rclcpp::QoS(10).best_effort());

  {
    rclcpp::QoS qos_rl(10);
    qos_rl.reliable();
    qos_rl.transient_local();
    pub_diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/lpc/diagnostics", qos_rl);
  }

  //  서비스 서버 
  srv_set_origin_ = create_service<rtk_lio::srv::SetOrigin>(
    "/lpc/localization/set_origin",
    std::bind(&GnssCorrectorNode::handleSetOrigin, this,
              std::placeholders::_1, std::placeholders::_2));

  srv_set_datum_ = create_service<rtk_lio::srv::SetDatum>(
    "/lpc/localization/set_datum",
    std::bind(&GnssCorrectorNode::handleSetDatum, this,
              std::placeholders::_1, std::placeholders::_2));

  //  1 Hz diagnostics 타이머 
  diag_timer_ = create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&GnssCorrectorNode::publishDiagnostics, this));


  path_map_.header.frame_id  = frame_map_;
  path_gnss_.header.frame_id = frame_map_;

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  RCLCPP_INFO(get_logger(),
    "GnssCorrectorNode started. datum=(%.9f, %.9f, %.4f rad) noise_sim=%s",
    datum_lat_, datum_lon_, datum_yaw_, enable_noise_sim_ ? "ON" : "off");
  RCLCPP_INFO(get_logger(),
    "Waiting for LIO... (init: %d samples, pos_std<%.2fm, yaw_std<%.3frad)",
    gnss_init_samples_, gnss_init_pos_std_, gnss_init_yaw_std_);
}

// ============================================================
// LIO 콜백 — EKF predict + 발행
// ============================================================
void GnssCorrectorNode::lioCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const rclcpp::Time now = msg->header.stamp;

  if (lio_received_ && init_state_ == InitState::RUNNING)
  {
    double dt = (now - last_lio_stamp_).seconds();

    if (dt > restart_gap_s_)
    {
      RCLCPP_WARN(get_logger(), "LIO restart detected (dt=%.2fs). Seamless re-init.", dt);
      if (last_map_valid_)
      {
        drift_(0) = last_map_x_;
        drift_(1) = last_map_y_;
      }
      else
      {
        drift_ = Eigen::Vector3d::Zero();
      }
      P_(0, 0) = 100.0;  P_(0, 1) = 0.0;
      P_(1, 0) = 0.0;    P_(1, 1) = 100.0;
      P_(0, 2) = 0.0;    P_(2, 0) = 0.0;
      P_(1, 2) = 0.0;    P_(2, 1) = 0.0;
    }
    else if (dt < 0.0)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Backward LIO timestamp (dt=%.4fs), skipping predict.", dt);
    }
    else if (dt > 1e-6)
    {
      ekfPredict(dt);
    }
  }

  last_lio_       = msg;
  last_lio_stamp_      = now;
  last_lio_recv_time_  = this->now();

  if (!lio_received_)
  {
    lio_received_ = true;
    init_state_   = InitState::WAIT_GNSS_STABLE;
    RCLCPP_INFO(get_logger(), "LIO received. Waiting for stable GNSS to initialise drift...");
  }

  if (init_state_ == InitState::RUNNING)
  {
    quality_ = computeQuality();

    //  quality 판정 근거 로그 (5초 주기) 
    // quality 0: LiDAR 없음 or 초기화 미완료 or timeout
    // quality 1: GNSS NONE
    // quality 2: frozen >= 60s
    // quality 3: FLOAT/SINGLE or frozen < 60s
    // quality 4: FIX + not frozen + P_trace >= threshold
    // quality 5: FIX + not frozen + P_trace < threshold
    {
      const double freeze_s = (gnss_frozen_ && gnss_freeze_time_valid_)
                              ? (now - gnss_freeze_start_time_).seconds() : 0.0;
      const double p_trace  = P_(0, 0) + P_(1, 1);
      const char * rtk_str  =
        (rtk_quality_ == RtkQuality::FIX)    ? "FIX"    :
        (rtk_quality_ == RtkQuality::FLOAT)   ? "FLOAT"  :
        (rtk_quality_ == RtkQuality::SINGLE)  ? "SINGLE" : "NONE";

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "[QUALITY=%d] rtk=%-6s frozen=%-3s(rej=%d good=%d/%d freeze=%.0fs) "
        "P_trace=%.4f(thresh=%.4f) reloc=%s",
        quality_, rtk_str,
        gnss_frozen_ ? "YES" : "no",
        gnss_reject_count_, gnss_good_count_, gnss_recover_count_,
        freeze_s, p_trace, pos_converge_threshold_,
        relocalization_ ? "true" : "false");
    }

    publishMapOdom(msg);
    publishPoseUpdate(msg);
    broadcastMapOdomTf(now);
  }
}

// GPS fix 콜백 — 초기화 샘플수집 또는 EKF 업데이트
void GnssCorrectorNode::fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  const int status = msg->status.status;
  if (status < 0)
  {
    rtk_quality_ = RtkQuality::NONE;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "GPS: NO FIX");
    return;
  }

  rtk_quality_ = (status >= 2) ? RtkQuality::FIX :
                 (status == 1) ? RtkQuality::FLOAT :
                                 RtkQuality::SINGLE;

  // 원시 lat/lon 저잔 (set_origin / set_datum용)
  last_fix_lat_ = msg->latitude;
  last_fix_lon_ = msg->longitude;
  fix_received_ = true;

  // lat/lon → map 프레임 좌표 변환
  double mx_raw, my_raw;
  latLonToMap(msg->latitude, msg->longitude, mx_raw, my_raw);

  // 레버암 보정: 안테나 위치(body frame) → map frame
  if (last_lio_ && (antenna_offset_x_ != 0.0 || antenna_offset_y_ != 0.0))
  {
    const double dyaw_est = (init_state_ == InitState::RUNNING) ? drift_(2) : 0.0;
    const double theta    = quaternionToYaw(last_lio_->pose.pose.orientation) + dyaw_est;
    const double cb = std::cos(theta);
    const double sb = std::sin(theta);
    mx_raw -= cb * antenna_offset_x_ - sb * antenna_offset_y_;
    my_raw -= sb * antenna_offset_x_ + cb * antenna_offset_y_;
  }

  //  초기화 단계: 샘플 수집 
  if (init_state_ == InitState::WAIT_GNSS_STABLE)
  {
    if (rtk_quality_ == RtkQuality::FIX)
    {
      init_e_samples_.push_back(mx_raw);
      init_n_samples_.push_back(my_raw);
      tryGnssInit();
    }
    else
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Init: waiting for RTK FIX (current: %s)",
        rtk_quality_ == RtkQuality::FLOAT ? "FLOAT" : "SINGLE");
    }
    return;
  }

  if (init_state_ != InitState::RUNNING) return;

  //정상 EKF 업데이트
  double r_pos = (rtk_quality_ == RtkQuality::FIX)   ? pos_r_fix_ :
                 (rtk_quality_ == RtkQuality::FLOAT)  ? pos_r_float_ :
                                                         pos_r_single_;
  if (use_rtk_covariance_ &&
      msg->position_covariance_type !=
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN)
  {
    r_pos = std::max({msg->position_covariance[0],
                      msg->position_covariance[4], 0.01});
  }

  const double sim_s = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  double mx_noisy = mx_raw;
  double my_noisy = my_raw;
  bool   skip_ekf = false;
  bool   is_shadow = false;

  if (enable_noise_sim_)
  {
    if (in_shadow_)
    {
      if (sim_s >= shadow_end_sim_s_)
      {
        in_shadow_ = false;
        RCLCPP_INFO(get_logger(), "[NOISE] GNSS shadow ended.");
      }
      else
      {
        skip_ekf  = true;
        is_shadow = true;
      }
    }
    else if (uniform01_(rng_) < shadow_prob_)
    {
      in_shadow_        = true;
      double dur        = shadow_duration_s_ * (0.5 + 0.5 * uniform01_(rng_));
      shadow_end_sim_s_ = sim_s + dur;
      skip_ekf  = true;
      is_shadow = true;
      RCLCPP_WARN(get_logger(), "[NOISE] GNSS shadow START  duration=%.1fs", dur);
    }

    if (!is_shadow)
    {
      if (in_multipath_)
      {
        if (sim_s >= multipath_end_sim_s_)
        {
          in_multipath_ = false;
          RCLCPP_INFO(get_logger(), "[NOISE] Multipath ended.");
        }
      }
      else if (uniform01_(rng_) < multipath_prob_)
      {
        in_multipath_        = true;
        double dur           = multipath_duration_s_ * (0.5 + 0.5 * uniform01_(rng_));
        multipath_end_sim_s_ = sim_s + dur;
        double angle         = uniform_angle_(rng_);
        double mag           = multipath_magnitude_ * (0.5 + 0.5 * uniform01_(rng_));
        multipath_e_offset_  = mag * std::cos(angle);
        multipath_n_offset_  = mag * std::sin(angle);
        RCLCPP_WARN(get_logger(),
          "[NOISE] Multipath START  offset=(%.1f, %.1f)m  duration=%.1fs",
          multipath_e_offset_, multipath_n_offset_, dur);
      }
      if (in_multipath_)
      {
        mx_noisy += multipath_e_offset_;
        my_noisy += multipath_n_offset_;
        r_pos = std::max(r_pos, multipath_magnitude_ * multipath_magnitude_);
      }
    }
  }

  if (!is_shadow)
  {
    publishGnssOdom(mx_noisy, my_noisy, r_pos, msg->header.stamp);
  }

  if (!skip_ekf)
  {
    Eigen::Matrix2d R_mat = Eigen::Matrix2d::Zero();
    R_mat(0, 0) = r_pos;
    R_mat(1, 1) = r_pos;

    const bool try_update = !gnss_frozen_ || rtk_quality_ == RtkQuality::FIX;

    if (try_update)
    {
      const double cap = gnss_frozen_ ? max_pos_correction_resume_ : max_pos_correction_;
      const bool accepted = ekfUpdatePosition(mx_noisy, my_noisy, R_mat, cap);

      if (accepted)
      {
        gnss_reject_count_ = 0;
        gnss_good_count_++;

        if (gnss_frozen_)
        {
          RCLCPP_INFO(get_logger(),
            "[FREEZE RECOVER] good=%d/%d — freeze 해제까지 %d회 남음",
            gnss_good_count_, gnss_recover_count_,
            gnss_recover_count_ - gnss_good_count_);

          if (gnss_good_count_ >= gnss_recover_count_)
          {
            gnss_frozen_            = false;
            gnss_freeze_time_valid_ = false;
            gnss_good_count_        = 0;
            RCLCPP_INFO(get_logger(),
              "=== GNSS freeze LIFTED === GPS 융합 정상 재개 (RTK FIX %d회 연속 통과)",
              gnss_recover_count_);
          }
        }
      }
      else
      {
        gnss_good_count_ = 0;
        if (!gnss_frozen_)
        {
          gnss_reject_count_++;
          if (gnss_reject_count_ >= gnss_freeze_count_)
          {
            gnss_frozen_            = true;
            gnss_freeze_start_time_ = this->now();
            gnss_freeze_time_valid_ = true;
            RCLCPP_WARN(get_logger(),
              "=== GNSS FROZEN === chi2 거부 %d회 연속 → FAST-LIO 단독 운용",
              gnss_reject_count_);
          }
        }
        else
        {
          // freeze 중 회복 시도했지만 chi2 거부 → good_count 리셋 로그
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "[FREEZE RECOVER] chi2 거부로 good_count 리셋 (good=%d→0)",
            gnss_good_count_);
        }
      }
    }
    else
    {
      gnss_good_count_ = 0;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "GNSS frozen — RTK FIX 대기 중 (현재: %s)",
        (rtk_quality_ == RtkQuality::FLOAT) ? "FLOAT" : "SINGLE");
    }
  }

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
    "GPS[%s%s%s] mx=%.2f my=%.2f | drift=[%.3f, %.3f, %.4f] | rej=%d good=%d",
    (rtk_quality_ == RtkQuality::FIX)   ? "FIX" :
    (rtk_quality_ == RtkQuality::FLOAT) ? "FLOAT" : "SGL",
    is_shadow ? "+SHADOW" : (in_multipath_ ? "+MULTI" : ""),
    gnss_frozen_ ? " FROZEN" : "",
    mx_noisy, my_noisy, drift_(0), drift_(1), drift_(2),
    gnss_reject_count_, gnss_good_count_);
}

// GPS heading 콜백 — 초기화 샘플 수집 또는 EKF 업데이트
void GnssCorrectorNode::headingCallback(
  const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
{
  double yaw_raw = quaternionToYaw(msg->quaternion);
  // GPS heading: CW-from-North (NED) → CCW-from-East (ENU)
  double yaw_enu = -yaw_raw + yaw_offset_;
  while (yaw_enu >  M_PI) yaw_enu -= 2.0 * M_PI;
  while (yaw_enu < -M_PI) yaw_enu += 2.0 * M_PI;

  //  초기화 단계: ENU heading 샘플 수집 
  if (init_state_ == InitState::WAIT_GNSS_STABLE)
  {
    if (rtk_quality_ != RtkQuality::NONE)
    {
      init_yaw_samples_.push_back(yaw_enu);
      tryGnssInit();
    }
    return;
  }

  if (init_state_ != InitState::RUNNING) return;

  // map 프레임 heading: ENU → map (datum_yaw 회전 적용)
  double yaw_map = yaw_enu - datum_yaw_;
  while (yaw_map >  M_PI) yaw_map -= 2.0 * M_PI;
  while (yaw_map < -M_PI) yaw_map += 2.0 * M_PI;

  //  innovation 게이트 (map 프레임 기준) 
  if (heading_received_ && last_lio_)
  {
    const double yaw_odom = quaternionToYaw(last_lio_->pose.pose.orientation);
    double innov = yaw_map - (yaw_odom + drift_(2));
    while (innov >  M_PI) innov -= 2.0 * M_PI;
    while (innov < -M_PI) innov += 2.0 * M_PI;

    RCLCPP_INFO(get_logger(),
      "HDG raw=%.4f enu=%.4f map=%.4f | LIO=%.4f drift_yaw=%.4f | innov=%.4f [%s]",
      yaw_raw, yaw_enu, yaw_map, yaw_odom, drift_(2), innov,
      std::abs(innov) > max_heading_innovation_ ? "REJECT" : "ACCEPT");

    if (std::abs(innov) > max_heading_innovation_)
    {
      return;
    }
  }

  last_heading_yaw_ = yaw_enu;  // ENU heading 저장
  heading_received_ = true;

  if (rtk_quality_ == RtkQuality::NONE) return;
  if (enable_noise_sim_ && in_shadow_)  return;

  const double r_yaw = (rtk_quality_ == RtkQuality::FIX)   ? yaw_r_ :
                       (rtk_quality_ == RtkQuality::FLOAT)  ? yaw_r_ * 4.0 :
                                                               yaw_r_ * 16.0;
  ekfUpdateHeading(yaw_map, r_yaw);
}

// GNSS 초기화: 샘플 안정성 확인 후 drift_ 초기화
void GnssCorrectorNode::tryGnssInit()
{
  if (!lio_received_ || !last_lio_) return;

  const std::size_t max_buf = static_cast<std::size_t>(gnss_init_samples_) * 3;
  if (init_e_samples_.size() > max_buf)
  {
    init_e_samples_.erase(init_e_samples_.begin());
    init_n_samples_.erase(init_n_samples_.begin());
  }
  if (init_yaw_samples_.size() > max_buf)
  {
    init_yaw_samples_.erase(init_yaw_samples_.begin());
  }

  if (static_cast<int>(init_e_samples_.size())   < gnss_init_samples_ ||
      static_cast<int>(init_yaw_samples_.size()) < gnss_init_samples_)
  {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
      "Init collecting: pos=%zu/%d  yaw=%zu/%d (RTK FIX required for pos)",
      init_e_samples_.size(), gnss_init_samples_,
      init_yaw_samples_.size(), gnss_init_samples_);
    return;
  }

  // 위치 안정성 확인 (map 프레임)
  const double e_mean = sampleMean(init_e_samples_);
  const double n_mean = sampleMean(init_n_samples_);
  const double e_std  = sampleStd(init_e_samples_, e_mean);
  const double n_std  = sampleStd(init_n_samples_, n_mean);

  if (e_std > gnss_init_pos_std_ || n_std > gnss_init_pos_std_)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Init: position not stable (e_std=%.3fm  n_std=%.3fm  limit=%.3fm). Keep robot stationary.",
      e_std, n_std, gnss_init_pos_std_);
    init_e_samples_.clear();
    init_n_samples_.clear();
    return;
  }

  // heading 안정성 확인 (ENU 기준 수집, map 프레임으로 변환)
  const double yaw_mean_enu = circularMean(init_yaw_samples_);
  const double yaw_std      = circularStd(init_yaw_samples_, yaw_mean_enu);

  if (yaw_std > gnss_init_yaw_std_)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Init: heading not stable (yaw_std=%.4f rad  limit=%.4f rad). Keep robot stationary.",
      yaw_std, gnss_init_yaw_std_);
    init_yaw_samples_.clear();
    return;
  }

  // map 프레임 heading으로 변환
  double yaw_mean = yaw_mean_enu - datum_yaw_;
  while (yaw_mean >  M_PI) yaw_mean -= 2.0 * M_PI;
  while (yaw_mean < -M_PI) yaw_mean += 2.0 * M_PI;

  //  두 조건 충족: drift_ 초기화 
  const double x_odom   = last_lio_->pose.pose.position.x;
  const double y_odom   = last_lio_->pose.pose.position.y;
  const double yaw_odom = quaternionToYaw(last_lio_->pose.pose.orientation);

  drift_(2) = yaw_mean - yaw_odom;
  while (drift_(2) >  M_PI) drift_(2) -= 2.0 * M_PI;
  while (drift_(2) < -M_PI) drift_(2) += 2.0 * M_PI;

  const double cd = std::cos(drift_(2));
  const double sd = std::sin(drift_(2));
  drift_(0) = e_mean - (cd * x_odom - sd * y_odom);
  drift_(1) = n_mean - (sd * x_odom + cd * y_odom);

  P_ = Eigen::Matrix3d::Zero();
  P_(0, 0) = pos_r_fix_;
  P_(1, 1) = pos_r_fix_;
  P_(2, 2) = yaw_r_;

  last_heading_yaw_ = yaw_mean_enu;
  heading_received_ = true;

  init_state_ = InitState::RUNNING;

  RCLCPP_INFO(get_logger(),
    "=== GNSS init complete ==="
    "\n  map origin : lat=%.9f  lon=%.9f  datum_yaw=%.4f rad"
    "\n  init pos   : mx=%.3f m  my=%.3f m  (std: e=%.3f  n=%.3f)"
    "\n  init yaw   : %.4f rad  (std: %.4f)"
    "\n  drift_     : [%.3f, %.3f, %.4f]",
    datum_lat_, datum_lon_, datum_yaw_,
    e_mean, n_mean, e_std, n_std,
    yaw_mean, yaw_std,
    drift_(0), drift_(1), drift_(2));
}

// EKF: predict (랜덤 워크 프로세스 모델)
void GnssCorrectorNode::ekfPredict(double dt)
{
  P_(0, 0) += pos_process_noise_ * dt;
  P_(1, 1) += pos_process_noise_ * dt;
  P_(2, 2) += yaw_process_noise_ * dt;
}

// EKF: 위치 측정 업데이트
// 반환값: true=수락, false=chi2 게이트 거부
bool GnssCorrectorNode::ekfUpdatePosition(
  double mx_meas, double my_meas, const Eigen::Matrix2d & R, double max_correction)
{
  if (!last_lio_) return false;

  const double x_b  = last_lio_->pose.pose.position.x;
  const double y_b  = last_lio_->pose.pose.position.y;
  const double dyaw = drift_(2);
  const double cd   = std::cos(dyaw);
  const double sd   = std::sin(dyaw);

  const double h_mx = drift_(0) + cd * x_b - sd * y_b;
  const double h_my = drift_(1) + sd * x_b + cd * y_b;

  Eigen::Vector2d z;
  z(0) = mx_meas - h_mx;
  z(1) = my_meas - h_my;

  Eigen::Matrix<double, 2, 3> H;
  H(0, 0) = 1.0;  H(0, 1) = 0.0;  H(0, 2) = -sd * x_b - cd * y_b;
  H(1, 0) = 0.0;  H(1, 1) = 1.0;  H(1, 2) =  cd * x_b - sd * y_b;

  const Eigen::Matrix2d S     = H * P_ * H.transpose() + R;
  const Eigen::Matrix2d S_inv = S.inverse();

  // Chi-squared Mahalanobis 게이트 (2-DOF, 99% CI = 9.21)
  const double chi2 = (z.transpose() * S_inv * z).value();
  if (chi2 > pos_innovation_gate_)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Pos REJECTED  chi2=%.2f > gate=%.2f  innov=[%.3f, %.3f]m  "
      "S_diag=[%.4f, %.4f]  reject_cnt=%d",
      chi2, pos_innovation_gate_, z(0), z(1),
      S(0, 0), S(1, 1), gnss_reject_count_ + 1);
    return false;
  }

  RCLCPP_DEBUG(get_logger(),
    "Pos ACCEPTED  chi2=%.2f  innov=[%.3f, %.3f]m  S_diag=[%.4f, %.4f]",
    chi2, z(0), z(1), S(0, 0), S(1, 1));

  const Eigen::Matrix<double, 3, 2> K = P_ * H.transpose() * S_inv;
  Eigen::Vector3d delta = K * z;

  const double pos_corr = delta.head<2>().norm();
  if (pos_corr > max_correction)
  {
    delta *= max_correction / pos_corr;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Position correction clipped: %.3f → %.3f m", pos_corr, max_correction);
  }
  if (std::abs(delta(2)) > max_yaw_correction_)
  {
    delta *= max_yaw_correction_ / std::abs(delta(2));
  }

  // relocalization 플래그: 1회 보정량 >= 임계값(기본 3cm)
  relocalization_ = (delta.head<2>().norm() >= relocalize_threshold_);

  drift_ += delta;
  while (drift_(2) >  M_PI) drift_(2) -= 2.0 * M_PI;
  while (drift_(2) < -M_PI) drift_(2) += 2.0 * M_PI;

  const Eigen::Matrix3d I_KH = Eigen::Matrix3d::Identity() - K * H;
  P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
  P_ = 0.5 * (P_ + P_.transpose());

  return true;
}

// EKF: heading 측정 업데이트
void GnssCorrectorNode::ekfUpdateHeading(double yaw_map_meas, double yaw_r)
{
  if (!last_lio_) return;

  const double yaw_odom = quaternionToYaw(last_lio_->pose.pose.orientation);

  double z = yaw_map_meas - (yaw_odom + drift_(2));
  while (z >  M_PI) z -= 2.0 * M_PI;
  while (z < -M_PI) z += 2.0 * M_PI;

  const Eigen::RowVector3d H(0.0, 0.0, 1.0);
  const double S = H * P_ * H.transpose() + yaw_r;
  const Eigen::Vector3d K = P_ * H.transpose() / S;

  Eigen::Vector3d delta = K * z;
  if (std::abs(delta(2)) > max_yaw_correction_)
  {
    delta *= max_yaw_correction_ / std::abs(delta(2));
  }

  drift_ += delta;
  while (drift_(2) >  M_PI) drift_(2) -= 2.0 * M_PI;
  while (drift_(2) < -M_PI) drift_(2) += 2.0 * M_PI;

  const Eigen::Matrix3d I_KH = Eigen::Matrix3d::Identity() - K * H;
  P_ = I_KH * P_ * I_KH.transpose() + K * (yaw_r * K.transpose());
  P_ = 0.5 * (P_ + P_.transpose());

  P_(0, 2) = P_(2, 0) = 0.0;
  P_(1, 2) = P_(2, 1) = 0.0;
}

// Datum 변경 후 EKF drift 재초기화
// 현재 GNSS(last_fix) + LIO pose로 즉시 단일 샘플 재계산
void GnssCorrectorNode::reinitDrift()
{
  if (!last_lio_ || !fix_received_ || !heading_received_)
  {
    RCLCPP_WARN(get_logger(), "reinitDrift: 데이터 부족 — P 확장 후 EKF 수렴 대기");
    P_ = Eigen::Matrix3d::Zero();
    P_(0, 0) = 100.0;
    P_(1, 1) = 100.0;
    P_(2, 2) = 0.1;
    return;
  }

  // 현재 GNSS를 새 datum 기준 map 좌표로 변환
  double mx_gnss, my_gnss;
  latLonToMap(last_fix_lat_, last_fix_lon_, mx_gnss, my_gnss);

  // 현재 heading을 map 프레임으로 변환
  double yaw_map = last_heading_yaw_ - datum_yaw_;
  while (yaw_map >  M_PI) yaw_map -= 2.0 * M_PI;
  while (yaw_map < -M_PI) yaw_map += 2.0 * M_PI;

  const double x_b    = last_lio_->pose.pose.position.x;
  const double y_b    = last_lio_->pose.pose.position.y;
  const double yaw_odom = quaternionToYaw(last_lio_->pose.pose.orientation);

  drift_(2) = yaw_map - yaw_odom;
  while (drift_(2) >  M_PI) drift_(2) -= 2.0 * M_PI;
  while (drift_(2) < -M_PI) drift_(2) += 2.0 * M_PI;

  const double cd = std::cos(drift_(2));
  const double sd = std::sin(drift_(2));
  drift_(0) = mx_gnss - (cd * x_b - sd * y_b);
  drift_(1) = my_gnss - (sd * x_b + cd * y_b);

  P_ = Eigen::Matrix3d::Zero();
  P_(0, 0) = pos_r_fix_;
  P_(1, 1) = pos_r_fix_;
  P_(2, 2) = yaw_r_;

  RCLCPP_INFO(get_logger(),
    "reinitDrift: datum=(%.9f, %.9f, %.4f) drift=[%.3f, %.3f, %.4f]",
    datum_lat_, datum_lon_, datum_yaw_,
    drift_(0), drift_(1), drift_(2));
}

// /odometry/map 발행(디버깅용)
void GnssCorrectorNode::publishMapOdom(const nav_msgs::msg::Odometry::SharedPtr & lio_msg)
{
  nav_msgs::msg::Odometry out = *lio_msg;
  out.header.frame_id = frame_map_;
  out.child_frame_id  = frame_body_;

  const double dx   = drift_(0);
  const double dy   = drift_(1);
  const double dyaw = drift_(2);
  const double cd   = std::cos(dyaw);
  const double sd   = std::sin(dyaw);

  const double x_o = lio_msg->pose.pose.position.x;
  const double y_o = lio_msg->pose.pose.position.y;
  out.pose.pose.position.x = cd * x_o - sd * y_o + dx;
  out.pose.pose.position.y = sd * x_o + cd * y_o + dy;
  out.pose.pose.position.z = lio_msg->pose.pose.position.z;

  tf2::Quaternion q_odom, q_drift;
  tf2::fromMsg(lio_msg->pose.pose.orientation, q_odom);
  q_drift.setRPY(0.0, 0.0, dyaw);
  out.pose.pose.orientation = tf2::toMsg((q_drift * q_odom).normalized());

  pub_odom_map_->publish(out);

  geometry_msgs::msg::PoseStamped ps;
  ps.header = out.header;
  ps.pose   = out.pose.pose;
  appendPath(path_map_, ps, PATH_MAX_POSES);
  path_map_.header.stamp = out.header.stamp;
  pub_path_map_->publish(path_map_);

  last_map_x_    = out.pose.pose.position.x;
  last_map_y_    = out.pose.pose.position.y;
  last_map_valid_ = true;
}

// /odometry/gnss 발행 (디버깅용)
void GnssCorrectorNode::publishGnssOdom(
  double mx, double my, double r_pos, const rclcpp::Time & stamp)
{
  nav_msgs::msg::Odometry gnss_odom;
  gnss_odom.header.stamp    = stamp;
  gnss_odom.header.frame_id = frame_map_;
  gnss_odom.child_frame_id  = "gps";

  gnss_odom.pose.pose.position.x = mx;
  gnss_odom.pose.pose.position.y = my;
  gnss_odom.pose.pose.position.z = 0.0;
  gnss_odom.pose.pose.orientation.w = 1.0;
  gnss_odom.pose.covariance[0]  = r_pos;
  gnss_odom.pose.covariance[7]  = r_pos;
  gnss_odom.pose.covariance[14] = 9999.0;

  pub_gnss_odom_->publish(gnss_odom);

  geometry_msgs::msg::PoseStamped ps;
  ps.header = gnss_odom.header;
  ps.pose   = gnss_odom.pose.pose;
  appendPath(path_gnss_, ps, PATH_MAX_POSES);
  path_gnss_.header.stamp = stamp;
  pub_path_gnss_->publish(path_gnss_);
}

// /lpc/localization/pose_update 발행 (NCU 인터페이스, 10 Hz)
// 출력 기준: base_link (차량 중심)
//
// [왜 body(IMU) 위치가 아닌 base_link 위치를 출력하는가]
//   Fast-LIO /Odometry 는 body(IMU 센서) 프레임의 위치를 추적한다.
//   NCU/제어계는 차량 중심(base_link)을 기준으로 경로 계획 및 제어를 수행하므로,
//   IMU 설치 위치가 아닌 차량 중심 좌표를 출력해야 한다.
//
// [body → base_link 오프셋 변환 방법]
//   Step 1. body 프레임 기준 오프셋 벡터 d = (body_to_base_x, body_to_base_y, body_to_base_z)
//           (yaml: body_to_base_link_x/y/z — USDA 실측값으로 설정)
//   Step 2. 차량의 map 프레임 기준 방위각(yaw_map) 계산:
//           yaw_map = LIO odom yaw + drift_yaw
//   Step 3. d를 yaw_map으로 map 프레임 회전 후 body 위치에 가산:
//           Δx_map = cos(yaw_map)*d_x - sin(yaw_map)*d_y
//           Δy_map = sin(yaw_map)*d_x + cos(yaw_map)*d_y
//
// [orientation은 그대로 사용하는 이유]
//   body 프레임은 ROS 표준(x=전방, y=좌방)이므로 yaw = 차량 전방 방위각.
//   base_link 위치로 보정해도 차량 헤딩 자체는 동일하므로 orientation 수정 불필요.
void GnssCorrectorNode::publishPoseUpdate(const nav_msgs::msg::Odometry::SharedPtr & lio_msg)
{
  rtk_lio::msg::PoseUpdate pu;
  pu.header.stamp    = lio_msg->header.stamp;
  pu.header.frame_id = frame_map_;

  const double dx   = drift_(0);
  const double dy   = drift_(1);
  const double dyaw = drift_(2);
  const double cd   = std::cos(dyaw);
  const double sd   = std::sin(dyaw);

  const double x_o = lio_msg->pose.pose.position.x;
  const double y_o = lio_msg->pose.pose.position.y;

  // body(IMU) 위치 → map 프레임
  double px = cd * x_o - sd * y_o + dx;
  double py = sd * x_o + cd * y_o + dy;
  double pz = lio_msg->pose.pose.position.z;

  // body → base_link 오프셋을 map 프레임으로 회전 후 적용
  const double yaw_map = quaternionToYaw(lio_msg->pose.pose.orientation) + dyaw;
  const double cm = std::cos(yaw_map);
  const double sm = std::sin(yaw_map);
  px += cm * body_to_base_x_ - sm * body_to_base_y_;
  py += sm * body_to_base_x_ + cm * body_to_base_y_;
  pz += body_to_base_z_;

  pu.pose.position.x = px;
  pu.pose.position.y = py;
  pu.pose.position.z = pz;

  tf2::Quaternion q_odom, q_drift;
  tf2::fromMsg(lio_msg->pose.pose.orientation, q_odom);
  q_drift.setRPY(0.0, 0.0, dyaw);
  pu.pose.orientation = tf2::toMsg((q_drift * q_odom).normalized());

  pu.quality        = quality_;
  pu.relocalization = relocalization_;

  pub_pose_update_->publish(pu);
}

// /lpc/diagnostics 발행 (1 Hz timer)
void GnssCorrectorNode::publishDiagnostics()
{
  using DS  = diagnostic_msgs::msg::DiagnosticStatus;
  using KV  = diagnostic_msgs::msg::KeyValue;

  auto make_kv = [](const std::string & k, const std::string & v)
  {
    KV kv;
    kv.key   = k;
    kv.value = v;
    return kv;
  };

  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp    = this->now();
  arr.header.frame_id = "";

  // LPC/System/StateMachine
  {
    const SystemState    ss  = computeSystemState();
    const SystemSubstate sub = computeSystemSubstate();
    uint8_t level = DS::OK;
    if (ss == SystemState::ERROR)         level = DS::ERROR;
    else if (ss == SystemState::DEGRADED ||
             ss == SystemState::INITIALIZING) level = DS::WARN;
    if (shutting_down_)                   level = DS::STALE;

    DS s;
    s.name       = "LPC/System/StateMachine";
    s.level      = level;
    s.message    = std::string(systemStateStr(ss)) + "/" + systemSubstateStr(sub);
    s.hardware_id = "";
    s.values.push_back(make_kv("state",    systemStateStr(ss)));
    s.values.push_back(make_kv("substate", systemSubstateStr(sub)));
    s.values.push_back(make_kv("quality",  std::to_string(quality_)));
    arr.status.push_back(s);
  }

  // LPC/Algorithm/GNSS_Corrector
  {
    const std::string fusion_mode =
      (rtk_quality_ == RtkQuality::FIX)    ? "RTK_FIX"    :
      (rtk_quality_ == RtkQuality::FLOAT)   ? "RTK_FLOAT"  :
      (rtk_quality_ == RtkQuality::SINGLE)  ? "GNSS_SINGLE" : "NONE";

    DS s;
    s.name       = "LPC/Algorithm/GNSS_Corrector";
    s.level      = gnss_frozen_ ? DS::WARN : DS::OK;
    s.message    = gnss_frozen_ ? "GNSS frozen — LiDAR only" : "GNSS fusion active";
    s.hardware_id = "";
    s.values.push_back(make_kv("gnss_frozen",  gnss_frozen_ ? "true" : "false"));
    s.values.push_back(make_kv("fusion_mode",  fusion_mode));
    s.values.push_back(make_kv("quality",      std::to_string(quality_)));
    s.values.push_back(make_kv("relocalization", relocalization_ ? "true" : "false"));
    arr.status.push_back(s);
  }

  // LPC/LiDAR/OS1-32_Front, Rear
  {
    const double age_ms = lio_received_ ?
      (this->now() - last_lio_stamp_).seconds() * 1000.0 : 99999.0;
    const uint8_t level = (age_ms > lidar_timeout_s_ * 1000.0) ? DS::ERROR :
                          (age_ms > 500.0)                      ? DS::WARN  : DS::OK;
    const std::string age_str = std::to_string(static_cast<int>(age_ms));
    const std::string msg_str = (level == DS::ERROR) ? "LiDAR packet timeout" :
                                (level == DS::WARN)  ? "LiDAR packet delay"   : "OK";

    for (const auto & name :
         {std::string("LPC/LiDAR/OS1-32_Front"), std::string("LPC/LiDAR/OS1-32_Rear")})
    {
      DS s;
      s.name       = name;
      s.level      = level;
      s.message    = msg_str;
      s.hardware_id = "";
      s.values.push_back(make_kv("last_packet_age_ms", age_str));
      arr.status.push_back(s);
    }
  }

  // LPC/GNSS/RTK_Receiver
  {
    const std::string fix_type =
      (rtk_quality_ == RtkQuality::FIX)    ? "RTK_FIX" :
      (rtk_quality_ == RtkQuality::FLOAT)   ? "FLOAT"   :
      (rtk_quality_ == RtkQuality::SINGLE)  ? "SINGLE"  : "NO_FIX";
    const uint8_t level =
      (rtk_quality_ == RtkQuality::NONE)  ? DS::ERROR :
      (rtk_quality_ != RtkQuality::FIX)   ? DS::WARN  : DS::OK;

    DS s;
    s.name       = "LPC/GNSS/RTK_Receiver";
    s.level      = level;
    s.message    = "RTK fix type: " + fix_type;
    s.hardware_id = "";
    s.values.push_back(make_kv("fix_type",       fix_type));
    s.values.push_back(make_kv("shadow_detected",
      (enable_noise_sim_ && in_shadow_) ? "true" : "false"));
    arr.status.push_back(s);
  }

  // LPC/Network/NCU_Link
  {
    const double contact_age_s = ncu_contacted_ ?
      (this->now() - last_ncu_contact_time_).seconds() : 9999.0;
    const uint8_t level = (contact_age_s > 30.0) ? DS::WARN : DS::OK;

    DS s;
    s.name       = "LPC/Network/NCU_Link";
    s.level      = level;
    s.message    = (level == DS::WARN) ? "NCU contact timeout" : "OK";
    s.hardware_id = "";
    s.values.push_back(make_kv("latency_ms",      "0"));
    s.values.push_back(make_kv("packet_loss_pct", "0"));
    arr.status.push_back(s);
  }

  pub_diagnostics_->publish(arr);
}

// map → odom TF 브로드캐스트
void GnssCorrectorNode::broadcastMapOdomTf(const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp    = stamp;
  tf_msg.header.frame_id = frame_map_;
  tf_msg.child_frame_id  = frame_odom_;

  tf_msg.transform.translation.x = drift_(0);
  tf_msg.transform.translation.y = drift_(1);
  tf_msg.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, drift_(2));
  tf_msg.transform.rotation = tf2::toMsg(q);

  tf_broadcaster_->sendTransform(tf_msg);
}

// Path ring-buffer 추가
void GnssCorrectorNode::appendPath(
  nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & pose,
  std::size_t max_poses)
{
  path.poses.push_back(pose);
  if (path.poses.size() > max_poses)
  {
    path.poses.erase(path.poses.begin());
  }
}

// set_origin 서비스 핸들러
// AGV가 기준 마킹 위에 있을 때 1회 호출.
// 현재 GNSS 위치를 NCU가 지정한 map 좌표로 매핑하여 datum 확정
void GnssCorrectorNode::handleSetOrigin(
  const std::shared_ptr<rtk_lio::srv::SetOrigin::Request>  req,
  std::shared_ptr<rtk_lio::srv::SetOrigin::Response>        res)
{
  last_ncu_contact_time_ = this->now();
  ncu_contacted_         = true;

  //  거부 조건 확인 
  if (init_state_ != InitState::RUNNING)
  {
    res->success = false;
    res->message = "GnssCorrectorNode not RUNNING (still initializing)";
    RCLCPP_WARN(get_logger(), "set_origin rejected: %s", res->message.c_str());
    return;
  }
  if (rtk_quality_ != RtkQuality::FIX)
  {
    res->success = false;
    res->message = "RTK FIX required for set_origin (current: " +
                   std::string(rtk_quality_ == RtkQuality::FLOAT ? "FLOAT" :
                               rtk_quality_ == RtkQuality::SINGLE ? "SINGLE" : "NONE") + ")";
    RCLCPP_WARN(get_logger(), "set_origin rejected: %s", res->message.c_str());
    return;
  }
  if (!fix_received_ || !heading_received_)
  {
    res->success = false;
    res->message = "GNSS data not yet received";
    RCLCPP_WARN(get_logger(), "set_origin rejected: %s", res->message.c_str());
    return;
  }

  // datum_yaw 계산
  // map heading [deg, 0~360] → rad
  const double map_heading_rad = req->map_heading * DEG2RAD;

  // datum_yaw = ENU heading - map heading
  double datum_yaw_new = last_heading_yaw_ - map_heading_rad;
  while (datum_yaw_new >  M_PI) datum_yaw_new -= 2.0 * M_PI;
  while (datum_yaw_new < -M_PI) datum_yaw_new += 2.0 * M_PI;

  // 새 datum origin 계산
  // map_x, map_y는 base_link(차량 중심) 기준이지만, datum은 body(IMU) 위치를 기준으로 계산한다.
  // publishPoseUpdate에서 body → base_link 오프셋이 추가되므로,
  // datum 계산 시에는 요청 좌표에서 body_to_base_link 오프셋을 역으로 빼야
  // 최종 출력이 (map_x, map_y)와 일치한다.
  // vehicle heading in map frame = map_heading_rad (사용자가 지정한 값)
  const double cm_h = std::cos(map_heading_rad);
  const double sm_h = std::sin(map_heading_rad);
  const double body_target_x = req->map_x - (cm_h * body_to_base_x_ - sm_h * body_to_base_y_);
  const double body_target_y = req->map_y - (sm_h * body_to_base_x_ + cm_h * body_to_base_y_);

  // 기준점 ENU 위치 (새 datum 기준): R(datum_yaw) * (body_target_x, body_target_y)
  const double cd = std::cos(datum_yaw_new);
  const double sd = std::sin(datum_yaw_new);
  const double e_ref = cd * body_target_x - sd * body_target_y;
  const double n_ref = sd * body_target_x + cd * body_target_y;

  // 새 datum origin = 현재 GNSS 위치에서 (-e_ref, -n_ref) 오프셋
  double new_datum_lat, new_datum_lon;
  enuFromRefToLatLon(last_fix_lat_, last_fix_lon_,
                      -e_ref, -n_ref,
                      new_datum_lat, new_datum_lon);

  // datum 적용
  datum_lat_ = new_datum_lat;
  datum_lon_ = new_datum_lon;
  datum_yaw_ = datum_yaw_new;

  // EKF 재초기화
  reinitDrift();

  // datum 파일 저장
  saveDatumToFile(req->map_x, req->map_y, req->map_heading);

  // 응답
  res->success     = true;
  res->map_x       = req->map_x;
  res->map_y       = req->map_y;
  res->map_heading = req->map_heading;
  res->datum_lat   = datum_lat_;
  res->datum_lon   = datum_lon_;
  res->datum_yaw   = datum_yaw_;
  res->message     = "OK";

  RCLCPP_INFO(get_logger(),
    "=== set_origin 완료 ==="
    "\n  map=(%+.3f, %+.3f)  heading=%.1f deg"
    "\n  datum: lat=%.9f  lon=%.9f  yaw=%.4f rad"
    "\n  saved: %s",
    req->map_x, req->map_y, req->map_heading,
    datum_lat_, datum_lon_, datum_yaw_,
    datum_file_path_.c_str());
}

// set_datum 서비스 핸들러
// 저장된 datum 값을 주입하여 map 원점 복원. 매 기동 시 호출.
// AGV는 어느 위치에 있어도 무방하며, EKF가 현재 GNSS로 수렴함.
void GnssCorrectorNode::handleSetDatum(
  const std::shared_ptr<rtk_lio::srv::SetDatum::Request>  req,
  std::shared_ptr<rtk_lio::srv::SetDatum::Response>        res)
{
  last_ncu_contact_time_ = this->now();
  ncu_contacted_         = true;

  // 거부 조건 확인
  if (std::abs(req->datum_lat) < 1e-6 && std::abs(req->datum_lon) < 1e-6)
  {
    res->success = false;
    res->message = "Invalid datum: lat/lon both zero";
    RCLCPP_WARN(get_logger(), "set_datum rejected: %s", res->message.c_str());
    return;
  }
  if (req->datum_lat < -90.0 || req->datum_lat > 90.0 ||
      req->datum_lon < -180.0 || req->datum_lon > 180.0)
  {
    res->success = false;
    res->message = "datum_lat/datum_lon out of valid range";
    RCLCPP_WARN(get_logger(), "set_datum rejected: %s", res->message.c_str());
    return;
  }
  if (init_state_ != InitState::RUNNING)
  {
    res->success = false;
    res->message = "GnssCorrectorNode not RUNNING (still initializing)";
    RCLCPP_WARN(get_logger(), "set_datum rejected: %s", res->message.c_str());
    return;
  }
  if (rtk_quality_ != RtkQuality::FIX)
  {
    res->success = false;
    res->message = "RTK FIX required for set_datum (EKF convergence needs accurate GNSS)";
    RCLCPP_WARN(get_logger(), "set_datum rejected: %s", res->message.c_str());
    return;
  }

  // datum 주입
  datum_lat_ = req->datum_lat;
  datum_lon_ = req->datum_lon;
  datum_yaw_ = req->datum_yaw;

  //  EKF 재초기화 (현재 위치 GNSS 기준) 
  reinitDrift();

  //  응답 
  res->success     = true;
  res->map_x       = req->map_x;
  res->map_y       = req->map_y;
  res->map_heading = req->map_heading;
  res->datum_lat   = datum_lat_;
  res->datum_lon   = datum_lon_;
  res->datum_yaw   = datum_yaw_;
  res->message     = "OK";

  RCLCPP_INFO(get_logger(),
    "=== set_datum 완료 ==="
    "\n  datum: lat=%.9f  lon=%.9f  yaw=%.4f rad"
    "\n  현재 위치에서 EKF 수렴 시작 (quality=%d)",
    datum_lat_, datum_lon_, datum_yaw_, quality_);
}

// /ncu/system/key_off 구독 콜백 — 안전 종료 준비
void GnssCorrectorNode::onKeyOff(const std_msgs::msg::Empty::SharedPtr)
{
  RCLCPP_WARN(get_logger(), "=== KEY OFF 수신 === 안전 종료 준비 중...");
  shutting_down_ = true;

  saveStateToFile();

  // STALE 상태로 diagnostics 즉시 발행
  publishDiagnostics();

  RCLCPP_INFO(get_logger(), "KEY OFF 처리 완료. PDU OFF 대기.");
}

// datum 파일 저장 (set_origin 성공 시)
bool GnssCorrectorNode::saveDatumToFile(
  double map_x, double map_y, double map_heading) const
{
  std::ofstream f(datum_file_path_);
  if (!f.is_open())
  {
    RCLCPP_WARN(get_logger(), "datum 파일 저장 실패: %s", datum_file_path_.c_str());
    return false;
  }
  f << std::fixed << std::setprecision(9);
  f << "datum_lat:   " << datum_lat_   << "\n";
  f << "datum_lon:   " << datum_lon_   << "\n";
  f << std::setprecision(6);
  f << "datum_yaw:   " << datum_yaw_   << "\n";
  f << "map_x:       " << map_x        << "\n";
  f << "map_y:       " << map_y        << "\n";
  f << "map_heading: " << map_heading  << "\n";
  RCLCPP_INFO(get_logger(), "datum 저장 완료: %s", datum_file_path_.c_str());
  return true;
}

// 종료 상태 파일 저장 (key_off 수신 시)
void GnssCorrectorNode::saveStateToFile() const
{
  std::ofstream f(state_file_path_);
  if (!f.is_open())
  {
    RCLCPP_WARN(get_logger(), "state 파일 저장 실패: %s", state_file_path_.c_str());
    return;
  }
  f << std::fixed << std::setprecision(9);
  f << "datum_lat:  " << datum_lat_  << "\n";
  f << "datum_lon:  " << datum_lon_  << "\n";
  f << std::setprecision(6);
  f << "datum_yaw:  " << datum_yaw_  << "\n";
  f << "drift_x:    " << drift_(0)   << "\n";
  f << "drift_y:    " << drift_(1)   << "\n";
  f << "drift_yaw:  " << drift_(2)   << "\n";
  f << "quality:    " << static_cast<int>(quality_) << "\n";
  RCLCPP_INFO(get_logger(), "state 저장 완료: %s", state_file_path_.c_str());
}

// quality 0~5 계산
uint8_t GnssCorrectorNode::computeQuality() const
{
  // 0: 초기화 미완료 또는 LiDAR 없음
  if (!lio_received_ || init_state_ != InitState::RUNNING)
  {
    return 0;
  }

  // LiDAR timeout → 0
  // last_lio_recv_time_: 수신 시점의 노드 클럭 (use_sim_time 무관, 클럭 소스 불일치 방지)
  const double lio_age_s = (this->now() - last_lio_recv_time_).seconds();
  if (lio_age_s > lidar_timeout_s_)
  {
    return 0;
  }

  // 1: GNSS 없음, LiDAR 단독
  if (rtk_quality_ == RtkQuality::NONE)
  {
    return 1;
  }

  // Freeze 상태 구분
  if (gnss_frozen_)
  {
    if (gnss_freeze_time_valid_)
    {
      const double freeze_s = (this->now() - gnss_freeze_start_time_).seconds();
      // 2: Freeze >= 60s
      if (freeze_s >= 60.0) return 2;
    }
    // 3: Freeze < 60s
    return 3;
  }

  // 3: RTK FLOAT 또는 SINGLE
  if (rtk_quality_ == RtkQuality::FLOAT || rtk_quality_ == RtkQuality::SINGLE)
  {
    return 3;
  }

  // RTK FIX — 수렴 여부로 4/5 구분
  const double p_trace = P_(0, 0) + P_(1, 1);
  if (p_trace < pos_converge_threshold_)
  {
    // 5: RTK FIX + drift 수렴 완료
    return 5;
  }

  // 4: RTK FIX + drift 보정 수렴 중
  return 4;
}

// NCU 노출 시스템 상태 계산
GnssCorrectorNode::SystemState GnssCorrectorNode::computeSystemState() const
{
  if (shutting_down_)
  {
    return SystemState::SHUTTING_DOWN;
  }

  // LiDAR timeout → ERROR
  if (lio_received_)
  {
    const double age_s = (this->now() - last_lio_stamp_).seconds();
    if (age_s > lidar_timeout_s_)
    {
      return SystemState::ERROR;
    }
  }

  if (init_state_ != InitState::RUNNING)
  {
    return SystemState::INITIALIZING;
  }

  if (rtk_quality_ == RtkQuality::NONE || gnss_frozen_)
  {
    return SystemState::DEGRADED;
  }

  return SystemState::LOCALIZED;
}

GnssCorrectorNode::SystemSubstate GnssCorrectorNode::computeSystemSubstate() const
{
  switch (computeSystemState())
  {
    case SystemState::INITIALIZING:   return SystemSubstate::GNSS_WAIT;
    case SystemState::LOCALIZED:      return SystemSubstate::TRACKING;
    case SystemState::DEGRADED:       return SystemSubstate::LIO_ONLY;
    case SystemState::ERROR:          return SystemSubstate::SENSOR_FAULT;
    case SystemState::SHUTTING_DOWN:  return SystemSubstate::LOG_SYNC;
  }
  return SystemSubstate::GNSS_WAIT;
}

const char * GnssCorrectorNode::systemStateStr(SystemState s)
{
  switch (s)
  {
    case SystemState::INITIALIZING:   return "INITIALIZING";
    case SystemState::LOCALIZED:      return "LOCALIZED";
    case SystemState::DEGRADED:       return "DEGRADED";
    case SystemState::ERROR:          return "ERROR";
    case SystemState::SHUTTING_DOWN:  return "SHUTTING_DOWN";
  }
  return "UNKNOWN";
}

const char * GnssCorrectorNode::systemSubstateStr(SystemSubstate s)
{
  switch (s)
  {
    case SystemSubstate::GNSS_WAIT:    return "GNSS_WAIT";
    case SystemSubstate::TRACKING:     return "TRACKING";
    case SystemSubstate::LIO_ONLY:     return "LIO_ONLY";
    case SystemSubstate::SENSOR_FAULT: return "SENSOR_FAULT";
    case SystemSubstate::LOG_SYNC:     return "LOG_SYNC";
  }
  return "UNKNOWN";
}

// lat/lon → ENU (datum_lat/lon 기준)
void GnssCorrectorNode::latLonToEnu(
  double lat, double lon, double & e, double & n) const
{
  const double lat0_rad = datum_lat_ * DEG2RAD;
  const double sin_l0   = std::sin(lat0_rad);
  const double denom    = std::sqrt(1.0 - WGS84_E2 * sin_l0 * sin_l0);
  const double N = WGS84_A / denom;
  const double M = WGS84_A * (1.0 - WGS84_E2) / (denom * denom * denom);

  n = M * (lat - datum_lat_) * DEG2RAD;
  e = N * std::cos(lat0_rad) * (lon - datum_lon_) * DEG2RAD;
}

// lat/lon → map 프레임 좌표
// ENU 변환 후 datum_yaw 회전 적용.
// datum_yaw=0이면 latLonToEnu와 동일.
void GnssCorrectorNode::latLonToMap(
  double lat, double lon, double & mx, double & my) const
{
  double e, n;
  latLonToEnu(lat, lon, e, n);

  // p_map = R^T(datum_yaw) * p_enu
  const double cd = std::cos(datum_yaw_);
  const double sd = std::sin(datum_yaw_);
  mx =  cd * e + sd * n;
  my = -sd * e + cd * n;
}

// ENU 오프셋 → lat/lon 역변환
// (ref_lat, ref_lon) 기준으로 (e_offset, n_offset) 떨어진 위치의 좌표.
// set_origin에서 새 datum lat/lon 계산 시 사용.
void GnssCorrectorNode::enuFromRefToLatLon(
  double ref_lat, double ref_lon,
  double e_offset, double n_offset,
  double & lat, double & lon) const
{
  const double lat0_rad = ref_lat * DEG2RAD;
  const double sin_l0   = std::sin(lat0_rad);
  const double denom    = std::sqrt(1.0 - WGS84_E2 * sin_l0 * sin_l0);
  const double N = WGS84_A / denom;
  const double M = WGS84_A * (1.0 - WGS84_E2) / (denom * denom * denom);

  lat = ref_lat + n_offset / M / DEG2RAD;
  lon = ref_lon + e_offset / (N * std::cos(lat0_rad)) / DEG2RAD;
}

// Quaternion → yaw 추출
double GnssCorrectorNode::quaternionToYaw(
  const geometry_msgs::msg::Quaternion & q) const
{
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  return yaw;
}

// 통계 helper
double GnssCorrectorNode::sampleMean(const std::vector<double> & v)
{
  return std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size());
}

double GnssCorrectorNode::sampleStd(const std::vector<double> & v, double mean)
{
  double var = 0.0;
  for (double x : v) var += (x - mean) * (x - mean);
  return std::sqrt(var / static_cast<double>(v.size()));
}

double GnssCorrectorNode::circularMean(const std::vector<double> & angles)
{
  double s = 0.0, c = 0.0;
  for (double a : angles) { s += std::sin(a); c += std::cos(a); }
  return std::atan2(s, c);
}

double GnssCorrectorNode::circularStd(
  const std::vector<double> & angles, double mean)
{
  double var = 0.0;
  for (double a : angles)
  {
    double d = a - mean;
    while (d >  M_PI) d -= 2.0 * M_PI;
    while (d < -M_PI) d += 2.0 * M_PI;
    var += d * d;
  }
  return std::sqrt(var / static_cast<double>(angles.size()));
}

}  // namespace rtk_lio

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rtk_lio::GnssCorrectorNode>());
  rclcpp::shutdown();
  return 0;
}
