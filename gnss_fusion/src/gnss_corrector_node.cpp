#include "gnss_fusion/gnss_corrector_node.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cmath>
#include <algorithm>
#include <numeric>

namespace gnss_fusion
{

GnssCorrectorNode::GnssCorrectorNode()
: Node("gnss_corrector_node"),
  drift_(Eigen::Vector3d::Zero()),
  P_(Eigen::Matrix3d::Zero()),
  init_state_(InitState::WAIT_LIO),
  datum_lat_(0.0), datum_lon_(0.0),
  lio_received_(false),
  last_heading_yaw_(0.0),
  heading_received_(false),
  last_map_x_(0.0), last_map_y_(0.0), last_map_valid_(false),
  rtk_quality_(RtkQuality::NONE),
  in_multipath_(false), multipath_end_sim_s_(0.0),
  multipath_e_offset_(0.0), multipath_n_offset_(0.0),
  in_shadow_(false), shadow_end_sim_s_(0.0),
  rng_(std::random_device{}()),
  uniform01_(0.0, 1.0),
  uniform_angle_(0.0, 2.0 * M_PI)
{
  // ---- frame / topic parameters ----
  declare_parameter<std::string>("frame_map",      "map");
  declare_parameter<std::string>("frame_odom",     "odom");
  declare_parameter<std::string>("frame_body",     "body");
  declare_parameter<std::string>("topic_lio",      "/Odometry");
  declare_parameter<std::string>("topic_fix",      "/fix");
  declare_parameter<std::string>("topic_heading",  "/heading");

  // ---- Fixed datum (harbour absolute reference point) ----
  declare_parameter<double>("datum_lat", 0.0);
  declare_parameter<double>("datum_lon", 0.0);

  // ---- GPS antenna lever arm in body frame (x=forward, y=left) [m] ----
  declare_parameter<double>("antenna_offset_x", 0.0);
  declare_parameter<double>("antenna_offset_y", 0.0);

  // ---- EKF tuning ----
  declare_parameter<double>("yaw_offset",         3.20);
  declare_parameter<double>("pos_process_noise",  0.001);
  declare_parameter<double>("yaw_process_noise",  0.0001);
  declare_parameter<double>("pos_r_fix",          0.05);
  declare_parameter<double>("pos_r_float",        0.5);
  declare_parameter<double>("pos_r_single",       10.0);
  declare_parameter<double>("yaw_r",              0.01);
  declare_parameter<double>("max_pos_correction",       2.0);
  declare_parameter<double>("max_yaw_correction",       0.3);
  declare_parameter<double>("max_heading_innovation",   0.3);
  declare_parameter<bool>  ("use_rtk_covariance", false);
  declare_parameter<double>("restart_gap_s",      3.0);

  // ---- GNSS initialisation stability thresholds ----
  declare_parameter<int>   ("gnss_init_samples",  10);
  declare_parameter<double>("gnss_init_pos_std",   0.3);
  declare_parameter<double>("gnss_init_yaw_std",   0.02);

  // ---- noise simulation ----
  declare_parameter<bool>  ("enable_noise_sim",     false);
  declare_parameter<double>("multipath_prob",        0.05);
  declare_parameter<double>("multipath_magnitude",   8.0);
  declare_parameter<double>("multipath_duration_s",  4.0);
  declare_parameter<double>("shadow_prob",           0.03);
  declare_parameter<double>("shadow_duration_s",     10.0);

  frame_map_          = get_parameter("frame_map").as_string();
  frame_odom_         = get_parameter("frame_odom").as_string();
  frame_body_         = get_parameter("frame_body").as_string();
  topic_lio_          = get_parameter("topic_lio").as_string();
  topic_fix_          = get_parameter("topic_fix").as_string();
  topic_heading_      = get_parameter("topic_heading").as_string();
  datum_lat_          = get_parameter("datum_lat").as_double();
  datum_lon_          = get_parameter("datum_lon").as_double();
  yaw_offset_         = get_parameter("yaw_offset").as_double();
  pos_process_noise_  = get_parameter("pos_process_noise").as_double();
  yaw_process_noise_  = get_parameter("yaw_process_noise").as_double();
  pos_r_fix_          = get_parameter("pos_r_fix").as_double();
  pos_r_float_        = get_parameter("pos_r_float").as_double();
  pos_r_single_       = get_parameter("pos_r_single").as_double();
  yaw_r_              = get_parameter("yaw_r").as_double();
  max_pos_correction_      = get_parameter("max_pos_correction").as_double();
  max_yaw_correction_      = get_parameter("max_yaw_correction").as_double();
  max_heading_innovation_  = get_parameter("max_heading_innovation").as_double();
  use_rtk_covariance_      = get_parameter("use_rtk_covariance").as_bool();
  restart_gap_s_      = get_parameter("restart_gap_s").as_double();
  gnss_init_samples_  = get_parameter("gnss_init_samples").as_int();
  gnss_init_pos_std_  = get_parameter("gnss_init_pos_std").as_double();
  gnss_init_yaw_std_  = get_parameter("gnss_init_yaw_std").as_double();
  enable_noise_sim_   = get_parameter("enable_noise_sim").as_bool();
  multipath_prob_     = get_parameter("multipath_prob").as_double();
  multipath_magnitude_= get_parameter("multipath_magnitude").as_double();
  multipath_duration_s_= get_parameter("multipath_duration_s").as_double();
  shadow_prob_        = get_parameter("shadow_prob").as_double();
  shadow_duration_s_  = get_parameter("shadow_duration_s").as_double();
  antenna_offset_x_   = get_parameter("antenna_offset_x").as_double();
  antenna_offset_y_   = get_parameter("antenna_offset_y").as_double();

  // ---- subscriptions ----
  sub_lio_ = create_subscription<nav_msgs::msg::Odometry>(
    topic_lio_, rclcpp::SensorDataQoS(),
    std::bind(&GnssCorrectorNode::lioCallback, this, std::placeholders::_1));

  sub_fix_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    topic_fix_, rclcpp::SensorDataQoS(),
    std::bind(&GnssCorrectorNode::fixCallback, this, std::placeholders::_1));

  sub_heading_ = create_subscription<geometry_msgs::msg::QuaternionStamped>(
    topic_heading_, rclcpp::SensorDataQoS(),
    std::bind(&GnssCorrectorNode::headingCallback, this, std::placeholders::_1));

  // ---- publishers ----
  pub_odom_map_  = create_publisher<nav_msgs::msg::Odometry>("/odometry/map",  rclcpp::QoS(1000));
  pub_gnss_odom_ = create_publisher<nav_msgs::msg::Odometry>("/odometry/gnss", rclcpp::QoS(1000));
  pub_path_map_  = create_publisher<nav_msgs::msg::Path>("/path/map",  rclcpp::QoS(10));
  pub_path_gnss_ = create_publisher<nav_msgs::msg::Path>("/path/gnss", rclcpp::QoS(10));

  path_map_.header.frame_id  = frame_map_;
  path_gnss_.header.frame_id = frame_map_;

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  RCLCPP_INFO(get_logger(),
    "GnssCorrectorNode started. datum=(%.9f, %.9f) yaw_offset=%.4f noise_sim=%s",
    datum_lat_, datum_lon_, yaw_offset_, enable_noise_sim_ ? "ON" : "off");
  RCLCPP_INFO(get_logger(),
    "Waiting for LIO... (init: %d samples, pos_std<%.2fm, yaw_std<%.3frad)",
    gnss_init_samples_, gnss_init_pos_std_, gnss_init_yaw_std_);
}

// ============================================================
// LIO callback — predict + publish /odometry/map
// ============================================================
void GnssCorrectorNode::lioCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const rclcpp::Time now = msg->header.stamp;

  if (lio_received_ && init_state_ == InitState::RUNNING)
  {
    double dt = (now - last_lio_stamp_).seconds();

    if (dt > restart_gap_s_)
    {
      RCLCPP_WARN(get_logger(),
        "LIO restart detected (dt=%.2fs). Seamless re-init.", dt);
      if (last_map_valid_)
      {
        drift_(0) = last_map_x_;
        drift_(1) = last_map_y_;
      }
      else
      {
        drift_ = Eigen::Vector3d::Zero();
      }
      // Inflate P to reflect re-initialisation uncertainty (GPS will re-converge)
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
  last_lio_stamp_ = now;

  if (!lio_received_)
  {
    lio_received_ = true;
    init_state_   = InitState::WAIT_GNSS_STABLE;
    RCLCPP_INFO(get_logger(),
      "LIO received. Waiting for stable GNSS to initialise drift...");
  }

  if (init_state_ == InitState::RUNNING)
  {
    publishMapOdom(msg);
    broadcastMapOdomTf(now);
  }
}

// ============================================================
// GPS fix callback — init sample collection or EKF update
// ============================================================
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

  double e_raw, n_raw;
  latLonToEnu(msg->latitude, msg->longitude, e_raw, n_raw);

  // ---- Lever arm: convert antenna position → body frame origin ----
  if (last_lio_ && (antenna_offset_x_ != 0.0 || antenna_offset_y_ != 0.0))
  {
    const double dyaw_est = (init_state_ == InitState::RUNNING) ? drift_(2) : 0.0;
    const double theta    = quaternionToYaw(last_lio_->pose.pose.orientation) + dyaw_est;
    const double cb = std::cos(theta);
    const double sb = std::sin(theta);
    e_raw -= cb * antenna_offset_x_ - sb * antenna_offset_y_;
    n_raw -= sb * antenna_offset_x_ + cb * antenna_offset_y_;
  }

  // ---- Initialisation phase: collect position samples ----
  if (init_state_ == InitState::WAIT_GNSS_STABLE)
  {
    if (rtk_quality_ == RtkQuality::FIX)
    {
      init_e_samples_.push_back(e_raw);
      init_n_samples_.push_back(n_raw);
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

  // ---- Normal EKF update ----
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

  const double sim_s = msg->header.stamp.sec +
                       msg->header.stamp.nanosec * 1e-9;

  double e_noisy = e_raw;
  double n_noisy = n_raw;
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
      in_shadow_       = true;
      double dur       = shadow_duration_s_ * (0.5 + 0.5 * uniform01_(rng_));
      shadow_end_sim_s_ = sim_s + dur;
      skip_ekf  = true;
      is_shadow = true;
      RCLCPP_WARN(get_logger(),
        "[NOISE] GNSS shadow START  duration=%.1fs", dur);
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
        in_multipath_       = true;
        double dur          = multipath_duration_s_ * (0.5 + 0.5 * uniform01_(rng_));
        multipath_end_sim_s_ = sim_s + dur;
        double angle        = uniform_angle_(rng_);
        double mag          = multipath_magnitude_ * (0.5 + 0.5 * uniform01_(rng_));
        multipath_e_offset_ = mag * std::cos(angle);
        multipath_n_offset_ = mag * std::sin(angle);
        RCLCPP_WARN(get_logger(),
          "[NOISE] Multipath START  offset=(%.1f, %.1f)m  duration=%.1fs",
          multipath_e_offset_, multipath_n_offset_, dur);
      }

      if (in_multipath_)
      {
        e_noisy += multipath_e_offset_;
        n_noisy += multipath_n_offset_;
        r_pos = std::max(r_pos, multipath_magnitude_ * multipath_magnitude_);
      }
    }
  }

  if (!is_shadow)
  {
    publishGnssOdom(e_noisy, n_noisy, r_pos, msg->header.stamp);
  }

  if (!skip_ekf)
  {
    Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
    R(0, 0) = r_pos;
    R(1, 1) = r_pos;
    ekfUpdatePosition(e_noisy, n_noisy, R);
  }

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
    "GPS[%s%s] e=%.2f n=%.2f | drift=[%.3f, %.3f, %.4f]",
    (rtk_quality_ == RtkQuality::FIX)   ? "FIX" :
    (rtk_quality_ == RtkQuality::FLOAT) ? "FLOAT" : "SGL",
    is_shadow ? "+SHADOW" : (in_multipath_ ? "+MULTI" : ""),
    e_noisy, n_noisy, drift_(0), drift_(1), drift_(2));
}

// ============================================================
// GPS heading callback — init sample collection or EKF update
// ============================================================
void GnssCorrectorNode::headingCallback(
  const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
{
  double yaw_raw = quaternionToYaw(msg->quaternion);
  // GPS heading is in CW-from-North (compass/NED) convention; negate to get CCW-from-East (ENU)
  double yaw_enu = -yaw_raw + yaw_offset_;
  while (yaw_enu >  M_PI) yaw_enu -= 2.0 * M_PI;
  while (yaw_enu < -M_PI) yaw_enu += 2.0 * M_PI;

  // ---- Initialisation phase: collect heading samples ----
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

  // ---- Normal EKF update ----
  if (heading_received_ && last_lio_)
  {
    const double yaw_odom = quaternionToYaw(last_lio_->pose.pose.orientation);
    double innov = yaw_enu - (yaw_odom + drift_(2));
    while (innov >  M_PI) innov -= 2.0 * M_PI;
    while (innov < -M_PI) innov += 2.0 * M_PI;

    // Detailed heading comparison log (every measurement)
    RCLCPP_INFO(get_logger(),
      "HDG raw=%.4f enu=%.4f | LIO=%.4f drift_yaw=%.4f | innov=%.4f [%s]",
      yaw_raw, yaw_enu, yaw_odom, drift_(2), innov,
      std::abs(innov) > max_heading_innovation_ ? "REJECT" : "ACCEPT");

    if (std::abs(innov) > max_heading_innovation_)
    {
      return;
    }
  }

  last_heading_yaw_ = yaw_enu;
  heading_received_ = true;

  if (rtk_quality_ == RtkQuality::NONE) return;
  if (enable_noise_sim_ && in_shadow_) return;

  const double r_yaw = (rtk_quality_ == RtkQuality::FIX)   ? yaw_r_ :
                       (rtk_quality_ == RtkQuality::FLOAT)  ? yaw_r_ * 4.0 :
                                                               yaw_r_ * 16.0;
  ekfUpdateHeading(yaw_enu, r_yaw);
}

// ============================================================
// GNSS initialisation: check sample stability and initialise drift_
// ============================================================
void GnssCorrectorNode::tryGnssInit()
{
  if (!lio_received_ || !last_lio_) return;

  // Keep a sliding window — discard oldest if buffer grows too large
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
      "Init collecting: pos=%zu/%d  yaw=%zu/%d samples (RTK FIX required for pos)",
      init_e_samples_.size(), gnss_init_samples_,
      init_yaw_samples_.size(), gnss_init_samples_);
    return;
  }

  // Check position stability
  const double e_mean = sampleMean(init_e_samples_);
  const double n_mean = sampleMean(init_n_samples_);
  const double e_std  = sampleStd(init_e_samples_, e_mean);
  const double n_std  = sampleStd(init_n_samples_, n_mean);

  if (e_std > gnss_init_pos_std_ || n_std > gnss_init_pos_std_)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Init: position not stable (e_std=%.3fm  n_std=%.3fm  limit=%.3fm). "
      "Keep robot stationary.",
      e_std, n_std, gnss_init_pos_std_);
    // Clear buffer — robot may have been moving; restart collection
    init_e_samples_.clear();
    init_n_samples_.clear();
    return;
  }

  // Check heading stability
  const double yaw_mean = circularMean(init_yaw_samples_);
  const double yaw_std  = circularStd(init_yaw_samples_, yaw_mean);

  if (yaw_std > gnss_init_yaw_std_)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Init: heading not stable (yaw_std=%.4f rad  limit=%.4f rad). "
      "Keep robot stationary.",
      yaw_std, gnss_init_yaw_std_);
    init_yaw_samples_.clear();
    return;
  }

  // ---- Both stable: initialise drift_ from GNSS ----
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

  // Set P to GPS measurement quality (well-initialised, low uncertainty)
  P_ = Eigen::Matrix3d::Zero();
  P_(0, 0) = pos_r_fix_;
  P_(1, 1) = pos_r_fix_;
  P_(2, 2) = yaw_r_;

  // Bootstrap heading tracker so the innovation gate is active immediately
  last_heading_yaw_ = yaw_mean;
  heading_received_ = true;

  init_state_ = InitState::RUNNING;

  RCLCPP_INFO(get_logger(),
    "=== GNSS init complete ==="
    "\n  map origin : lat=%.9f  lon=%.9f"
    "\n  init pos   : e=%.3f m  n=%.3f m  (std: e=%.3f  n=%.3f)"
    "\n  init yaw   : %.4f rad  (std: %.4f)"
    "\n  drift_     : [%.3f, %.3f, %.4f]",
    datum_lat_, datum_lon_,
    e_mean, n_mean, e_std, n_std,
    yaw_mean, yaw_std,
    drift_(0), drift_(1), drift_(2));
}

// ============================================================
// EKF: predict (random walk process model)
// ============================================================
void GnssCorrectorNode::ekfPredict(double dt)
{
  P_(0, 0) += pos_process_noise_ * dt;
  P_(1, 1) += pos_process_noise_ * dt;
  P_(2, 2) += yaw_process_noise_ * dt;
}

// ============================================================
// EKF: position measurement update
// ============================================================
void GnssCorrectorNode::ekfUpdatePosition(
  double e_meas, double n_meas, const Eigen::Matrix2d & R)
{
  if (!last_lio_) return;

  const double x_b  = last_lio_->pose.pose.position.x;
  const double y_b  = last_lio_->pose.pose.position.y;
  const double dyaw = drift_(2);
  const double cd   = std::cos(dyaw);
  const double sd   = std::sin(dyaw);

  const double h_e = drift_(0) + cd * x_b - sd * y_b;
  const double h_n = drift_(1) + sd * x_b + cd * y_b;

  Eigen::Vector2d z;
  z(0) = e_meas - h_e;
  z(1) = n_meas - h_n;

  // Full Jacobian: GPS position observes dyaw when robot is displaced from odom origin.
  // ∂h_e/∂dyaw = -sin(dyaw)*x_b - cos(dyaw)*y_b
  // ∂h_n/∂dyaw =  cos(dyaw)*x_b - sin(dyaw)*y_b
  // This allows GPS position to correct yaw drift during straight-line travel.
  Eigen::Matrix<double, 2, 3> H;
  H(0, 0) = 1.0;  H(0, 1) = 0.0;  H(0, 2) = -sd * x_b - cd * y_b;
  H(1, 0) = 0.0;  H(1, 1) = 1.0;  H(1, 2) =  cd * x_b - sd * y_b;

  const Eigen::Matrix2d S = H * P_ * H.transpose() + R;
  const Eigen::Matrix<double, 3, 2> K = P_ * H.transpose() * S.inverse();

  Eigen::Vector3d delta = K * z;

  const double pos_corr = delta.head<2>().norm();
  if (pos_corr > max_pos_correction_)
  {
    const double scale = max_pos_correction_ / pos_corr;
    delta(0) *= scale;
    delta(1) *= scale;
    delta(2) *= scale;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Position correction clipped: %.2f → %.2f m", pos_corr, max_pos_correction_);
  }
  if (std::abs(delta(2)) > max_yaw_correction_)
  {
    const double scale = max_yaw_correction_ / std::abs(delta(2));
    delta *= scale;
  }

  drift_ += delta;
  while (drift_(2) >  M_PI) drift_(2) -= 2.0 * M_PI;
  while (drift_(2) < -M_PI) drift_(2) += 2.0 * M_PI;

  const Eigen::Matrix3d I_KH = Eigen::Matrix3d::Identity() - K * H;
  P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
  P_ = 0.5 * (P_ + P_.transpose());
}

// ============================================================
// EKF: heading measurement update
// ============================================================
void GnssCorrectorNode::ekfUpdateHeading(double yaw_meas, double yaw_r)
{
  if (!last_lio_) return;

  const double yaw_odom = quaternionToYaw(last_lio_->pose.pose.orientation);

  double z = yaw_meas - (yaw_odom + drift_(2));
  while (z >  M_PI) z -= 2.0 * M_PI;
  while (z < -M_PI) z += 2.0 * M_PI;

  const Eigen::RowVector3d H(0.0, 0.0, 1.0);
  const double S = H * P_ * H.transpose() + yaw_r;
  const Eigen::Vector3d K = P_ * H.transpose() / S;

  Eigen::Vector3d delta = K * z;
  if (std::abs(delta(2)) > max_yaw_correction_)
  {
    const double scale = max_yaw_correction_ / std::abs(delta(2));
    delta *= scale;
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

// ============================================================
// Publish /odometry/map — LIO transformed into map frame
// ============================================================
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

// ============================================================
// Publish /odometry/gnss — raw GPS position in map frame
// ============================================================
void GnssCorrectorNode::publishGnssOdom(
  double e, double n, double r_pos, const rclcpp::Time & stamp)
{
  nav_msgs::msg::Odometry gnss_odom;
  gnss_odom.header.stamp    = stamp;
  gnss_odom.header.frame_id = frame_map_;
  gnss_odom.child_frame_id  = "gps";

  gnss_odom.pose.pose.position.x = e;
  gnss_odom.pose.pose.position.y = n;
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

// ============================================================
// Broadcast dynamic map → odom TF
// ============================================================
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

// ============================================================
// Append pose to path with ring-buffer cap
// ============================================================
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

// ============================================================
// lat/lon → local ENU (fixed datum)
// ============================================================
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

// ============================================================
// Extract yaw from geometry_msgs::Quaternion
// ============================================================
double GnssCorrectorNode::quaternionToYaw(
  const geometry_msgs::msg::Quaternion & q) const
{
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  return yaw;
}

// ============================================================
// Statistical helpers
// ============================================================
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

}  // namespace gnss_fusion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gnss_fusion::GnssCorrectorNode>());
  rclcpp::shutdown();
  return 0;
}
