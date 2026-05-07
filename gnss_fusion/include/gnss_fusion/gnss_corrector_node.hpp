#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>
#include <random>
#include <string>
#include <vector>

namespace gnss_fusion
{

/**
 * LIO long-term drift correction using Dual RTK GNSS.
 *
 * Maintains a 3-state EKF [dx, dy, dyaw] representing the accumulated
 * drift of the odom frame relative to the GPS-defined map frame.
 *
 * Initialisation sequence (state machine):
 *   WAIT_LIO         – waiting for first LIO message
 *   WAIT_GNSS_STABLE – collecting GPS position + heading samples while
 *                      robot is stationary; transitions to RUNNING once
 *                      both are stable (std-dev within thresholds)
 *   RUNNING          – normal EKF predict / update / publish
 *
 * The datum (ENU origin) is a fixed parameter set to a known harbour
 * reference point so that all sessions share the same absolute coordinate
 * frame (instead of resetting to (0,0) at each boot).
 *
 * Inputs:  /Odometry (LIO), /fix (NavSatFix), /heading (QuaternionStamped)
 * Outputs: /odometry/map    — LIO odometry corrected into map frame
 *          /odometry/gnss   — raw GPS position in map frame (for comparison)
 *          /path/map        — accumulated path of /odometry/map (gapless)
 *          /path/gnss       — accumulated GPS track (gapless)
 *          map→odom TF      — dynamic drift correction transform
 */
class GnssCorrectorNode : public rclcpp::Node
{
public:
  GnssCorrectorNode();

private:
  // ---- Initialisation state machine ----
  enum class InitState { WAIT_LIO, WAIT_GNSS_STABLE, RUNNING };

  // Callbacks
  void lioCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void headingCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg);

  // Initialisation
  void tryGnssInit();

  // EKF operations
  void ekfPredict(double dt);
  void ekfUpdatePosition(double e_meas, double n_meas, const Eigen::Matrix2d & R);
  void ekfUpdateHeading(double yaw_meas, double yaw_r);

  // Publishing helpers
  void publishMapOdom(const nav_msgs::msg::Odometry::SharedPtr & lio_msg);
  void publishGnssOdom(double e, double n, double r_pos, const rclcpp::Time & stamp);
  void broadcastMapOdomTf(const rclcpp::Time & stamp);
  void appendPath(nav_msgs::msg::Path & path,
                  const geometry_msgs::msg::PoseStamped & pose,
                  std::size_t max_poses);

  // Coordinate / quaternion helpers
  void latLonToEnu(double lat, double lon, double & e, double & n) const;
  double quaternionToYaw(const geometry_msgs::msg::Quaternion & q) const;

  // Statistical helpers for initialisation
  static double sampleMean(const std::vector<double> & v);
  static double sampleStd(const std::vector<double> & v, double mean);
  static double circularMean(const std::vector<double> & angles);
  static double circularStd(const std::vector<double> & angles, double mean);

  // ---- ROS interfaces ----
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_lio_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_fix_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr sub_heading_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_map_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_gnss_odom_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_map_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_gnss_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Accumulated paths (ring-buffered for memory safety)
  nav_msgs::msg::Path path_map_;
  nav_msgs::msg::Path path_gnss_;
  static constexpr std::size_t PATH_MAX_POSES = 100000;

  // ---- Parameters ----
  std::string frame_map_;
  std::string frame_odom_;
  std::string frame_body_;
  std::string topic_lio_;
  std::string topic_fix_;
  std::string topic_heading_;
  double yaw_offset_;
  double pos_process_noise_;
  double yaw_process_noise_;
  double pos_r_fix_;
  double pos_r_float_;
  double pos_r_single_;
  double yaw_r_;
  double max_pos_correction_;
  double max_yaw_correction_;
  double max_heading_innovation_;
  bool   use_rtk_covariance_;
  double restart_gap_s_;

  // Initialisation parameters
  int    gnss_init_samples_;   // minimum samples required
  double gnss_init_pos_std_;   // max position std-dev [m] to declare stable
  double gnss_init_yaw_std_;   // max heading std-dev [rad] to declare stable

  // GPS antenna lever arm in body frame [m]
  double antenna_offset_x_;
  double antenna_offset_y_;

  // Noise simulation parameters
  bool   enable_noise_sim_;
  double multipath_prob_;
  double multipath_magnitude_;
  double multipath_duration_s_;
  double shadow_prob_;
  double shadow_duration_s_;

  // ---- EKF state: [dx, dy, dyaw] ----
  Eigen::Vector3d drift_;
  Eigen::Matrix3d P_;

  // ---- Init state machine ----
  InitState init_state_;

  // Sample accumulators for GNSS initialisation
  std::vector<double> init_e_samples_;
  std::vector<double> init_n_samples_;
  std::vector<double> init_yaw_samples_;

  // ---- Datum (fixed harbour reference point) ----
  double datum_lat_;
  double datum_lon_;

  // WGS84 constants
  static constexpr double WGS84_A  = 6378137.0;
  static constexpr double WGS84_E2 = 0.00669437999014;
  static constexpr double DEG2RAD  = M_PI / 180.0;

  // ---- Cached sensor state ----
  nav_msgs::msg::Odometry::SharedPtr last_lio_;
  rclcpp::Time last_lio_stamp_;
  bool lio_received_;

  double last_heading_yaw_;
  bool   heading_received_;

  // Last valid map-frame body position — for seamless LIO restart
  double last_map_x_;
  double last_map_y_;
  bool   last_map_valid_;

  // RTK quality state machine
  enum class RtkQuality { NONE, SINGLE, FLOAT, FIX };
  RtkQuality rtk_quality_;

  // ---- Noise simulation state ----
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

}  // namespace gnss_fusion
