#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <std_srvs/srv/trigger.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <mutex>
#include <string>
#include <memory>
#include <limits>
#include <vector>

namespace lidar_calib
{

using PointT   = pcl::PointXYZ;
using Cloud    = pcl::PointCloud<PointT>;
using CloudPtr = Cloud::Ptr;

using SyncPolicy = message_filters::sync_policies::ApproximateTime<
  sensor_msgs::msg::PointCloud2,
  sensor_msgs::msg::PointCloud2>;

struct CubeTarget {
  Eigen::Vector3f center;   // plane RANSAC으로 추정한 큐브 기하 중심
  CloudPtr        cloud;    // 클러스터 포인트 클라우드
  float           quality;  // plane inlier 비율 (0.0 = centroid fallback)
};

class LidarCalibrationNode : public rclcpp::Node
{
public:
  explicit LidarCalibrationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── Callbacks ─────────────────────────────────────────────────────────────
  void cloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg1,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg2);

  // ── Preprocessing ─────────────────────────────────────────────────────────
  // voxel_size <= 0 이면 voxelGrid 생략 (sphere 검출용)
  CloudPtr preprocessCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg,
    double voxel_size);

  // ── Multi-yaw ICP (번역 초기값 + yaw 자동 탐색) ───────────────────────────
  bool runMultiYawICP(
    const CloudPtr & source, const CloudPtr & target,
    double tx, double ty, double tz,
    Eigen::Matrix4f & best_result, double & best_score);

  bool runICP(
    const CloudPtr & source, const CloudPtr & target,
    const Eigen::Matrix4f & initial_guess,
    Eigen::Matrix4f & result, double & fitness_score,
    double corr_dist);

  // ── Sphere-based calibration ───────────────────────────────────────────────
  // 포인트클라우드에서 반지름이 sphere_radius_ 인 구의 중심 목록 반환
  std::vector<Eigen::Vector3f> detectSpheres(const CloudPtr & cloud);

  // 큐브 클러스터 검출 → plane RANSAC으로 기하 중심 추정
  std::vector<CubeTarget> detectCubeCenters(const CloudPtr & cloud);

  // Kabsch SVD: T 를 구해 T * src[i] ≈ dst[i] 를 만족
  static Eigen::Matrix4f kabsch(
    const std::vector<Eigen::Vector3f> & src,
    const std::vector<Eigen::Vector3f> & dst);

  // src 순열 전수 탐색 → 잔차 최소 T 반환
  static Eigen::Matrix4f matchAndSolve(
    const std::vector<Eigen::Vector3f> & src,
    const std::vector<Eigen::Vector3f> & dst,
    double & residual);

  // ── Output helpers ─────────────────────────────────────────────────────────
  void broadcastTransform(const Eigen::Matrix4f & T);
  void publishClouds(
    const CloudPtr & cloud1, const CloudPtr & cloud2_aligned,
    const rclcpp::Time & stamp);
  bool saveYaml(const std::string & path);

  // ── Services ──────────────────────────────────────────────────────────────
  void srvCalibrate(          // Multi-yaw ICP
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);

  void srvSphereCalibrate(    // Sphere SVD (초기값 불필요)
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);

  void srvCubeCalibrate(      // 큐브 Kabsch SVD + ICP 정제 (초기값 불필요)
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);

  void srvSave(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);

  // ── Math helpers ──────────────────────────────────────────────────────────
  static Eigen::Matrix4f rpyToMatrix(
    double x, double y, double z,
    double roll, double pitch, double yaw);

  static void matrixToRpy(
    const Eigen::Matrix4f & mat,
    double & x, double & y, double & z,
    double & roll, double & pitch, double & yaw);

  // ── Parameters ────────────────────────────────────────────────────────────
  std::string lidar_mode_;
  std::string lidar1_frame_;
  std::string lidar2_frame_;

  // Preprocessing
  double voxel_size_;
  double min_range_, max_range_;
  double min_z_, max_z_;

  // Multi-yaw ICP
  double init_x_, init_y_, init_z_;
  int    yaw_candidates_;
  double search_corr_dist_;
  double refine_corr_dist_;
  int    max_iter_;
  double trans_eps_;
  double fit_eps_;
  double fit_threshold_;

  // Sphere calibration
  double sphere_radius_;
  double sphere_radius_tol_;
  double sphere_cluster_tol_;
  int    sphere_min_cluster_;
  int    sphere_max_cluster_;

  // Cube calibration
  double cube_size_xy_;
  double cube_size_z_;
  double cube_size_tolerance_;
  double cube_cluster_tol_;
  int    cube_min_cluster_;
  int    cube_max_cluster_;
  double cube_plane_inlier_thresh_;
  double cube_min_distance_;    // 클러스터 무게중심이 이 거리 이상이어야 큐브로 인정

  double sync_slop_sec_;
  std::string output_path_;

  // ── Runtime state ─────────────────────────────────────────────────────────
  Eigen::Matrix4f current_T_{Eigen::Matrix4f::Identity()};
  double          current_score_{std::numeric_limits<double>::max()};
  bool            icp_done_{false};
  std::mutex      cloud_mutex_;

  CloudPtr cached_cloud1_;      // voxelized (ICP용)
  CloudPtr cached_cloud2_;
  CloudPtr cached_raw1_;        // raw 최신 단일 프레임
  CloudPtr cached_raw2_;
  CloudPtr accumulated_raw1_;   // 누적 raw cloud (큐브 검출용)
  CloudPtr accumulated_raw2_;
  int      accum_count_{0};     // 누적 프레임 수

  // ── ROS2 interfaces ───────────────────────────────────────────────────────
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub1_, sub2_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_aligned_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_merged_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_br_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_calib_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_sphere_calib_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_cube_calib_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_save_;
};

}  // namespace lidar_calib
