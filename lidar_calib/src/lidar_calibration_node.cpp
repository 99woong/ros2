#include "lidar_calib/lidar_calibration_node.hpp"

#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <sstream>

namespace lidar_calib
{

// ─────────────────────────────────────────────────────────────── constructor ──

LidarCalibrationNode::LidarCalibrationNode(const rclcpp::NodeOptions & options)
: Node("lidar_calibration_node", options)
{
  declare_parameter("lidar_mode", "general");
  lidar_mode_ = get_parameter("lidar_mode").as_string();
  const bool is_ouster = (lidar_mode_ == "ouster");

  declare_parameter("lidar1_topic", is_ouster ? "/ouster1/points" : "/point_cloud");
  declare_parameter("lidar2_topic", is_ouster ? "/ouster2/points" : "/point_cloud2");
  declare_parameter("lidar1_frame", is_ouster ? "os_sensor"  : "sim_lidar");
  declare_parameter("lidar2_frame", is_ouster ? "os_sensor2" : "sim_lidar2");

  declare_parameter("voxel_size",  0.1);
  declare_parameter("min_z",      -2.0);
  declare_parameter("max_z",       5.0);
  declare_parameter("min_range",   1.0);
  declare_parameter("max_range",  60.0);

  declare_parameter("init_x",  0.0);
  declare_parameter("init_y",  0.0);
  declare_parameter("init_z",  0.0);
  declare_parameter("yaw_candidates", 8);
  declare_parameter("search_correspondence_distance", 3.0);
  declare_parameter("refine_correspondence_distance", 1.0);
  declare_parameter("max_iterations",             100);
  declare_parameter("transformation_epsilon",     1e-8);
  declare_parameter("euclidean_fitness_epsilon",  1e-6);
  declare_parameter("fitness_score_threshold",    0.5);

  declare_parameter("sphere_radius",          0.5);
  declare_parameter("sphere_radius_tolerance", 0.1);
  declare_parameter("sphere_cluster_tolerance", 0.4);
  declare_parameter("sphere_min_cluster_size",   15);
  declare_parameter("sphere_max_cluster_size", 3000);

  declare_parameter("cube_size_xy",           1.0);
  declare_parameter("cube_size_z",            2.0);
  declare_parameter("cube_size_tolerance",    0.4);
  declare_parameter("cube_cluster_tolerance", 0.25);
  declare_parameter("cube_min_cluster_size",    8);
  declare_parameter("cube_max_cluster_size", 2000);
  declare_parameter("cube_plane_inlier_min",  0.4);
  declare_parameter("cube_min_distance",      5.0);

  declare_parameter("sync_slop_sec", is_ouster ? 0.01 : 0.05);
  declare_parameter("output_path", "/tmp/lidar_calib_result.yaml");

  lidar1_frame_  = get_parameter("lidar1_frame").as_string();
  lidar2_frame_  = get_parameter("lidar2_frame").as_string();
  voxel_size_    = get_parameter("voxel_size").as_double();
  min_z_         = get_parameter("min_z").as_double();
  max_z_         = get_parameter("max_z").as_double();
  min_range_     = get_parameter("min_range").as_double();
  max_range_     = get_parameter("max_range").as_double();

  init_x_           = get_parameter("init_x").as_double();
  init_y_           = get_parameter("init_y").as_double();
  init_z_           = get_parameter("init_z").as_double();
  yaw_candidates_   = get_parameter("yaw_candidates").as_int();
  search_corr_dist_ = get_parameter("search_correspondence_distance").as_double();
  refine_corr_dist_ = get_parameter("refine_correspondence_distance").as_double();
  max_iter_         = get_parameter("max_iterations").as_int();
  trans_eps_        = get_parameter("transformation_epsilon").as_double();
  fit_eps_          = get_parameter("euclidean_fitness_epsilon").as_double();
  fit_threshold_    = get_parameter("fitness_score_threshold").as_double();

  sphere_radius_      = get_parameter("sphere_radius").as_double();
  sphere_radius_tol_  = get_parameter("sphere_radius_tolerance").as_double();
  sphere_cluster_tol_ = get_parameter("sphere_cluster_tolerance").as_double();
  sphere_min_cluster_ = get_parameter("sphere_min_cluster_size").as_int();
  sphere_max_cluster_ = get_parameter("sphere_max_cluster_size").as_int();

  cube_size_xy_             = get_parameter("cube_size_xy").as_double();
  cube_size_z_              = get_parameter("cube_size_z").as_double();
  cube_size_tolerance_      = get_parameter("cube_size_tolerance").as_double();
  cube_cluster_tol_         = get_parameter("cube_cluster_tolerance").as_double();
  cube_min_cluster_         = get_parameter("cube_min_cluster_size").as_int();
  cube_max_cluster_         = get_parameter("cube_max_cluster_size").as_int();
  cube_plane_inlier_thresh_ = get_parameter("cube_plane_inlier_min").as_double();
  cube_min_distance_        = get_parameter("cube_min_distance").as_double();

  sync_slop_sec_ = get_parameter("sync_slop_sec").as_double();
  output_path_   = get_parameter("output_path").as_string();

  pub_aligned_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "/lidar_calib/aligned_cloud", rclcpp::SensorDataQoS());
  pub_merged_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "/lidar_calib/merged_cloud", rclcpp::SensorDataQoS());

  static_tf_br_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  broadcastTransform(current_T_);

  srv_calib_ = create_service<std_srvs::srv::Trigger>(
    "/lidar_calib/run_calibration",
    std::bind(&LidarCalibrationNode::srvCalibrate, this,
      std::placeholders::_1, std::placeholders::_2));

  srv_sphere_calib_ = create_service<std_srvs::srv::Trigger>(
    "/lidar_calib/sphere_calibration",
    std::bind(&LidarCalibrationNode::srvSphereCalibrate, this,
      std::placeholders::_1, std::placeholders::_2));

  srv_cube_calib_ = create_service<std_srvs::srv::Trigger>(
    "/lidar_calib/cube_calibration",
    std::bind(&LidarCalibrationNode::srvCubeCalibrate, this,
      std::placeholders::_1, std::placeholders::_2));

  srv_save_ = create_service<std_srvs::srv::Trigger>(
    "/lidar_calib/save_calibration",
    std::bind(&LidarCalibrationNode::srvSave, this,
      std::placeholders::_1, std::placeholders::_2));

  const std::string t1 = get_parameter("lidar1_topic").as_string();
  const std::string t2 = get_parameter("lidar2_topic").as_string();

  sub1_.subscribe(this, t1, rmw_qos_profile_sensor_data);
  sub2_.subscribe(this, t2, rmw_qos_profile_sensor_data);

  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(20), sub1_, sub2_);
  sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_slop_sec_));
  sync_->registerCallback(
    std::bind(&LidarCalibrationNode::cloudCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "=== lidar_calib node started ===");
  RCLCPP_INFO(get_logger(), "  mode   : %s", lidar_mode_.c_str());
  RCLCPP_INFO(get_logger(), "  lidar1 : %s  frame=%s", t1.c_str(), lidar1_frame_.c_str());
  RCLCPP_INFO(get_logger(), "  lidar2 : %s  frame=%s", t2.c_str(), lidar2_frame_.c_str());
  RCLCPP_INFO(get_logger(), "  services:");
  RCLCPP_INFO(get_logger(), "    /lidar_calib/cube_calibration    — 큐브 Kabsch SVD + ICP (초기값 불필요) ★");
  RCLCPP_INFO(get_logger(), "    /lidar_calib/sphere_calibration  — 구 기반 SVD (초기값 불필요)");
  RCLCPP_INFO(get_logger(), "    /lidar_calib/run_calibration     — 다중 yaw ICP");
  RCLCPP_INFO(get_logger(), "    /lidar_calib/save_calibration    — YAML 저장 → %s",
    output_path_.c_str());
}

// ──────────────────────────────────────────────────── synchronized callback ──

void LidarCalibrationNode::cloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg1,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg2)
{
  // voxelized (ICP용)
  CloudPtr c1 = preprocessCloud(msg1, voxel_size_);
  CloudPtr c2 = preprocessCloud(msg2, voxel_size_);

  // raw (sphere 검출용 — voxel 생략)
  CloudPtr r1 = preprocessCloud(msg1, -1.0);
  CloudPtr r2 = preprocessCloud(msg2, -1.0);

  if (c1->empty() || c2->empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "전처리 후 포인트 클라우드가 비어 있습니다.");
    return;
  }

  {
    std::lock_guard<std::mutex> lk(cloud_mutex_);
    cached_cloud1_ = c1;
    cached_cloud2_ = c2;
    cached_raw1_   = r1;
    cached_raw2_   = r2;

    // 큐브 검출용 누적 cloud — 매 프레임 추가
    if (!accumulated_raw1_) accumulated_raw1_ = std::make_shared<Cloud>();
    if (!accumulated_raw2_) accumulated_raw2_ = std::make_shared<Cloud>();
    *accumulated_raw1_ += *r1;
    *accumulated_raw2_ += *r2;
    accum_count_++;

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
      "[누적] %d 프레임  pts1=%zu  pts2=%zu",
      accum_count_, accumulated_raw1_->size(), accumulated_raw2_->size());

    // 누적 포인트가 50k 초과 시 voxel 다운샘플 (0.05m) 로 크기 제어
    const size_t MAX_ACCUM = 50000;
    if (accumulated_raw1_->size() > MAX_ACCUM) {
      auto voxel_reduce = [](CloudPtr & cloud) {
        pcl::VoxelGrid<PointT> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.05f, 0.05f, 0.05f);
        CloudPtr tmp = std::make_shared<Cloud>();
        vg.filter(*tmp);
        cloud = tmp;
      };
      voxel_reduce(accumulated_raw1_);
      voxel_reduce(accumulated_raw2_);
    }
  }

  CloudPtr c2_aligned = std::make_shared<Cloud>();
  pcl::transformPointCloud(*c2, *c2_aligned, current_T_);
  publishClouds(c1, c2_aligned, now());
}

// ──────────────────────────────────────────────────────── preprocessing ──────

CloudPtr LidarCalibrationNode::preprocessCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg,
  double voxel_size)
{
  CloudPtr cloud = std::make_shared<Cloud>();
  pcl::fromROSMsg(*msg, *cloud);

  {
    std::vector<int> dummy;
    cloud->is_dense = false;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, dummy);
  }

  {
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(static_cast<float>(min_z_), static_cast<float>(max_z_));
    pass.filter(*cloud);
  }

  {
    CloudPtr in_range = std::make_shared<Cloud>();
    in_range->reserve(cloud->size());
    const float rmin2 = static_cast<float>(min_range_ * min_range_);
    const float rmax2 = static_cast<float>(max_range_ * max_range_);
    for (const auto & pt : *cloud) {
      const float r2 = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
      if (r2 >= rmin2 && r2 <= rmax2) {
        in_range->push_back(pt);
      }
    }
    cloud = in_range;
  }

  if (voxel_size > 0.0) {
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    const float leaf = static_cast<float>(voxel_size);
    vg.setLeafSize(leaf, leaf, leaf);
    vg.filter(*cloud);
  }

  return cloud;
}

// ──────────────────────────────── Sphere detection (RANSAC) ──────────────────

std::vector<Eigen::Vector3f> LidarCalibrationNode::detectSpheres(const CloudPtr & cloud)
{
  std::vector<Eigen::Vector3f> centers;

  if (cloud->size() < static_cast<size_t>(sphere_min_cluster_)) {
    return centers;
  }

  // Euclidean clustering
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> clusters;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(static_cast<float>(sphere_cluster_tol_));
  ec.setMinClusterSize(sphere_min_cluster_);
  ec.setMaxClusterSize(sphere_max_cluster_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(clusters);

  RCLCPP_INFO(get_logger(), "  클러스터 %zu개 검출", clusters.size());

  for (const auto & idx : clusters) {
    CloudPtr cluster(new Cloud);
    pcl::ExtractIndices<PointT> ext;
    ext.setInputCloud(cloud);
    ext.setIndices(std::make_shared<pcl::PointIndices>(idx));
    ext.filter(*cluster);

    // RANSAC sphere fitting
    pcl::SACSegmentation<PointT> seg;
    seg.setModelType(pcl::SACMODEL_SPHERE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.03);
    seg.setRadiusLimits(
      sphere_radius_ - sphere_radius_tol_,
      sphere_radius_ + sphere_radius_tol_);
    seg.setMaxIterations(1000);
    seg.setInputCloud(cluster);

    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.segment(*inliers, *coeff);

    // 인라이어 비율이 충분해야 구로 인정
    const float inlier_ratio =
      static_cast<float>(inliers->indices.size()) / cluster->size();

    if (coeff->values.size() == 4 && inlier_ratio >= 0.4f) {
      Eigen::Vector3f c(coeff->values[0], coeff->values[1], coeff->values[2]);
      centers.push_back(c);
      RCLCPP_INFO(get_logger(),
        "    구 검출: center=(%.3f, %.3f, %.3f)  r=%.3f  inlier=%.0f%%",
        c.x(), c.y(), c.z(), coeff->values[3], inlier_ratio * 100.f);
    }
  }

  return centers;
}

// ──────────────────────────── Cube detection (bounding-box + plane RANSAC) ───

std::vector<CubeTarget> LidarCalibrationNode::detectCubeCenters(const CloudPtr & cloud)
{
  std::vector<CubeTarget> results;

  if (cloud->size() < static_cast<size_t>(cube_min_cluster_)) {
    RCLCPP_WARN(get_logger(),
      "  입력 pts=%zu < cube_min_cluster(%d) — 클러스터링 생략",
      cloud->size(), cube_min_cluster_);
    return results;
  }

  // 누적 raw cloud는 프레임×큐브당 수만 pts까지 증가해 cube_max_cluster_size를 초과하므로
  // 클러스터링 전 0.1m voxelization으로 밀도를 고정 (1m³ 큐브 → 최대 ~1000 pts)
  CloudPtr cloud_in = std::make_shared<Cloud>();
  {
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.1f, 0.1f, 0.1f);
    vg.filter(*cloud_in);
  }
  RCLCPP_INFO(get_logger(),
    "  [전처리] 입력=%zu pts → voxel(0.1m) 후=%zu pts",
    cloud->size(), cloud_in->size());

  // ── 클라우드 공간 범위 (bbox) ────────────────────────────────────────────────
  {
    PointT cmin, cmax;
    pcl::getMinMax3D(*cloud_in, cmin, cmax);
    RCLCPP_INFO(get_logger(),
      "  [클라우드 bbox] x=[%.2f~%.2f]  y=[%.2f~%.2f]  z=[%.2f~%.2f]  (범위 dx=%.2f dy=%.2f dz=%.2f)",
      cmin.x, cmax.x, cmin.y, cmax.y, cmin.z, cmax.z,
      cmax.x - cmin.x, cmax.y - cmin.y, cmax.z - cmin.z);
    if (cmax.z - cmin.z < 0.05f) {
      RCLCPP_WARN(get_logger(),
        "  ⚠ z 전체범위=%.3f m — 포인트클라우드가 2D에 가깝습니다"
        " (센서 좌표계 축 방향 확인 필요)",
        cmax.z - cmin.z);
    }
  }

  // ── Euclidean 클러스터링: PCL max 제한 없이 전체 추출 후 직접 필터링 ────────
  // PCL은 [min,max] 밖의 클러스터를 조용히 폐기하므로 모두 꺼내서 직접 분류
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud_in);

  std::vector<pcl::PointIndices> all_clusters;
  {
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(static_cast<float>(cube_cluster_tol_));
    ec.setMinClusterSize(1);
    ec.setMaxClusterSize(static_cast<int>(cloud_in->size()));
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_in);
    ec.extract(all_clusters);
  }

  // ── 클러스터 크기 분포 ────────────────────────────────────────────────────────
  {
    int b_tiny = 0, b_small = 0, b_mid = 0, b_large = 0, b_over = 0;
    for (const auto & idx : all_clusters) {
      const int n = static_cast<int>(idx.indices.size());
      if      (n < cube_min_cluster_)  b_tiny++;
      else if (n < 50)                 b_small++;
      else if (n < 200)                b_mid++;
      else if (n <= cube_max_cluster_) b_large++;
      else                             b_over++;
    }
    RCLCPP_INFO(get_logger(),
      "  [클러스터 크기분포] 전체=%zu개  <min(%d):%d  [%d~50):%d  [50~200):%d"
      "  [200~max(%d)]:%d  >max:%d",
      all_clusters.size(),
      cube_min_cluster_, b_tiny,
      cube_min_cluster_, b_small,
      b_mid,
      cube_max_cluster_, b_large,
      b_over);
    if (b_over > 0) {
      RCLCPP_WARN(get_logger(),
        "  ⚠ 크기초과 클러스터 %d개 존재 — cube_max_cluster_size(%d) 증가 고려"
        " 또는 씬에 연결된 대형 구조물 확인",
        b_over, cube_max_cluster_);
    }
  }

  // 크기 범위 내 클러스터만 남김
  std::vector<pcl::PointIndices> clusters;
  clusters.reserve(all_clusters.size());
  for (auto & idx : all_clusters) {
    const int n = static_cast<int>(idx.indices.size());
    if (n >= cube_min_cluster_ && n <= cube_max_cluster_) {
      clusters.push_back(std::move(idx));
    }
  }
  RCLCPP_INFO(get_logger(),
    "  크기필터 통과: %zu개 (cube_min=%d, cube_max=%d)",
    clusters.size(), cube_min_cluster_, cube_max_cluster_);

  const float sz   = static_cast<float>(cube_size_z_);
  const float sxy  = static_cast<float>(cube_size_xy_);
  const float tol  = static_cast<float>(cube_size_tolerance_);
  const float qmin = static_cast<float>(cube_plane_inlier_thresh_);
  const float dz_min = sz * 0.2f;

  int reject_dz = 0, reject_horiz = 0, reject_dist = 0;

  for (size_t ci = 0; ci < clusters.size(); ++ci) {
    CloudPtr cluster(new Cloud);
    pcl::ExtractIndices<PointT> ext;
    ext.setInputCloud(cloud_in);
    ext.setIndices(std::make_shared<pcl::PointIndices>(clusters[ci]));
    ext.filter(*cluster);

    // ── 바운딩 박스 ──────────────────────────────────────────────────────────
    PointT min_pt, max_pt;
    pcl::getMinMax3D(*cluster, min_pt, max_pt);
    const float dx = max_pt.x - min_pt.x;
    const float dy = max_pt.y - min_pt.y;
    const float dz = max_pt.z - min_pt.z;
    const float max_horiz = std::max(dx, dy);

    Eigen::Vector4f raw_c4;
    pcl::compute3DCentroid(*cluster, raw_c4);
    const float dist = raw_c4.head<3>().norm();

    RCLCPP_INFO(get_logger(),
      "  [클러스터%zu] pts=%zu  bbox dx=%.2f dy=%.2f dz=%.2f"
      "  z=[%.2f~%.2f]  centroid=(%.2f,%.2f,%.2f)  원점거리=%.2f m",
      ci, cluster->size(), dx, dy, dz,
      min_pt.z, max_pt.z,
      raw_c4.x(), raw_c4.y(), raw_c4.z(), dist);

    // ── 높이 필터 ──────────────────────────────────────────────────────────
    if (dz < dz_min || dz > sz + tol) {
      RCLCPP_INFO(get_logger(),
        "    → 거부 [높이] dz=%.3f  기대=[%.2f, %.2f]",
        dz, dz_min, sz + tol);
      // dz가 기준 미달이면 z-히스토그램으로 빔 분포 확인
      if (dz < dz_min) {
        constexpr int ZBINS = 8;
        std::array<int, ZBINS> zh = {};
        const float zrange = (dz > 1e-4f) ? dz : 1e-4f;
        for (const auto & p : *cluster) {
          int b = std::min(ZBINS - 1, static_cast<int>((p.z - min_pt.z) / zrange * ZBINS));
          zh[b]++;
        }
        std::string zstr;
        char buf[64];
        for (int b = 0; b < ZBINS; ++b) {
          if (zh[b] == 0) continue;
          snprintf(buf, sizeof(buf), "[%.2f~%.2f]=%d ",
            min_pt.z + b * zrange / ZBINS,
            min_pt.z + (b + 1) * zrange / ZBINS,
            zh[b]);
          zstr += buf;
        }
        RCLCPP_INFO(get_logger(), "      z-히스토그램: %s", zstr.c_str());
        RCLCPP_INFO(get_logger(),
          "      → 단일 LiDAR 빔 링만 맞은 것으로 추정"
          " (큐브 높이·위치 또는 cube_size_z 파라미터 확인)");
      }
      reject_dz++;
      continue;
    }

    // ── 두 번째 큰 치수 필터 (벽·모서리·라인 클러스터 제거) ──────────────────
    // 큐브 면은 반드시 2개 이상의 축에서 15% cube_size 이상 퍼져야 함
    // (예: dx=0.00, dy=0.14, dz=0.83 → 두 번째 큰 값=0.14 < 0.15 → 벽·모서리)
    {
      float dims[3] = {dx, dy, dz};
      std::sort(dims, dims + 3);
      const float second_largest = dims[1];
      const float min_face_extent = sxy * 0.15f;
      if (second_largest < min_face_extent) {
        RCLCPP_INFO(get_logger(),
          "    → 거부 [라인/벽] 두번째큰치수=%.3f m < %.3f m"
          "  (dx=%.2f dy=%.2f dz=%.2f) — 벽·모서리·라인 클러스터로 판단",
          second_largest, min_face_extent, dx, dy, dz);
        reject_horiz++;
        continue;
      }
    }

    // ── 수평 과대 필터 ─────────────────────────────────────────────────────
    const float max_horiz_limit = sz * 1.5f;
    if (max_horiz > max_horiz_limit) {
      RCLCPP_INFO(get_logger(),
        "    → 거부 [수평과대] max(dx,dy)=%.2f  허용=%.2f",
        max_horiz, max_horiz_limit);
      reject_horiz++;
      continue;
    }

    // ── 근거리 필터 ─────────────────────────────────────────────────────────
    if (dist < static_cast<float>(cube_min_distance_)) {
      RCLCPP_INFO(get_logger(),
        "    → 거부 [근거리] 원점거리=%.2f m  최소=%.2f m",
        dist, cube_min_distance_);
      reject_dist++;
      continue;
    }

    // ── Plane RANSAC → 큐브 기하 중심 계산 ──────────────────────────────────
    pcl::SACSegmentation<PointT> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.05f);
    seg.setMaxIterations(500);
    seg.setInputCloud(cluster);

    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.segment(*inliers, *coeff);

    Eigen::Vector3f cube_center;
    float quality = 0.0f;

    const float inlier_ratio =
      static_cast<float>(inliers->indices.size()) / cluster->size();

    if (coeff->values.size() == 4 && inlier_ratio >= qmin) {
      Eigen::Vector3f n(coeff->values[0], coeff->values[1], coeff->values[2]);
      n.normalize();
      const float d = coeff->values[3];
      const Eigen::Vector3f n_out = (d > 0.0f) ? n : -n;

      CloudPtr face_cloud(new Cloud);
      pcl::ExtractIndices<PointT> ext2;
      ext2.setInputCloud(cluster);
      ext2.setIndices(inliers);
      ext2.filter(*face_cloud);

      Eigen::Vector4f fc4;
      pcl::compute3DCentroid(*face_cloud, fc4);
      const Eigen::Vector3f face_center = fc4.head<3>();

      const float half_depth = (std::abs(n.z()) > 0.7f) ? sz / 2.0f : sxy / 2.0f;
      cube_center = face_center - half_depth * n_out;
      quality = inlier_ratio;

      RCLCPP_INFO(get_logger(),
        "    → 큐브 검출 center=(%.3f,%.3f,%.3f)  inlier=%.0f%%"
        "  법선=(%.2f,%.2f,%.2f)",
        cube_center.x(), cube_center.y(), cube_center.z(),
        inlier_ratio * 100.f, n_out.x(), n_out.y(), n_out.z());

      // face_cloud(RANSAC inlier) z 범위·편향 진단:
      // 빔 커버리지가 불완전하면 centroid z가 실제 면 중심과 달라져 z-translation 오차 유발
      {
        PointT fmin, fmax;
        pcl::getMinMax3D(*face_cloud, fmin, fmax);
        const float face_z_mid  = (fmin.z + fmax.z) * 0.5f;
        const float face_z_cov  = (fmax.z - fmin.z) / sz;           // 큐브높이 대비 커버리지
        const float face_z_bias = face_center.z() - face_z_mid;     // >0: 상향 bias, <0: 하향
        RCLCPP_INFO(get_logger(),
          "    → [z진단] face_z=[%.3f~%.3f]  mid=%.3f"
          "  centroid_z=%.3f  bias=%+.4f m  커버리지=%.0f%%",
          fmin.z, fmax.z, face_z_mid,
          face_center.z(), face_z_bias, face_z_cov * 100.f);
        if (face_z_cov < 0.80f) {
          RCLCPP_WARN(get_logger(),
            "    → [z진단] ★ 커버리지 %.0f%% < 80%% — 빔이 큐브 일부만 닿음 → centroid z 불신뢰",
            face_z_cov * 100.f);
        }
      }
    } else {
      Eigen::Vector4f c4;
      pcl::compute3DCentroid(*cluster, c4);
      cube_center = c4.head<3>();
      quality = 0.0f;

      RCLCPP_WARN(get_logger(),
        "    → plane fit 불안정 (inlier=%.0f%% < %.0f%%) → 무게중심 fallback"
        "  center=(%.3f,%.3f,%.3f)",
        inlier_ratio * 100.f, qmin * 100.f,
        cube_center.x(), cube_center.y(), cube_center.z());
    }

    results.push_back({cube_center, cluster, quality});
  }

  RCLCPP_INFO(get_logger(),
    "  [필터 요약] 입력클러스터=%zu  통과=%zu  거부: 높이=%d  수평과대=%d  근거리=%d",
    clusters.size(), results.size(), reject_dz, reject_horiz, reject_dist);

  // ── 검출 실패 시 진단 안내 ──────────────────────────────────────────────────
  if (results.empty() && !all_clusters.empty()) {
    const int total = static_cast<int>(all_clusters.size());
    int n_over = 0;
    for (const auto & idx : all_clusters) {
      if (static_cast<int>(idx.indices.size()) > cube_max_cluster_) n_over++;
    }
    if (reject_dz > 0) {
      RCLCPP_WARN(get_logger(),
        "  ★ 진단: 높이(dz) 거부 %d건 — 큐브가 z축 방향으로 얇게 보임."
        " [원인 후보] ①큐브 높이/위치 재배치 ②cube_size_z 파라미터 조정"
        " ③센서 z축 방향 확인",
        reject_dz);
    }
    if (reject_dist > 0) {
      RCLCPP_WARN(get_logger(),
        "  ★ 진단: 근거리 거부 %d건 — cube_min_distance=%.1f m 보다 가까운 물체."
        " 큐브를 더 멀리 배치하거나 파라미터 감소 필요",
        reject_dist, cube_min_distance_);
    }
    if (n_over > 0) {
      RCLCPP_WARN(get_logger(),
        "  ★ 진단: 크기초과 클러스터 %d개 (>%d pts) — cube_max_cluster_size 증가"
        " 또는 voxel_size 조정 필요 (현재 0.1m)",
        n_over, cube_max_cluster_);
    }
    if (reject_dz == 0 && reject_dist == 0 && reject_horiz == 0 && n_over == 0) {
      RCLCPP_WARN(get_logger(),
        "  ★ 진단: 크기필터 통과 클러스터 없음 — 전체 %d개 클러스터 모두 min(%d) 미만."
        " 씬에 큐브가 없거나 min_range/max_range 필터로 제외됐을 수 있음",
        total, cube_min_cluster_);
    }
  }

  return results;
}

// ─────────────────── Kabsch SVD: T * src[i] ≈ dst[i] ────────────────────────

Eigen::Matrix4f LidarCalibrationNode::kabsch(
  const std::vector<Eigen::Vector3f> & src,
  const std::vector<Eigen::Vector3f> & dst,
  const std::vector<float> & weights)
{
  const int N = static_cast<int>(src.size());
  const bool use_w = (static_cast<int>(weights.size()) == N);

  float w_sum = 0.0f;
  Eigen::Vector3f src_c = Eigen::Vector3f::Zero();
  Eigen::Vector3f dst_c = Eigen::Vector3f::Zero();
  for (int i = 0; i < N; ++i) {
    const float w = use_w ? weights[i] : 1.0f;
    src_c += w * src[i];
    dst_c += w * dst[i];
    w_sum += w;
  }
  src_c /= w_sum;
  dst_c /= w_sum;

  Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
  for (int i = 0; i < N; ++i) {
    const float w = use_w ? weights[i] : 1.0f;
    H += w * (src[i] - src_c) * (dst[i] - dst_c).transpose();
  }

  Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3f R = svd.matrixV() * svd.matrixU().transpose();

  // 반사 보정 (det < 0 이면 좌표계 반전 방지)
  if (R.determinant() < 0.0f) {
    Eigen::Matrix3f V = svd.matrixV();
    V.col(2) *= -1.0f;
    R = V * svd.matrixU().transpose();
  }

  const Eigen::Vector3f t = dst_c - R * src_c;

  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = t;
  return T;
}

// ───────────── 순열 전수 탐색으로 최적 매칭 + Kabsch 계산 ───────────────────

Eigen::Matrix4f LidarCalibrationNode::matchAndSolve(
  const std::vector<Eigen::Vector3f> & src,  // lidar2 구 중심
  const std::vector<Eigen::Vector3f> & dst,  // lidar1 구 중심
  double & best_residual)
{
  const int N = static_cast<int>(dst.size());

  std::vector<int> perm(N);
  std::iota(perm.begin(), perm.end(), 0);

  Eigen::Matrix4f best_T = Eigen::Matrix4f::Identity();
  best_residual = std::numeric_limits<double>::max();
  std::vector<int> best_perm(N);

  do {
    std::vector<Eigen::Vector3f> src_perm(N);
    for (int i = 0; i < N; ++i) {
      src_perm[i] = src[perm[i]];
    }

    const Eigen::Matrix4f T = kabsch(src_perm, dst);

    double residual = 0.0;
    for (int i = 0; i < N; ++i) {
      Eigen::Vector4f p(src_perm[i].x(), src_perm[i].y(), src_perm[i].z(), 1.0f);
      Eigen::Vector3f transformed = (T * p).head<3>();
      residual += (transformed - dst[i]).norm();
    }

    if (residual < best_residual) {
      best_residual = residual;
      best_T = T;
      best_perm = perm;
    }
  } while (std::next_permutation(perm.begin(), perm.end()));

  // 승자 순열 및 각 대응쌍 로그 (잘못된 매칭 진단용)
  {
    std::string perm_str;
    for (int i = 0; i < N; ++i) {
      char buf[16];
      snprintf(buf, sizeof(buf), "%d→%d ", best_perm[i], i);
      perm_str += buf;
    }
    RCLCPP_INFO(rclcpp::get_logger("lidar_calibration_node"),
      "  [Kabsch 순열] %s (src[j]→dst[i])",
      perm_str.c_str());
    for (int i = 0; i < N; ++i) {
      const Eigen::Vector3f & s = src[best_perm[i]];
      const Eigen::Vector3f & d = dst[i];
      Eigen::Vector4f p(s.x(), s.y(), s.z(), 1.0f);
      const float err = ((best_T * p).head<3>() - d).norm();
      RCLCPP_INFO(rclcpp::get_logger("lidar_calibration_node"),
        "    [%d] src=(%.3f,%.3f,%.3f) → dst=(%.3f,%.3f,%.3f)  잔차=%.4f m",
        i, s.x(), s.y(), s.z(), d.x(), d.y(), d.z(), err);
    }
  }

  return best_T;
}

// ──────────────────────────── service: sphere calibration ────────────────────

void LidarCalibrationNode::srvSphereCalibrate(
  const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  CloudPtr r1, r2;
  {
    std::lock_guard<std::mutex> lk(cloud_mutex_);
    r1 = cached_raw1_;
    r2 = cached_raw2_;
  }

  if (!r1 || !r2) {
    res->success = false;
    res->message = "아직 포인트 클라우드 수신 전입니다. 잠시 후 다시 시도하세요.";
    return;
  }

  RCLCPP_INFO(get_logger(), "=== 구 기반 캘리브레이션 시작 ===");
  RCLCPP_INFO(get_logger(), "  cloud1=%zu pts  cloud2=%zu pts", r1->size(), r2->size());
  RCLCPP_INFO(get_logger(), "  기대 구 반지름: %.2f ± %.2f m", sphere_radius_, sphere_radius_tol_);

  RCLCPP_INFO(get_logger(), "[LiDAR1] 구 검출 중...");
  auto centers1 = detectSpheres(r1);

  RCLCPP_INFO(get_logger(), "[LiDAR2] 구 검출 중...");
  auto centers2 = detectSpheres(r2);

  RCLCPP_INFO(get_logger(), "검출 결과: LiDAR1=%zu개  LiDAR2=%zu개", centers1.size(), centers2.size());

  if (centers1.size() < 3 || centers2.size() < 3) {
    std::ostringstream oss;
    oss << "구 검출 실패 (LiDAR1=" << centers1.size()
        << "개, LiDAR2=" << centers2.size() << "개). "
        << "최소 3개 필요. sphere_radius / sphere_cluster_tolerance 조정 또는 구 배치 확인.";
    res->success = false;
    res->message = oss.str();
    RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
    return;
  }

  if (centers1.size() != centers2.size()) {
    // 양쪽 중 더 많이 검출된 쪽을 3개로 제한
    const size_t n = std::min(centers1.size(), centers2.size());
    centers1.resize(n);
    centers2.resize(n);
    RCLCPP_WARN(get_logger(), "검출 수 불일치 → 각 %zu개로 사용", n);
  }

  // 순열 전수 탐색으로 최적 매칭 + Kabsch SVD
  double residual;
  const Eigen::Matrix4f T = matchAndSolve(centers2, centers1, residual);

  const double residual_per_sphere = residual / centers1.size();
  RCLCPP_INFO(get_logger(), "Kabsch SVD 완료: 평균 잔차=%.4f m/구", residual_per_sphere);

  const double residual_threshold = sphere_radius_ * 0.5;
  if (residual_per_sphere > residual_threshold) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4)
        << "잔차(" << residual_per_sphere << " m) > 임계값(" << residual_threshold
        << " m). 구 배치가 직선에 가깝거나 검출 오류. 구 위치를 조정하세요.";
    res->success = false;
    res->message = oss.str();
    RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
    return;
  }

  current_T_     = T;
  current_score_ = residual_per_sphere;
  icp_done_      = true;
  broadcastTransform(current_T_);

  double x, y, z, roll, pitch, yaw;
  matrixToRpy(current_T_, x, y, z, roll, pitch, yaw);

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6)
      << "구 캘리브레이션 성공  잔차=" << residual_per_sphere << " m/구"
      << "  x=" << x << " y=" << y << " z=" << z
      << " roll=" << roll << " pitch=" << pitch << " yaw=" << yaw;
  res->success = true;
  res->message = oss.str();
  RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
}

// ──────────────────────────── service: cube_calibration ──────────────────────

void LidarCalibrationNode::srvCubeCalibrate(
  const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  CloudPtr r1, r2;
  {
    std::lock_guard<std::mutex> lk(cloud_mutex_);
    // 큐브 검출·ICP 모두 누적 cloud 사용 (단일 프레임은 포인트 수 불안정)
    r1 = accumulated_raw1_;
    r2 = accumulated_raw2_;
  }

  // ICP용 클라우드: 누적 raw를 voxel(0.1m) 다운샘플링
  // (cached_cloud*는 최근 단일 프레임 → 프레임에 따라 2pts 등 극도로 희박할 수 있음)
  CloudPtr c1, c2;
  {
    pcl::VoxelGrid<PointT> vg;
    vg.setLeafSize(0.1f, 0.1f, 0.1f);
    c1 = std::make_shared<Cloud>();
    vg.setInputCloud(r1);
    vg.filter(*c1);
    c2 = std::make_shared<Cloud>();
    vg.setInputCloud(r2);
    vg.filter(*c2);
  }

  if (!r1 || r1->empty() || !r2 || r2->empty() || !c1 || !c2) {
    res->success = false;
    res->message = "아직 포인트 클라우드 수신 전입니다. 잠시 후 다시 시도하세요.";
    return;
  }

  RCLCPP_INFO(get_logger(), "=== 큐브 기반 캘리브레이션 시작 ===");
  RCLCPP_INFO(get_logger(), "  누적 프레임=%d  cloud1=%zu pts  cloud2=%zu pts",
    accum_count_, r1->size(), r2->size());
  RCLCPP_INFO(get_logger(), "  기대 큐브 크기: %.1f×%.1f×%.1f m  허용오차 ±%.1f m",
    cube_size_xy_, cube_size_xy_, cube_size_z_, cube_size_tolerance_);
  RCLCPP_INFO(get_logger(),
    "[CONFIG] cluster_tol=%.2f m  min_pts=%d  plane_inlier≥%.0f%%  min_dist=%.1f m",
    cube_cluster_tol_, cube_min_cluster_, cube_plane_inlier_thresh_ * 100.f, cube_min_distance_);
  RCLCPP_INFO(get_logger(),
    "[CONFIG] ICP: search_corr=%.2f m  refine_corr=%.2f m  max_iter=%d  fit_thr=%.4f",
    search_corr_dist_, refine_corr_dist_, max_iter_, fit_threshold_);

  RCLCPP_INFO(get_logger(), "[LiDAR1] 큐브 검출 중...");
  auto cubes1 = detectCubeCenters(r1);

  RCLCPP_INFO(get_logger(), "[LiDAR2] 큐브 검출 중...");
  auto cubes2 = detectCubeCenters(r2);

  RCLCPP_INFO(get_logger(),
    "검출 결과: LiDAR1=%zu개  LiDAR2=%zu개", cubes1.size(), cubes2.size());

  // 검출된 각 큐브 상세 (center, 원점거리, quality)
  auto logCubeList = [&](const std::vector<CubeTarget> & cubes, const char * label) {
    for (size_t i = 0; i < cubes.size(); ++i) {
      RCLCPP_INFO(get_logger(),
        "  [%s][%zu] center=(%.3f, %.3f, %.3f)  원점거리=%.2f m  quality=%.0f%%",
        label, i,
        cubes[i].center.x(), cubes[i].center.y(), cubes[i].center.z(),
        cubes[i].center.norm(), cubes[i].quality * 100.f);
    }
  };
  logCubeList(cubes1, "LiDAR1");
  logCubeList(cubes2, "LiDAR2");

  // plane fit fallback 경고
  for (size_t i = 0; i < cubes1.size(); ++i) {
    if (cubes1[i].quality < 0.01f) {
      RCLCPP_WARN(get_logger(),
        "  ⚠ LiDAR1 큐브[%zu]: plane fit 실패 → 무게중심 사용 (정확도 저하)", i);
    }
  }
  for (size_t i = 0; i < cubes2.size(); ++i) {
    if (cubes2[i].quality < 0.01f) {
      RCLCPP_WARN(get_logger(),
        "  ⚠ LiDAR2 큐브[%zu]: plane fit 실패 → 무게중심 사용 (정확도 저하)", i);
    }
  }

  if (cubes1.size() < 2 || cubes2.size() < 2) {
    std::ostringstream oss;
    oss << "큐브 검출 부족 (LiDAR1=" << cubes1.size()
        << "개, LiDAR2=" << cubes2.size() << "개). "
        << "최소 2개 필요. cube_size_tolerance 또는 큐브 배치 확인.";
    res->success = false;
    res->message = oss.str();
    RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
    return;
  }
  if (cubes1.size() < 3 || cubes2.size() < 3) {
    RCLCPP_WARN(get_logger(),
      "  ⚠ 큐브 2개로 진행 (3개 권장). 캘리브레이션 정확도가 낮을 수 있음.");
  }

  // 고품질 큐브만 추출 (plane fit 성공, inlier ≥ cube_plane_inlier_min)
  auto filter_quality = [&](const std::vector<CubeTarget> & cubes,
                             const std::string & label) {
    std::vector<CubeTarget> good;
    for (const auto & ct : cubes) {
      if (ct.quality >= static_cast<float>(cube_plane_inlier_thresh_)) {
        good.push_back(ct);
      } else {
        RCLCPP_WARN(get_logger(),
          "  [%s] 저품질 큐브 제외: quality=%.0f%% < %.0f%%",
          label.c_str(), ct.quality * 100.f,
          cube_plane_inlier_thresh_ * 100.f);
      }
    }
    return good;
  };

  auto good1 = filter_quality(cubes1, "LiDAR1");
  auto good2 = filter_quality(cubes2, "LiDAR2");
  RCLCPP_INFO(get_logger(),
    "  품질 필터 후: LiDAR1=%zu개  LiDAR2=%zu개",
    good1.size(), good2.size());

  // 고품질 큐브 중심 및 pairwise 거리 (Kabsch conditioning 진단)
  auto logPairwise = [&](const std::vector<CubeTarget> & cubes, const char * label) {
    for (size_t i = 0; i < cubes.size(); ++i) {
      RCLCPP_INFO(get_logger(),
        "  [%s 고품질][%zu] center=(%.3f, %.3f, %.3f)  원점거리=%.2f m",
        label, i,
        cubes[i].center.x(), cubes[i].center.y(), cubes[i].center.z(),
        cubes[i].center.norm());
    }
    for (size_t i = 0; i < cubes.size(); ++i) {
      for (size_t j = i + 1; j < cubes.size(); ++j) {
        const float d = (cubes[i].center - cubes[j].center).norm();
        RCLCPP_INFO(get_logger(),
          "  [%s] 큐브[%zu↔%zu] 간격=%.3f m", label, i, j, d);
      }
    }
  };
  logPairwise(good1, "LiDAR1");
  logPairwise(good2, "LiDAR2");

  if (good1.size() < 2 && good2.size() < 2) {
    std::ostringstream oss;
    oss << "고품질 큐브 부족 (LiDAR1=" << good1.size()
        << "개, LiDAR2=" << good2.size() << "개). "
        << "cube_plane_inlier_min(" << cube_plane_inlier_thresh_
        << ") 조정 또는 큐브 배치 확인.";
    res->success = false;
    res->message = oss.str();
    RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
    return;
  }
  if (good1.size() < 2 || good2.size() < 2) {
    // ── Fallback: 1개 큐브 무게중심으로 translation 추정 → multi-yaw ICP ──
    RCLCPP_WARN(get_logger(),
      "  ⚠ Kabsch 부족 (LiDAR1=%zu, LiDAR2=%zu) → 큐브 무게중심 translation + multi-yaw ICP로 전환",
      good1.size(), good2.size());

    // 각 LiDAR의 고품질 큐브 무게중심 계산
    Eigen::Vector3f c1_mean = Eigen::Vector3f::Zero();
    for (const auto & ct : good1) c1_mean += ct.center;
    if (!good1.empty()) c1_mean /= static_cast<float>(good1.size());

    Eigen::Vector3f c2_mean = Eigen::Vector3f::Zero();
    for (const auto & ct : good2) c2_mean += ct.center;
    if (!good2.empty()) c2_mean /= static_cast<float>(good2.size());

    // 대략적인 translation 초기값: dst - src (rotation 0 가정)
    const Eigen::Vector3f t_approx = (good1.empty() || good2.empty())
      ? Eigen::Vector3f(init_x_, init_y_, init_z_)
      : (c1_mean - c2_mean);

    RCLCPP_INFO(get_logger(),
      "  큐브 무게중심 translation 추정: x=%.3f y=%.3f z=%.3f",
      t_approx.x(), t_approx.y(), t_approx.z());

    // multi-yaw ICP (voxelized cached cloud 사용)
    CloudPtr ci1, ci2;
    {
      std::lock_guard<std::mutex> lk(cloud_mutex_);
      ci1 = cached_cloud1_;
      ci2 = cached_cloud2_;
    }
    if (!ci1 || !ci2) {
      res->success = false;
      res->message = "ICP용 cloud 미수신.";
      return;
    }

    Eigen::Matrix4f best_T;
    double best_score = std::numeric_limits<double>::max();
    bool found = false;
    const int n_yaw = yaw_candidates_;

    RCLCPP_INFO(get_logger(), "  multi-yaw ICP (%d방향)  t=(%.3f, %.3f, %.3f)...",
      n_yaw, t_approx.x(), t_approx.y(), t_approx.z());
    for (int i = 0; i < n_yaw; ++i) {
      const double yaw = i * (2.0 * M_PI / n_yaw);
      Eigen::Matrix4f g = rpyToMatrix(
        static_cast<double>(t_approx.x()),
        static_cast<double>(t_approx.y()),
        static_cast<double>(t_approx.z()), 0.0, 0.0, yaw);

      Eigen::Matrix4f cr, fr;
      double cs, fs;
      if (!runICP(ci2, ci1, g, cr, cs, search_corr_dist_)) continue;
      if (!runICP(ci2, ci1, cr, fr, fs, refine_corr_dist_)) { fr = cr; fs = cs; }

      RCLCPP_INFO(get_logger(), "    [%d/%d] yaw=%.0f°  score=%.4f%s",
        i+1, n_yaw, yaw*180/M_PI, fs, fs < best_score ? " ★" : "");
      if (fs < best_score) { best_score = fs; best_T = fr; found = true; }
    }

    if (!found || best_score > fit_threshold_) {
      std::ostringstream oss;
      oss << "ICP fallback 실패 (score=" << best_score << "). "
          << "LiDAR1에서 고품질 큐브 2개 이상 검출되도록 큐브를 재배치하세요.";
      res->success = false;
      res->message = oss.str();
      RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
      return;
    }

    current_T_     = best_T;
    current_score_ = best_score;
    icp_done_      = true;
    broadcastTransform(current_T_);
    double x, y, z, roll, pitch, yaw2;
    matrixToRpy(current_T_, x, y, z, roll, pitch, yaw2);
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6)
        << "큐브→ICP fallback 성공  score=" << best_score
        << "  x=" << x << " y=" << y << " z=" << z
        << " yaw=" << yaw2;
    res->success = true;
    res->message = oss.str();
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
    return;
  }

  // 이후 고품질 큐브만 사용
  const auto & use1 = good1;
  const auto & use2 = good2;
  const size_t n = std::min(use1.size(), use2.size());

  // 큐브 중심 벡터 추출
  std::vector<Eigen::Vector3f> centers1(n), centers2(n);
  for (size_t i = 0; i < n; ++i) {
    centers1[i] = use1[i].center;
    centers2[i] = use2[i].center;
  }

  // ── 직선 배치 검사 (3개 이상일 때만) ────────────────────────────────────
  if (n >= 3) {
    const Eigen::Vector3f v1 = use1[1].center - use1[0].center;
    const Eigen::Vector3f v2 = use1[2].center - use1[0].center;
    const float area = v1.cross(v2).norm() * 0.5f;
    RCLCPP_INFO(get_logger(), "  큐브 삼각형 면적 (LiDAR1 기준): %.1f m²", area);
    if (area < 5.0f) {
      RCLCPP_WARN(get_logger(),
        "  ⚠ 삼각형 면적 %.1f m² — 큐브 거의 직선 배치. Kabsch SVD 불안정 가능.", area);
    }
  } else {
    RCLCPP_INFO(get_logger(), "  2개 큐브로 Kabsch SVD 수행 (rotation 일부 미결정 → ICP 정제로 보완)");
  }

  // ── Kabsch SVD (순열 전수 탐색) ───────────────────────────────────────────
  RCLCPP_INFO(get_logger(), "Kabsch SVD 시작 (%zu개 큐브 중심)...", n);
  double residual;
  const Eigen::Matrix4f kabsch_T = matchAndSolve(centers2, centers1, residual);
  const double residual_per_cube = residual / static_cast<double>(n);

  {
    double kx, ky, kz, kr, kp, kyaw;
    matrixToRpy(kabsch_T, kx, ky, kz, kr, kp, kyaw);
    RCLCPP_INFO(get_logger(),
      "  Kabsch 결과: x=%.3f y=%.3f z=%.3f yaw=%.3f rad  잔차=%.4f m/큐브",
      kx, ky, kz, kyaw, residual_per_cube);
  }

  // 잔차가 큐브 한 변의 길이보다 크면 검출 오류
  if (residual_per_cube > cube_size_xy_) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4)
        << "Kabsch 잔차(" << residual_per_cube << " m/큐브) 과대 — "
        << "큐브 검출 오류 또는 잘못된 배치. 직선 배치 여부 확인.";
    res->success = false;
    res->message = oss.str();
    RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
    return;
  }

  // ── Kabsch 결과 물리 타당성 검증 ──────────────────────────────────────────
  // 지상 차량 캘리브레이션 가정: z 이동·roll·pitch 가 과도하면 spurious 해 의심
  {
    double kx, ky, kz, kr, kp, kyaw;
    matrixToRpy(kabsch_T, kx, ky, kz, kr, kp, kyaw);
    const double t_xy  = std::sqrt(kx * kx + ky * ky);
    const double t_z   = std::abs(kz);
    const double rp_max = std::max(std::abs(kr), std::abs(kp));
    bool suspicious = false;

    // z 이동이 수평 이동보다 크면 비정상 (센서가 같은 높이에 있어야 함)
    if (t_z > t_xy + 0.5) {
      RCLCPP_WARN(get_logger(),
        "  ★ [Kabsch 타당성] z이동(%.3f m) > xy이동(%.3f m) — "
        "spurious Ry(π) 해 가능성. 큐브 B·C 배치가 센서 중간점(x=%.1f)에 대해 "
        "대칭인지 확인하고 비대칭 배치로 수정하세요.",
        kz, t_xy, (init_x_ * 0.5));
      suspicious = true;
    }
    // roll 또는 pitch 가 크면 비정상
    if (rp_max > 0.5) {
      RCLCPP_WARN(get_logger(),
        "  ★ [Kabsch 타당성] roll=%.3f rad  pitch=%.3f rad — "
        "지상 차량에서 roll/pitch 과대. 센서 장착 방향 또는 큐브 배치 확인.",
        kr, kp);
      suspicious = true;
    }
    if (suspicious) {
      RCLCPP_WARN(get_logger(),
        "  ★ [Kabsch 타당성] 기대값: x≈%.1f y≈0 z≈0 yaw≈0"
        " / 실제: x=%.3f y=%.3f z=%.3f yaw=%.3f rad"
        "  → ICP로 이 초기값을 사용하면 틀린 결과가 나옵니다.",
        init_x_, kx, ky, kz, kyaw);
    } else {
      RCLCPP_INFO(get_logger(),
        "  [Kabsch 타당성] OK — z이동=%.3f m  roll=%.3f  pitch=%.3f rad",
        kz, kr, kp);
    }
  }

  // ── 지상차량 구속: roll·pitch 소거 → z 직접 계산 ─────────────────────────
  // 근거: 지상차량에서 LiDAR간 roll·pitch ≈ 0 (물리 가정).
  // Kabsch roll 원인: 큐브 face centroid z가 beam 커버리지 불균형으로
  // range-dependent bias를 가짐 → Kabsch가 이를 roll로 해석 →
  // roll × y_centroid ≈ -0.23m 의 허수 z-translation 발생.
  // 구속 후: R_fixed=Rot_z(yaw),  t_fixed=mean(dst_i - R_fixed·src_i).
  // t_fixed.z ≈ mean(Δz_i) 이므로 roll 오염 없이 순수 높이차를 추정.
  Eigen::Matrix4f icp_init_T = kabsch_T;  // 기본값: Kabsch
  {
    double kx, ky, kz, kr, kp, kyaw;
    matrixToRpy(kabsch_T, kx, ky, kz, kr, kp, kyaw);
    const double rp_thr = 0.10;  // [rad] ≈ 5.7°

    // per-pair Δz 진단: roll 기여 없이 순수 높이차 확인
    float dz_sum = 0.0f, yc_sum = 0.0f;
    RCLCPP_INFO(get_logger(), "  [z진단] 각 쌍 Δz=dst_z−src_z (roll 미포함, 순수 높이차):");
    for (size_t i = 0; i < n; ++i) {
      const float dz = centers1[i].z() - centers2[i].z();
      dz_sum += dz;
      yc_sum += centers2[i].y();
      RCLCPP_INFO(get_logger(),
        "    [쌍%zu] src_z=%.3f → dst_z=%.3f  Δz=%+.4f m",
        i, centers2[i].z(), centers1[i].z(), static_cast<double>(dz));
    }
    const float dz_mean = dz_sum / static_cast<float>(n);
    const float yc_mean = yc_sum / static_cast<float>(n);
    RCLCPP_INFO(get_logger(),
      "    Δz 평균=%+.4f m | roll=%.3f rad × y중심=%.2f m → z오염≈%.3f m | Kabsch z=%.3f m",
      static_cast<double>(dz_mean), kr, static_cast<double>(yc_mean),
      static_cast<double>(-static_cast<float>(kr) * yc_mean), kz);

    if (std::abs(kr) < rp_thr && std::abs(kp) < rp_thr) {
      // yaw만 남긴 순수 회전 행렬
      const float cy = std::cos(static_cast<float>(kyaw));
      const float sy = std::sin(static_cast<float>(kyaw));
      Eigen::Matrix4f R_yaw = Eigen::Matrix4f::Identity();
      R_yaw(0, 0) =  cy;  R_yaw(0, 1) = -sy;
      R_yaw(1, 0) =  sy;  R_yaw(1, 1) =  cy;

      // 각 쌍의 t_i = dst_i - R_yaw * src_i → 평균 = 순수 translation (z오염 없음)
      Eigen::Vector3f t_mean = Eigen::Vector3f::Zero();
      for (size_t i = 0; i < n; ++i) {
        Eigen::Vector4f sp(centers2[i].x(), centers2[i].y(), centers2[i].z(), 1.0f);
        t_mean += centers1[i] - (R_yaw * sp).head<3>();
      }
      t_mean /= static_cast<float>(n);

      icp_init_T = R_yaw;
      icp_init_T.block<3, 1>(0, 3) = t_mean;

      double gx, gy, gz, gr, gp, gyaw2;
      matrixToRpy(icp_init_T, gx, gy, gz, gr, gp, gyaw2);
      RCLCPP_INFO(get_logger(),
        "  [지상구속] 적용: x=%.3f y=%.3f z=%.3f yaw=%.4f rad"
        "  (Kabsch z=%.3f → 개선 Δ=%.3f m)",
        gx, gy, gz, gyaw2, kz, gz - kz);
    } else {
      RCLCPP_INFO(get_logger(),
        "  [지상구속] 미적용 — roll=%.3f pitch=%.3f rad (임계값 %.2f rad 초과)",
        kr, kp, rp_thr);
    }
  }

  // ── ICP 정제: 지상구속 결과 → coarse → fine ──────────────────────────────
  RCLCPP_INFO(get_logger(),
    "ICP 정제 시작 (c1=%zu pts  c2=%zu pts  coarse_corr=%.2f m  fine_corr=%.2f m)...",
    c1->size(), c2->size(), search_corr_dist_, refine_corr_dist_);

  Eigen::Matrix4f coarse_T;
  double coarse_score;
  const bool coarse_ok =
    runICP(c2, c1, icp_init_T, coarse_T, coarse_score, search_corr_dist_);

  if (coarse_ok) {
    RCLCPP_INFO(get_logger(), "  coarse ICP score=%.4f", coarse_score);
  } else {
    RCLCPP_WARN(get_logger(), "  coarse ICP 수렴 실패 → 지상구속 결과로 fine ICP 재시도");
    coarse_T     = icp_init_T;
    coarse_score = residual_per_cube;
  }

  Eigen::Matrix4f fine_T;
  double fine_score;
  const bool fine_ok =
    runICP(c2, c1, coarse_T, fine_T, fine_score, refine_corr_dist_);

  if (fine_ok) {
    RCLCPP_INFO(get_logger(), "  fine   ICP score=%.4f", fine_score);
  } else {
    RCLCPP_WARN(get_logger(), "  fine ICP 수렴 실패 → coarse 결과 사용");
    fine_T     = coarse_T;
    fine_score = coarse_score;
  }

  // Kabsch → fine ICP drift 진단 + drift guard
  // 누적 클라우드(3개 큐브 클러스터)로 ICP를 돌리면 서로 다른 큐브 면끼리
  // 잘못 대응하여 국소 최솟값에 빠질 수 있음 → drift가 크면 Kabsch 결과 사용
  {
    const float drift_limit = 0.15f;  // [m]
    const Eigen::Vector3f t_init = icp_init_T.block<3, 1>(0, 3);
    const Eigen::Vector3f t_fine = fine_T.block<3, 1>(0, 3);
    const float drift = (t_fine - t_init).norm();
    RCLCPP_INFO(get_logger(),
      "  [drift] init=(%.4f, %.4f, %.4f)  fine=(%.4f, %.4f, %.4f)  이동=%.4f m",
      t_init.x(), t_init.y(), t_init.z(),
      t_fine.x(), t_fine.y(), t_fine.z(), drift);
    if (drift > drift_limit) {
      RCLCPP_WARN(get_logger(),
        "  ★ ICP drift=%.3f m > %.3f m — 잘못된 면 대응 수렴 추정 → 지상구속 결과 사용",
        drift, drift_limit);
      fine_T     = icp_init_T;
      fine_score = residual_per_cube;
    }
  }

  // ICP가 모두 실패했으면 지상구속 결과로 저장
  if (!coarse_ok && !fine_ok) {
    RCLCPP_WARN(get_logger(), "  ICP 전체 실패 → 지상구속 결과로 저장");
    current_T_     = icp_init_T;
    current_score_ = residual_per_cube;
    icp_done_      = true;
    broadcastTransform(current_T_);

    double x, y, z, roll, pitch, yaw;
    matrixToRpy(current_T_, x, y, z, roll, pitch, yaw);
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6)
        << "큐브 캘리브레이션 (Kabsch only)  잔차=" << residual_per_cube << " m/큐브"
        << "  x=" << x << " y=" << y << " z=" << z
        << " roll=" << roll << " pitch=" << pitch << " yaw=" << yaw;
    res->success = true;
    res->message = oss.str();
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
    return;
  }

  if (fine_score > fit_threshold_) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4)
        << "ICP fitness(" << fine_score << ") > threshold(" << fit_threshold_ << "). "
        << "큐브 배치 또는 파라미터를 확인하세요.";
    res->success = false;
    res->message = oss.str();
    RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
    return;
  }

  current_T_     = fine_T;
  current_score_ = fine_score;
  icp_done_      = true;
  broadcastTransform(current_T_);

  double x, y, z, roll, pitch, yaw;
  matrixToRpy(current_T_, x, y, z, roll, pitch, yaw);

  RCLCPP_INFO(get_logger(), "══════════════════ 캘리브레이션 결과 요약 ══════════════════");
  RCLCPP_INFO(get_logger(), "  [검출] LiDAR1=%zu  LiDAR2=%zu  (고품질 %zu개 사용)",
    cubes1.size(), cubes2.size(), n);
  RCLCPP_INFO(get_logger(), "  [Kabsch ] 잔차=%.4f m/큐브", residual_per_cube);
  RCLCPP_INFO(get_logger(), "  [coarse ] score=%.4f  수렴=%s", coarse_score, coarse_ok ? "OK" : "FAIL");
  RCLCPP_INFO(get_logger(), "  [fine   ] score=%.4f  수렴=%s", fine_score,   fine_ok   ? "OK" : "FAIL");
  RCLCPP_INFO(get_logger(), "  [결과   ] x=%.4f y=%.4f z=%.4f  roll=%.4f pitch=%.4f yaw=%.4f rad",
    x, y, z, roll, pitch, yaw);
  RCLCPP_INFO(get_logger(), "════════════════════════════════════════════════════════════");

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6)
      << "큐브 캘리브레이션 성공  ICP_score=" << fine_score
      << "  x=" << x << " y=" << y << " z=" << z
      << " roll=" << roll << " pitch=" << pitch << " yaw=" << yaw;
  res->success = true;
  res->message = oss.str();
  RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
}

// ──────────────────────────── Multi-yaw ICP ──────────────────────────────────

bool LidarCalibrationNode::runMultiYawICP(
  const CloudPtr & source, const CloudPtr & target,
  double tx, double ty, double tz,
  Eigen::Matrix4f & best_result, double & best_score)
{
  best_score = std::numeric_limits<double>::max();
  bool found = false;

  RCLCPP_INFO(get_logger(), "다중 yaw ICP (%d방향, %.0f° 간격)  t=(%.3f, %.3f, %.3f)",
    yaw_candidates_, 360.0 / yaw_candidates_, tx, ty, tz);

  for (int i = 0; i < yaw_candidates_; ++i) {
    const double yaw = i * (2.0 * M_PI / yaw_candidates_);
    const Eigen::Matrix4f init_T = rpyToMatrix(tx, ty, tz, 0.0, 0.0, yaw);

    Eigen::Matrix4f coarse_result;
    double coarse_score;
    if (!runICP(source, target, init_T, coarse_result, coarse_score, search_corr_dist_)) {
      RCLCPP_INFO(get_logger(), "  [%d/%d] yaw=%.1f°  → 수렴 실패",
        i + 1, yaw_candidates_, yaw * 180.0 / M_PI);
      continue;
    }

    Eigen::Matrix4f fine_result;
    double fine_score;
    if (!runICP(source, target, coarse_result, fine_result, fine_score, refine_corr_dist_)) {
      fine_result = coarse_result;
      fine_score  = coarse_score;
    }

    const bool is_best = fine_score < best_score;
    RCLCPP_INFO(get_logger(), "  [%d/%d] yaw=%.1f°  coarse=%.4f  fine=%.4f%s",
      i + 1, yaw_candidates_, yaw * 180.0 / M_PI,
      coarse_score, fine_score, is_best ? "  ★" : "");

    if (is_best) {
      best_score  = fine_score;
      best_result = fine_result;
      found = true;
    }
  }
  return found;
}

bool LidarCalibrationNode::runICP(
  const CloudPtr & source, const CloudPtr & target,
  const Eigen::Matrix4f & initial_guess,
  Eigen::Matrix4f & result, double & fitness_score,
  double corr_dist)
{
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setInputSource(source);
  icp.setInputTarget(target);
  icp.setMaxCorrespondenceDistance(corr_dist);
  icp.setMaximumIterations(max_iter_);
  icp.setTransformationEpsilon(trans_eps_);
  icp.setEuclideanFitnessEpsilon(fit_eps_);

  Cloud aligned;
  icp.align(aligned, initial_guess);

  if (!icp.hasConverged()) return false;
  result        = icp.getFinalTransformation();
  fitness_score = icp.getFitnessScore();
  return true;
}

// ──────────────────────────── service: run_calibration (ICP) ─────────────────

void LidarCalibrationNode::srvCalibrate(
  const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  CloudPtr c1, c2;
  {
    std::lock_guard<std::mutex> lk(cloud_mutex_);
    c1 = cached_cloud1_;
    c2 = cached_cloud2_;
  }

  if (!c1 || !c2) {
    res->success = false;
    res->message = "아직 포인트 클라우드 수신 전입니다. 잠시 후 다시 시도하세요.";
    return;
  }

  RCLCPP_INFO(get_logger(), "다중 yaw ICP 캘리브레이션 시작 (cloud1=%zu pts, cloud2=%zu pts)",
    c1->size(), c2->size());

  // init_x_, init_y_ 의 절댓값을 기준으로 부호(4가지) × x/y swap(2가지) = 8 조합 전수 탐색
  const double ax = std::abs(init_x_);
  const double ay = std::abs(init_y_);
  const std::vector<std::pair<double, double>> tx_ty_list = {
    {-ax, -ay}, {+ax, +ay}, {-ax, +ay}, {+ax, -ay},   // 부호 조합
    {-ay, -ax}, {+ay, +ax}, {-ay, +ax}, {+ay, -ax},   // x/y 뒤바뀐 경우
  };

  Eigen::Matrix4f best_T;
  double best_score = std::numeric_limits<double>::max();
  bool found = false;

  for (size_t ci = 0; ci < tx_ty_list.size(); ++ci) {
    const double tx = tx_ty_list[ci].first;
    const double ty = tx_ty_list[ci].second;
    RCLCPP_INFO(get_logger(), "── 조합 [%zu/%zu]: tx=%.3f  ty=%.3f ──",
      ci + 1, tx_ty_list.size(), tx, ty);

    Eigen::Matrix4f cand_T;
    double cand_score;
    if (!runMultiYawICP(c2, c1, tx, ty, init_z_, cand_T, cand_score)) {
      continue;
    }
    RCLCPP_INFO(get_logger(), "  → score=%.4f%s",
      cand_score, cand_score < best_score ? "  ★ 최적 갱신" : "");
    if (cand_score < best_score) {
      best_score = cand_score;
      best_T     = cand_T;
      found      = true;
    }
  }

  if (!found) {
    res->success = false;
    res->message = "ICP 수렴 실패 (8개 조합 모두 실패). search_correspondence_distance 확대 필요.";
    return;
  }

  if (best_score > fit_threshold_) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4)
        << "최적 fitness(" << best_score << ") > threshold(" << fit_threshold_ << ").";
    res->success = false;
    res->message = oss.str();
    RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
    return;
  }

  current_T_     = best_T;
  current_score_ = best_score;
  icp_done_      = true;
  broadcastTransform(current_T_);

  double x, y, z, roll, pitch, yaw;
  matrixToRpy(current_T_, x, y, z, roll, pitch, yaw);

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(6)
      << "ICP 성공  score=" << best_score
      << "  x=" << x << " y=" << y << " z=" << z
      << " roll=" << roll << " pitch=" << pitch << " yaw=" << yaw;
  res->success = true;
  res->message = oss.str();
  RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
}

// ──────────────────────────── service: save ──────────────────────────────────

void LidarCalibrationNode::srvSave(
  const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  if (saveYaml(output_path_)) {
    res->success = true;
    res->message = "저장 완료: " + output_path_;
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  } else {
    res->success = false;
    res->message = "저장 실패: " + output_path_;
    RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
  }
}

// ──────────────────────────── YAML save ──────────────────────────────────────

bool LidarCalibrationNode::saveYaml(const std::string & path)
{
  double x, y, z, roll, pitch, yaw;
  matrixToRpy(current_T_, x, y, z, roll, pitch, yaw);
  const Eigen::Affine3f aff(current_T_);
  const Eigen::Quaternionf q(aff.rotation());

  try {
    std::filesystem::create_directories(std::filesystem::path(path).parent_path());
    std::ofstream ofs(path);
    if (!ofs.is_open()) return false;

    ofs << std::fixed << std::setprecision(8);
    ofs << "# LiDAR Extrinsic Calibration Result\n";
    ofs << "# parent: " << lidar1_frame_ << "  child: " << lidar2_frame_ << "\n";
    ofs << "# residual/score: " << current_score_ << "\n\n";
    ofs << "lidar_extrinsic:\n";
    ofs << "  parent_frame: \"" << lidar1_frame_ << "\"\n";
    ofs << "  child_frame:  \"" << lidar2_frame_ << "\"\n";
    ofs << "  translation:\n";
    ofs << "    x: " << x << "\n";
    ofs << "    y: " << y << "\n";
    ofs << "    z: " << z << "\n";
    ofs << "  rotation_rpy:  # radians, ZYX\n";
    ofs << "    roll:  " << roll  << "\n";
    ofs << "    pitch: " << pitch << "\n";
    ofs << "    yaw:   " << yaw   << "\n";
    ofs << "  rotation_quaternion:\n";
    ofs << "    x: " << static_cast<double>(q.x()) << "\n";
    ofs << "    y: " << static_cast<double>(q.y()) << "\n";
    ofs << "    z: " << static_cast<double>(q.z()) << "\n";
    ofs << "    w: " << static_cast<double>(q.w()) << "\n";
    ofs << "  matrix4x4:  # row-major\n";
    for (int r = 0; r < 4; ++r) {
      ofs << "    - [";
      for (int c = 0; c < 4; ++c) {
        ofs << static_cast<double>(current_T_(r, c));
        if (c < 3) ofs << ", ";
      }
      ofs << "]\n";
    }
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "YAML 저장 오류: %s", e.what());
    return false;
  }
}

// ──────────────────────────── math helpers ───────────────────────────────────

Eigen::Matrix4f LidarCalibrationNode::rpyToMatrix(
  double x, double y, double z,
  double roll, double pitch, double yaw)
{
  Eigen::Affine3f t = Eigen::Affine3f::Identity();
  t.translation() << static_cast<float>(x),
                     static_cast<float>(y),
                     static_cast<float>(z);
  t.rotate(Eigen::AngleAxisf(static_cast<float>(yaw),   Eigen::Vector3f::UnitZ()));
  t.rotate(Eigen::AngleAxisf(static_cast<float>(pitch), Eigen::Vector3f::UnitY()));
  t.rotate(Eigen::AngleAxisf(static_cast<float>(roll),  Eigen::Vector3f::UnitX()));
  return t.matrix();
}

void LidarCalibrationNode::matrixToRpy(
  const Eigen::Matrix4f & mat,
  double & x, double & y, double & z,
  double & roll, double & pitch, double & yaw)
{
  const Eigen::Affine3f aff(mat);
  x = static_cast<double>(aff.translation().x());
  y = static_cast<double>(aff.translation().y());
  z = static_cast<double>(aff.translation().z());
  const Eigen::Matrix3f R = aff.rotation();
  pitch = std::asin(static_cast<double>(-R(2, 0)));
  if (std::abs(std::cos(pitch)) > 1e-6) {
    roll = std::atan2(static_cast<double>(R(2, 1)), static_cast<double>(R(2, 2)));
    yaw  = std::atan2(static_cast<double>(R(1, 0)), static_cast<double>(R(0, 0)));
  } else {
    roll = 0.0;
    yaw  = std::atan2(static_cast<double>(-R(0, 1)), static_cast<double>(R(1, 1)));
  }
}

// ──────────────────────────── TF broadcast ───────────────────────────────────

void LidarCalibrationNode::broadcastTransform(const Eigen::Matrix4f & T)
{
  const Eigen::Affine3f aff(T);
  const Eigen::Quaternionf q(aff.rotation());

  geometry_msgs::msg::TransformStamped ts;
  ts.header.stamp    = now();
  ts.header.frame_id = lidar1_frame_;
  ts.child_frame_id  = lidar2_frame_;
  ts.transform.translation.x = static_cast<double>(aff.translation().x());
  ts.transform.translation.y = static_cast<double>(aff.translation().y());
  ts.transform.translation.z = static_cast<double>(aff.translation().z());
  ts.transform.rotation.x    = static_cast<double>(q.x());
  ts.transform.rotation.y    = static_cast<double>(q.y());
  ts.transform.rotation.z    = static_cast<double>(q.z());
  ts.transform.rotation.w    = static_cast<double>(q.w());
  static_tf_br_->sendTransform(ts);
}

// ──────────────────────────── publish clouds ─────────────────────────────────

void LidarCalibrationNode::publishClouds(
  const CloudPtr & cloud1, const CloudPtr & cloud2_aligned,
  const rclcpp::Time & stamp)
{
  {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud2_aligned, msg);
    msg.header.stamp    = stamp;
    msg.header.frame_id = lidar1_frame_;
    pub_aligned_->publish(msg);
  }
  {
    CloudPtr merged = std::make_shared<Cloud>(*cloud1);
    *merged += *cloud2_aligned;
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*merged, msg);
    msg.header.stamp    = stamp;
    msg.header.frame_id = lidar1_frame_;
    pub_merged_->publish(msg);
  }
}

}  // namespace lidar_calib

// ──────────────────────────── main ───────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_calib::LidarCalibrationNode>());
  rclcpp::shutdown();
  return 0;
}
