#include "time_sync_calib/bag_reader.hpp"

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <iostream>
#include <stdexcept>

namespace time_sync_calib
{

BagReader::BagReader(const Config& cfg) : cfg_(cfg) {}

bool BagReader::read(ImuData& internal_out, ImuData& external_out)
{
  internal_out = ImuData{};
  external_out = ImuData{};

  // -------------------------------------------------------
  // bag 열기
  // -------------------------------------------------------
  rosbag2_storage::StorageOptions storage_opts;
  storage_opts.uri        = cfg_.bag_path;
  storage_opts.storage_id = "sqlite3";

  rosbag2_cpp::ConverterOptions converter_opts;
  converter_opts.input_serialization_format  = "cdr";
  converter_opts.output_serialization_format = "cdr";

  rosbag2_cpp::Reader reader;
  try {
    reader.open(storage_opts, converter_opts);
  } catch (const std::exception& e) {
    std::cerr << "[BagReader] bag 열기 실패: " << e.what() << std::endl;
    return false;
  }

  // -------------------------------------------------------
  // 토픽 확인
  // -------------------------------------------------------
  auto topic_list = reader.get_all_topics_and_types();
  bool found_internal = false;
  bool found_external = false;

  for (const auto& t : topic_list) {
    if (t.name == cfg_.internal_imu_topic) found_internal = true;
    if (t.name == cfg_.external_imu_topic) found_external = true;
  }

  if (!found_internal) {
    std::cerr << "[BagReader] 내부 IMU 토픽 없음: "
              << cfg_.internal_imu_topic << std::endl;
    return false;
  }
  if (!found_external) {
    std::cerr << "[BagReader] 외부 IMU 토픽 없음: "
              << cfg_.external_imu_topic << std::endl;
    return false;
  }

  // -------------------------------------------------------
  // 메시지 읽기
  // -------------------------------------------------------
  rclcpp::Serialization<sensor_msgs::msg::Imu> serializer;

  size_t cnt_internal = 0;
  size_t cnt_external = 0;

  while (reader.has_next()) {
    auto bag_msg = reader.read_next();

    const bool is_internal = (bag_msg->topic_name == cfg_.internal_imu_topic);
    const bool is_external = (bag_msg->topic_name == cfg_.external_imu_topic);

    if (!is_internal && !is_external) continue;

    // 디시리얼라이즈
    sensor_msgs::msg::Imu imu_msg;
    rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
    serializer.deserialize_message(&serialized_msg, &imu_msg);

    // timestamp (nanosec → sec)
    double t = static_cast<double>(imu_msg.header.stamp.sec)
             + static_cast<double>(imu_msg.header.stamp.nanosec) * 1e-9;

    double wz = imu_msg.angular_velocity.z;

    if (is_internal) {
      internal_out.timestamps.push_back(t);
      internal_out.omega_z.push_back(wz);
      ++cnt_internal;
    } else {
      external_out.timestamps.push_back(t);
      external_out.omega_z.push_back(wz);
      ++cnt_external;
    }
  }

  // -------------------------------------------------------
  // 결과 확인
  // -------------------------------------------------------
  if (internal_out.empty()) {
    std::cerr << "[BagReader] 내부 IMU 데이터 없음" << std::endl;
    return false;
  }
  if (external_out.empty()) {
    std::cerr << "[BagReader] 외부 IMU 데이터 없음" << std::endl;
    return false;
  }

  if (cfg_.verbose) {
    std::cout << "[BagReader] 읽기 완료" << std::endl;
    std::cout << "  내부 IMU: " << cnt_internal << "개 샘플, "
              << internal_out.duration() << "초"
              << " (평균 " << internal_out.avg_rate() << " Hz)" << std::endl;
    std::cout << "  외부 IMU: " << cnt_external << "개 샘플, "
              << external_out.duration() << "초"
              << " (평균 " << external_out.avg_rate() << " Hz)" << std::endl;
  }

  return true;
}

}  // namespace time_sync_calib