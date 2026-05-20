#pragma once
#include <vector>
#include <string>
#include <cstdint>

namespace time_sync_calib
{

// -------------------------------------------------------
// IMU 샘플 하나
// -------------------------------------------------------
struct ImuSample
{
  double timestamp;   // Unix time [sec]
  double omega_z;     // angular_velocity.z [rad/s]
};

// -------------------------------------------------------
// IMU 시계열 데이터
// -------------------------------------------------------
struct ImuData
{
  std::vector<double> timestamps;  // [sec]
  std::vector<double> omega_z;     // [rad/s]

  size_t size() const { return timestamps.size(); }
  bool   empty() const { return timestamps.empty(); }

  double duration() const
  {
    if (size() < 2) return 0.0;
    return timestamps.back() - timestamps.front();
  }

  double avg_rate() const
  {
    if (size() < 2) return 0.0;
    return static_cast<double>(size() - 1) / duration();
  }
};

// -------------------------------------------------------
// Cross-correlation 결과
// -------------------------------------------------------
struct XCorrResult
{
  // 정밀 시간 오프셋 (포물선 fitting 적용)
  double time_offset_sec;     // [sec]  양수: external이 internal보다 늦음

  // 정수 lag에서의 오프셋
  double time_offset_coarse_sec;

  // 최대 상관계수 (정규화, -1 ~ 1)
  double max_correlation;

  // cross-correlation 함수 전체 (시각화용)
  std::vector<double> lags_sec;
  std::vector<double> corr_values;

  // 신뢰도 평가
  bool   is_reliable() const { return max_correlation > 0.7; }
  bool   is_good()     const { return max_correlation > 0.9; }
};

// -------------------------------------------------------
// 설정 파라미터
// -------------------------------------------------------
struct Config
{
  // 입력
  std::string bag_path;
  std::string internal_imu_topic = "/ouster/imu";
  std::string external_imu_topic = "/imu/data";

  // 전처리
  double resample_rate_hz   = 500.0;   // 공통 리샘플링 레이트
  double lpf_cutoff_hz      = 10.0;    // 저주파 통과 필터 컷오프
  bool   apply_lpf          = true;    // LPF 적용 여부

  // Cross-correlation
  double max_lag_sec        = 0.1;     // 탐색 범위 ±100ms
  bool   use_parabola_fit   = true;    // 포물선 fitting 사용

  // 출력
  std::string output_csv    = "xcorr_result.csv";
  bool        verbose       = true;
};

}  // namespace time_sync_calib