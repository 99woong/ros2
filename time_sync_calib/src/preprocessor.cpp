#include "time_sync_calib/preprocessor.hpp"

#include <algorithm>
#include <numeric>
#include <stdexcept>
#include <iostream>
#include <cmath>

namespace time_sync_calib
{

Preprocessor::Preprocessor(const Config& cfg) : cfg_(cfg) {}

// -------------------------------------------------------
// 메인 처리 흐름
// -------------------------------------------------------
ResampledSignals Preprocessor::process(const ImuData& internal,
                                        const ImuData& external) const
{
  // -------------------------------------------------------
  // 1. 공통 시간 구간 추출
  // -------------------------------------------------------
  double t_start = std::max(internal.timestamps.front(),
                             external.timestamps.front());
  double t_end   = std::min(internal.timestamps.back(),
                             external.timestamps.back());

  if (t_end <= t_start) {
    throw std::runtime_error("[Preprocessor] 공통 시간 구간 없음");
  }

  double common_duration = t_end - t_start;

  if (cfg_.verbose) {
    std::cout << "[Preprocessor] 공통 구간: "
              << common_duration << "초" << std::endl;
  }

  // -------------------------------------------------------
  // 2. 공통 시간축 생성
  // -------------------------------------------------------
  double dt = 1.0 / cfg_.resample_rate_hz;
  std::vector<double> common_time;
  common_time.reserve(static_cast<size_t>(common_duration * cfg_.resample_rate_hz) + 1);

  for (double t = t_start; t <= t_end; t += dt) {
    common_time.push_back(t);
  }

  if (cfg_.verbose) {
    std::cout << "[Preprocessor] 리샘플링: "
              << common_time.size() << "개 샘플 "
              << "@ " << cfg_.resample_rate_hz << " Hz" << std::endl;
  }

  // -------------------------------------------------------
  // 3. 선형 보간
  // -------------------------------------------------------
  auto int_wz = linear_interp(internal.timestamps,
                               internal.omega_z,
                               common_time);
  auto ext_wz = linear_interp(external.timestamps,
                               external.omega_z,
                               common_time);

  // -------------------------------------------------------
  // 4. LPF (선택)
  // -------------------------------------------------------
  if (cfg_.apply_lpf) {
    apply_lpf(int_wz, dt);
    apply_lpf(ext_wz, dt);

    if (cfg_.verbose) {
      std::cout << "[Preprocessor] LPF 적용: "
                << cfg_.lpf_cutoff_hz << " Hz" << std::endl;
    }
  }

  // -------------------------------------------------------
  // 5. DC 제거 (평균 빼기)
  // -------------------------------------------------------
  remove_dc(int_wz);
  remove_dc(ext_wz);

  // -------------------------------------------------------
  // 6. 신호 품질 확인
  // -------------------------------------------------------
  double max_int = *std::max_element(int_wz.begin(), int_wz.end());
  double min_int = *std::min_element(int_wz.begin(), int_wz.end());
  double max_ext = *std::max_element(ext_wz.begin(), ext_wz.end());
  double min_ext = *std::min_element(ext_wz.begin(), ext_wz.end());

  double range_int = max_int - min_int;
  double range_ext = max_ext - min_ext;

  if (cfg_.verbose) {
    std::cout << "[Preprocessor] 신호 범위"  << std::endl;
    std::cout << "  내부 IMU: [" << min_int
              << ", " << max_int << "] rad/s"
              << " (range: " << range_int << ")" << std::endl;
    std::cout << "  외부 IMU: [" << min_ext
              << ", " << max_ext << "] rad/s"
              << " (range: " << range_ext << ")" << std::endl;
  }

  // 신호가 너무 작으면 경고
  constexpr double MIN_RANGE = 0.05;  // rad/s
  if (range_int < MIN_RANGE) {
    std::cerr << "[Preprocessor] ⚠️  내부 IMU 신호가 너무 작습니다 "
              << "(range: " << range_int << " rad/s). "
              << "더 빠른 회전이 필요합니다." << std::endl;
  }
  if (range_ext < MIN_RANGE) {
    std::cerr << "[Preprocessor] ⚠️  외부 IMU 신호가 너무 작습니다 "
              << "(range: " << range_ext << " rad/s). "
              << "더 빠른 회전이 필요합니다." << std::endl;
  }

  ResampledSignals result;
  result.common_time  = std::move(common_time);
  result.internal_wz  = std::move(int_wz);
  result.external_wz  = std::move(ext_wz);
  result.dt           = dt;

  return result;
}

// -------------------------------------------------------
// 선형 보간
// -------------------------------------------------------
std::vector<double> Preprocessor::linear_interp(
    const std::vector<double>& t_in,
    const std::vector<double>& y_in,
    const std::vector<double>& t_out) const
{
  std::vector<double> y_out;
  y_out.reserve(t_out.size());

  size_t idx = 0;

  for (double t : t_out) 
  {
    // t보다 작은 마지막 인덱스 찾기
    while (idx + 1 < t_in.size() && t_in[idx + 1] <= t) 
    {
      ++idx;
    }

    if (idx + 1 >= t_in.size()) 
    {
      // 범위 밖: 마지막 값으로 채움
      y_out.push_back(y_in.back());
    } 
    else if (t <= t_in[idx]) 
    {
      // 범위 밖: 첫 번째 값으로 채움
      y_out.push_back(y_in.front());
    } 
    else 
    {
      // 선형 보간
      double alpha = (t - t_in[idx]) / (t_in[idx + 1] - t_in[idx]);
      y_out.push_back(y_in[idx] + alpha * (y_in[idx + 1] - y_in[idx]));
    }
  }

  return y_out;
}

// -------------------------------------------------------
// DC 제거
// -------------------------------------------------------
void Preprocessor::remove_dc(std::vector<double>& signal) const
{
  if (signal.empty()) return;

  double mean = std::accumulate(signal.begin(), signal.end(), 0.0)
              / static_cast<double>(signal.size());

  for (auto& v : signal) v -= mean;
}

// -------------------------------------------------------
// 1차 IIR 저주파 통과 필터
//   y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
//   alpha = dt / (dt + RC)
//   RC = 1 / (2 * pi * cutoff_hz)
// -------------------------------------------------------
void Preprocessor::apply_lpf(std::vector<double>& signal, double dt) const
{
  if (signal.empty()) return;

  const double RC    = 1.0 / (2.0 * M_PI * cfg_.lpf_cutoff_hz);
  const double alpha = dt / (dt + RC);

  double y_prev = signal[0];

  for (size_t i = 1; i < signal.size(); i++) 
  {
    y_prev     = alpha * signal[i] + (1.0 - alpha) * y_prev;
    signal[i]  = y_prev;
  }
}

}  // namespace time_sync_calib