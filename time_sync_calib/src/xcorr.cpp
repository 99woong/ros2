#include "time_sync_calib/xcorr.hpp"

#include <algorithm>
#include <numeric>
#include <cmath>
#include <stdexcept>
#include <iostream>

namespace time_sync_calib
{

XCorr::XCorr(const Config& cfg) : cfg_(cfg) {}

// -------------------------------------------------------
// 메인: 시간 오프셋 계산
// -------------------------------------------------------
XCorrResult XCorr::compute(const ResampledSignals& signals) const
{
  const auto& x  = signals.internal_wz;   // 기준 신호 (내부 IMU)
  const auto& y  = signals.external_wz;   // 비교 신호 (외부 IMU)
  const double dt = signals.dt;

  if (x.empty() || y.empty()) {
    throw std::runtime_error("[XCorr] 신호가 비어 있습니다");
  }

  // -------------------------------------------------------
  // 1. Cross-correlation 계산
  //    R[k] = sum(x[n] * y[n+k]) / (std_x * std_y * N)
  //
  //    k > 0: y가 x보다 k*dt만큼 앞서 있음 (external이 빠름)
  //    k < 0: y가 x보다 |k|*dt만큼 늦음  (external이 느림)
  // -------------------------------------------------------
  int max_lag_samples = static_cast<int>(cfg_.max_lag_sec / dt);

  auto corr_values = normalized_xcorr(x, y, max_lag_samples);

  // lag 시간축 생성 [-max_lag, ..., 0, ..., +max_lag]
  std::vector<double> lags_sec;
  lags_sec.reserve(corr_values.size());
  for (int k = -max_lag_samples; k <= max_lag_samples; ++k) {
    lags_sec.push_back(static_cast<double>(k) * dt);
  }

  // -------------------------------------------------------
  // 2. 정수 lag 피크 탐색
  // -------------------------------------------------------
  auto max_it   = std::max_element(corr_values.begin(), corr_values.end());
  int  peak_idx = static_cast<int>(std::distance(corr_values.begin(), max_it));

  double max_corr         = *max_it;
  double coarse_lag_sec   = lags_sec[peak_idx];

  if (cfg_.verbose) {
    std::cout << "[XCorr] 정수 lag 피크: "
              << coarse_lag_sec * 1000.0 << " ms"
              << "  (상관계수: " << max_corr << ")" << std::endl;
  }

  // -------------------------------------------------------
  // 3. 포물선 fitting으로 sub-sample 정밀도 추정
  //
  //    피크 주변 3점으로 포물선 y = a*k^2 + b*k + c를 fitting
  //    정점: k_peak = -b / (2*a)
  //    최종 lag = coarse_lag + k_peak * dt
  // -------------------------------------------------------
  double precise_lag_sec = coarse_lag_sec;

  if (cfg_.use_parabola_fit &&
      peak_idx > 0 &&
      peak_idx < static_cast<int>(corr_values.size()) - 1)
  {
    double y_m1 = corr_values[peak_idx - 1];
    double y_0  = corr_values[peak_idx];
    double y_p1 = corr_values[peak_idx + 1];

    double k_offset = parabola_peak_offset(y_m1, y_0, y_p1);

    // sub-sample 오프셋이 유효한 범위인지 확인 (|offset| < 1)
    if (std::abs(k_offset) < 1.0) {
      precise_lag_sec = coarse_lag_sec + k_offset * dt;

      if (cfg_.verbose) {
        std::cout << "[XCorr] 포물선 fitting 보정: "
                  << k_offset * dt * 1000.0 << " ms"
                  << " → 최종: " << precise_lag_sec * 1000.0 << " ms"
                  << std::endl;
      }
    } else {
      std::cerr << "[XCorr] ⚠️  포물선 fitting 결과 범위 초과 ("
                << k_offset << "), 정수 lag 사용" << std::endl;
    }
  }

  // -------------------------------------------------------
  // 4. 결과 정리
  //
  //    부호 정의:
  //      precise_lag_sec > 0: y(external)가 x(internal)보다 늦음
  //                           external_time = internal_time + lag
  //      precise_lag_sec < 0: y(external)가 x(internal)보다 빠름
  // -------------------------------------------------------
  XCorrResult result;
  result.time_offset_sec        = precise_lag_sec;
  result.time_offset_coarse_sec = coarse_lag_sec;
  result.max_correlation        = max_corr;
  result.lags_sec               = std::move(lags_sec);
  result.corr_values            = std::move(corr_values);

  return result;
}

// -------------------------------------------------------
// 정규화된 cross-correlation
//   R[k] = (1/N) * sum_{n}(x[n] * y[n+k]) / (std_x * std_y)
//
//   k 범위: [-max_lag_samples, +max_lag_samples]
//   총 2*max_lag_samples + 1개의 값 반환
// -------------------------------------------------------
std::vector<double> XCorr::normalized_xcorr(
    const std::vector<double>& x,
    const std::vector<double>& y,
    int max_lag_samples) const
{
  const int N = static_cast<int>(x.size());

  // 표준편차 계산
  double std_x = stddev(x);
  double std_y = stddev(y);

  // 분모가 0인 경우 처리
  if (std_x < 1e-10 || std_y < 1e-10) {
    std::cerr << "[XCorr] ⚠️  신호의 표준편차가 0에 가깝습니다 "
              << "(std_x=" << std_x << ", std_y=" << std_y << ")"
              << std::endl;
    return std::vector<double>(2 * max_lag_samples + 1, 0.0);
  }

  double denom = std_x * std_y;

  // Cross-correlation 계산
  // k > 0: y가 x보다 k 샘플 앞서 있다고 가정했을 때의 상관
  //        sum_{n}(x[n] * y[n+k])
  // k < 0: y가 x보다 |k| 샘플 늦다고 가정
  //        sum_{n}(x[n] * y[n+k]) = sum_{n}(x[n-k] * y[n]) (인덱스 재정렬)

  std::vector<double> corr;
  corr.reserve(2 * max_lag_samples + 1);

  for (int k = -max_lag_samples; k <= max_lag_samples; ++k) {
    double sum   = 0.0;
    int    count = 0;

    int n_start = std::max(0, -k);
    int n_end   = std::min(N, N - k);

    for (int n = n_start; n < n_end; ++n) {
      sum += x[n] * y[n + k];
      ++count;
    }

    if (count > 0) {
      corr.push_back(sum / (denom * count));
    } else {
      corr.push_back(0.0);
    }
  }

  return corr;
}

// -------------------------------------------------------
// 포물선 fitting으로 sub-sample 피크 위치 추정
//
//   3점 (k=-1, 0, +1)에서 포물선 y = a*x^2 + b*x + c fitting
//   k=-1: a - b + c = y_minus1
//   k= 0:         c = y_0
//   k=+1: a + b + c = y_plus1
//
//   풀면:
//     c = y_0
//     a = (y_minus1 - 2*y_0 + y_plus1) / 2
//     b = (y_plus1 - y_minus1) / 2
//
//   정점: k_peak = -b / (2*a)
// -------------------------------------------------------
double XCorr::parabola_peak_offset(double y_minus1,
                                    double y_0,
                                    double y_plus1) const
{
  double a = (y_minus1 - 2.0 * y_0 + y_plus1) / 2.0;
  double b = (y_plus1  - y_minus1)              / 2.0;

  // a가 0이거나 양수면 최솟값이므로 오류
  if (std::abs(a) < 1e-15 || a > 0.0) {
    return 0.0;
  }

  return -b / (2.0 * a);
}

// -------------------------------------------------------
// 표준편차 계산
// -------------------------------------------------------
double XCorr::stddev(const std::vector<double>& v) const
{
  if (v.empty()) return 0.0;

  double mean = std::accumulate(v.begin(), v.end(), 0.0)
              / static_cast<double>(v.size());

  double sq_sum = 0.0;
  for (double x : v) {
    double diff = x - mean;
    sq_sum += diff * diff;
  }

  return std::sqrt(sq_sum / static_cast<double>(v.size()));
}

}  // namespace time_sync_calib