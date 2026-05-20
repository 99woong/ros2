#pragma once
#include "time_sync_calib/types.hpp"
#include "time_sync_calib/preprocessor.hpp"

namespace time_sync_calib
{

// -------------------------------------------------------
// Cross-correlation 계산 클래스
// -------------------------------------------------------
class XCorr
{
public:
  explicit XCorr(const Config& cfg);

  // 리샘플된 신호로부터 시간 오프셋 계산
  XCorrResult compute(const ResampledSignals& signals) const;

private:
  const Config& cfg_;

  // -------------------------------------------------------
  // 정규화된 cross-correlation 계산
  //   R[k] = sum(x[n] * y[n+k]) / (std_x * std_y * N)
  // -------------------------------------------------------
  std::vector<double> normalized_xcorr(
      const std::vector<double>& x,
      const std::vector<double>& y,
      int max_lag_samples) const;

  // -------------------------------------------------------
  // 포물선 fitting으로 sub-sample 정밀도 피크 위치 추정
  //   3점: y_{-1}, y_0, y_1  →  정점 위치 = -b/(2a)
  // -------------------------------------------------------
  double parabola_peak_offset(double y_minus1,
                               double y_0,
                               double y_plus1) const;

  // -------------------------------------------------------
  // 표준편차 계산
  // -------------------------------------------------------
  double stddev(const std::vector<double>& v) const;
};

}  // namespace time_sync_calib