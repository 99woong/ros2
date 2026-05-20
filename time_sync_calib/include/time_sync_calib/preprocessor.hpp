#pragma once
#include "time_sync_calib/types.hpp"

namespace time_sync_calib
{

// -------------------------------------------------------
// 리샘플링된 신호 (공통 시간축)
// -------------------------------------------------------
struct ResampledSignals
{
  std::vector<double> common_time;   // 공통 시간축 [sec]
  std::vector<double> internal_wz;   // 리샘플된 내부 IMU omega_z
  std::vector<double> external_wz;   // 리샘플된 외부 IMU omega_z
  double              dt;            // 샘플 간격 [sec]
};

// -------------------------------------------------------
// 전처리 클래스
//   1. 공통 시간 구간 추출
//   2. 선형 보간으로 리샘플링
//   3. DC 제거 (평균 빼기)
//   4. (선택) 저주파 통과 필터
// -------------------------------------------------------
class Preprocessor
{
public:
  explicit Preprocessor(const Config& cfg);

  // internal, external 데이터를 받아 리샘플된 신호 반환
  ResampledSignals process(const ImuData& internal,
                           const ImuData& external) const;

private:
  const Config& cfg_;

  // 선형 보간
  std::vector<double> linear_interp(
      const std::vector<double>& t_in,
      const std::vector<double>& y_in,
      const std::vector<double>& t_out) const;

  // DC 제거 (평균 빼기)
  void remove_dc(std::vector<double>& signal) const;

  // 1차 IIR 저주파 통과 필터
  void apply_lpf(std::vector<double>& signal, double dt) const;
};

}  // namespace time_sync_calib