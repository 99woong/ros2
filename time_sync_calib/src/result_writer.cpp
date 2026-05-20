#include "time_sync_calib/result_writer.hpp"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <numeric>

namespace time_sync_calib
{

ResultWriter::ResultWriter(const Config& cfg) : cfg_(cfg) {}

// -------------------------------------------------------
// 터미널 요약 출력
// -------------------------------------------------------
void ResultWriter::print_summary(const XCorrResult& result) const
{
  const double offset_ms = result.time_offset_sec * 1000.0;

  std::cout << "\n";
  std::cout << "╔══════════════════════════════════════════╗\n";
  std::cout << "║        시간동기 검증 결과                ║\n";
  std::cout << "╚══════════════════════════════════════════╝\n";

  // 오프셋
  std::cout << std::fixed << std::setprecision(3);
  std::cout << "  시간 오프셋 (정밀): "
            << offset_ms << " ms\n";
  std::cout << "  시간 오프셋 (정수): "
            << result.time_offset_coarse_sec * 1000.0 << " ms\n";

  // 부호 해석
  if (offset_ms > 0.0) {
    std::cout << "  → 외부 IMU가 내부 IMU보다 "
              << std::abs(offset_ms) << " ms 늦음\n";
  } else if (offset_ms < 0.0) {
    std::cout << "  → 외부 IMU가 내부 IMU보다 "
              << std::abs(offset_ms) << " ms 빠름\n";
  } else {
    std::cout << "  → 두 IMU의 시간이 완벽히 일치\n";
  }

  // 상관계수
  std::cout << std::setprecision(4);
  std::cout << "  최대 상관계수: " << result.max_correlation << "\n";

  // 신뢰도
  if (result.is_good()) {
    std::cout << "  신뢰도: ✓ 높음 (> 0.9)\n";
  } else if (result.is_reliable()) {
    std::cout << "  신뢰도: △ 보통 (0.7 ~ 0.9)\n";
  } else {
    std::cout << "  신뢰도: ✗ 낮음 (< 0.7) → 데이터 재수집 권장\n";
  }

  // 동기 상태 평가
  double abs_offset_ms = std::abs(offset_ms);
  if (abs_offset_ms < 2.0) {
    std::cout << "  동기 상태: ✓ 매우 양호 (< 2ms)\n";
  } else if (abs_offset_ms < 5.0) {
    std::cout << "  동기 상태: ✓ 양호 (< 5ms)\n";
  } else if (abs_offset_ms < 20.0) {
    std::cout << "  동기 상태: △ 보통 (5 ~ 20ms) → 보정 권장\n";
  } else {
    std::cout << "  동기 상태: ✗ 불량 (> 20ms) → 하드웨어 점검 필요\n";
  }

  // FAST-LIO2 설정값
  std::cout << "\n";
  std::cout << "  ──────────────────────────────────────\n";
  std::cout << "  FAST-LIO2 설정 적용값:\n";
  std::cout << "  time_offset_lidar_to_imu: "
            << std::setprecision(6)
            << (-result.time_offset_sec) << "\n";
  std::cout << "  ──────────────────────────────────────\n";
  std::cout << "\n";

  // 주의: 부호 설명
  std::cout << "  [부호 정의]\n";
  std::cout << "  time_offset_sec > 0: external이 internal보다 늦음\n";
  std::cout << "                       external_time = internal_time + offset\n";
  std::cout << "  FAST-LIO2의 time_offset_lidar_to_imu에는 부호 반전하여 적용\n";
  std::cout << "\n";
}

// -------------------------------------------------------
// CSV 저장
// -------------------------------------------------------
void ResultWriter::save_csv(const XCorrResult&      result,
                             const ResampledSignals&  signals,
                             const ImuData&           internal,
                             const ImuData&           external) const
{
  // -------------------------------------------------------
  // (1) cross-correlation 함수 저장
  //     xcorr_result.csv
  //     → PlotJuggler로 열어서 피크 위치 확인
  // -------------------------------------------------------
  {
    std::string xcorr_path = "xcorr_result.csv";
    std::ofstream f(xcorr_path);

    if (!f.is_open()) {
      std::cerr << "[ResultWriter] 파일 열기 실패: " << xcorr_path << std::endl;
      return;
    }

    f << "lag_ms,correlation\n";
    f << std::fixed << std::setprecision(6);

    for (size_t i = 0; i < result.lags_sec.size(); ++i) {
      f << result.lags_sec[i] * 1000.0 << ","
        << result.corr_values[i]        << "\n";
    }

    std::cout << "[ResultWriter] Cross-correlation 저장: "
              << xcorr_path << std::endl;
  }

  // -------------------------------------------------------
  // (2) 두 신호 비교 (보정 전/후) 저장
  //     aligned_signals.csv
  //     → PlotJuggler로 열어서 신호 정렬 확인
  //
  //     컬럼:
  //       time_sec       : 공통 시간축
  //       internal_wz    : 내부 IMU omega_z (리샘플링)
  //       external_wz    : 외부 IMU omega_z (보정 전)
  //       external_wz_aligned : 외부 IMU omega_z (보정 후)
  // -------------------------------------------------------
  {
    std::string signal_path = "aligned_signals.csv";
    std::ofstream f(signal_path);

    if (!f.is_open()) {
      std::cerr << "[ResultWriter] 파일 열기 실패: "
                << signal_path << std::endl;
      return;
    }

    // 외부 IMU를 time_offset만큼 shift하여 재보간
    // external_time_shifted = external_time - time_offset
    // → 공통 시간축에서 보정된 신호 계산
    const double offset = result.time_offset_sec;
    const auto&  ext_t  = external.timestamps;
    const auto&  ext_wz = external.omega_z;
    const auto&  com_t  = signals.common_time;

    std::vector<double> ext_wz_aligned;
    ext_wz_aligned.reserve(com_t.size());

    size_t idx = 0;
    for (double t : com_t) {
      // 보정된 외부 IMU 시간으로 쿼리
      // external_at_t_corrected = external(t + offset) 와 동일
      double t_query = t + offset;

      while (idx + 1 < ext_t.size() && ext_t[idx + 1] <= t_query) {
        ++idx;
      }

      double wz_aligned;
      if (idx + 1 >= ext_t.size()) {
        wz_aligned = ext_wz.back();
      } else if (t_query <= ext_t[0]) {
        wz_aligned = ext_wz.front();
      } else {
        double alpha = (t_query - ext_t[idx])
                     / (ext_t[idx + 1] - ext_t[idx]);
        wz_aligned = ext_wz[idx] + alpha * (ext_wz[idx + 1] - ext_wz[idx]);
      }
      ext_wz_aligned.push_back(wz_aligned);
    }

    // DC 제거
    double mean_aligned = std::accumulate(
        ext_wz_aligned.begin(), ext_wz_aligned.end(), 0.0)
        / static_cast<double>(ext_wz_aligned.size());
    for (auto& v : ext_wz_aligned) v -= mean_aligned;

    // CSV 쓰기
    f << "time_sec,internal_wz,external_wz,external_wz_aligned\n";
    f << std::fixed << std::setprecision(6);

    double t0 = com_t.front();
    for (size_t i = 0; i < com_t.size(); ++i) {
      f << (com_t[i] - t0)              << ","
        << signals.internal_wz[i]       << ","
        << signals.external_wz[i]       << ","
        << ext_wz_aligned[i]            << "\n";
    }

    std::cout << "[ResultWriter] 신호 비교 저장: "
              << signal_path << std::endl;
  }

  // -------------------------------------------------------
  // (3) 원본 데이터 저장 (다운샘플 없이)
  //     raw_imu_comparison.csv
  //     → 원본 샘플링 레이트 차이 확인용
  // -------------------------------------------------------
  {
    std::string raw_path = "raw_imu_comparison.csv";
    std::ofstream f(raw_path);

    if (!f.is_open()) {
      std::cerr << "[ResultWriter] 파일 열기 실패: "
                << raw_path << std::endl;
      return;
    }

    // 내부 IMU 원본 (100Hz)
    f << "# 내부 IMU 원본 데이터\n";
    f << "internal_time_sec,internal_wz\n";
    f << std::fixed << std::setprecision(9);

    double t0 = std::min(internal.timestamps.front(),
                          external.timestamps.front());
    for (size_t i = 0; i < internal.timestamps.size(); ++i) {
      f << (internal.timestamps[i] - t0) << ","
        << internal.omega_z[i]           << "\n";
    }

    // 외부 IMU 원본 (500Hz)
    f << "\n# 외부 IMU 원본 데이터\n";
    f << "external_time_sec,external_wz\n";
    for (size_t i = 0; i < external.timestamps.size(); ++i) {
      f << (external.timestamps[i] - t0) << ","
        << external.omega_z[i]           << "\n";
    }

    std::cout << "[ResultWriter] 원본 데이터 저장: "
              << raw_path << std::endl;
  }

  std::cout << "\n[PlotJuggler 사용법]\n";
  std::cout << "  1. ros2 run plotjuggler plotjuggler\n";
  std::cout << "  2. File → Load Data File → xcorr_result.csv 열기\n";
  std::cout << "     → correlation vs lag_ms 그래프로 피크 확인\n";
  std::cout << "  3. File → Load Data File → aligned_signals.csv 열기\n";
  std::cout << "     → internal_wz, external_wz, external_wz_aligned\n";
  std::cout << "       세 신호 동시에 plot하여 정렬 확인\n";
}

}  // namespace time_sync_calib