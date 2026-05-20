#pragma once
#include "time_sync_calib/types.hpp"
#include "time_sync_calib/preprocessor.hpp"

namespace time_sync_calib
{

// -------------------------------------------------------
// 결과 출력 클래스
//   - 터미널 출력 (요약)
//   - CSV 저장 (PlotJuggler 시각화용)
// -------------------------------------------------------
class ResultWriter
{
public:
  explicit ResultWriter(const Config& cfg);

  // 터미널에 결과 요약 출력
  void print_summary(const XCorrResult& result) const;

  // CSV 파일 저장 (PlotJuggler로 확인 가능)
  //   - xcorr_result.csv    : cross-correlation 함수
  //   - aligned_signals.csv : 보정 전/후 신호 비교
  void save_csv(const XCorrResult&     result,
                const ResampledSignals& signals,
                const ImuData&         internal,
                const ImuData&         external) const;

private:
  const Config& cfg_;
};

}  // namespace time_sync_calib