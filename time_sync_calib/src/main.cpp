#include "time_sync_calib/types.hpp"
#include "time_sync_calib/bag_reader.hpp"
#include "time_sync_calib/preprocessor.hpp"
#include "time_sync_calib/xcorr.hpp"
#include "time_sync_calib/result_writer.hpp"

#include <iostream>
#include <string>
#include <stdexcept>

// -------------------------------------------------------
// 사용법 출력
// -------------------------------------------------------
static void print_usage(const char* prog)
{
  std::cout << "\n사용법:\n";
  std::cout << "  " << prog << " <bag_path> [옵션]\n\n";
  std::cout << "옵션:\n";
  std::cout << "  --internal_topic  <topic>   내부 IMU 토픽 (기본: /ouster/imu)\n";
  std::cout << "  --external_topic  <topic>   외부 IMU 토픽 (기본: /imu/data)\n";
  std::cout << "  --resample_hz     <hz>      리샘플링 레이트 (기본: 500.0)\n";
  std::cout << "  --max_lag_ms      <ms>      최대 탐색 범위 ms (기본: 100.0)\n";
  std::cout << "  --lpf_cutoff_hz   <hz>      LPF 컷오프 (기본: 10.0)\n";
  std::cout << "  --no_lpf                    LPF 비활성화\n";
  std::cout << "  --no_parabola               포물선 fitting 비활성화\n";
  std::cout << "  --quiet                     최소 출력\n\n";
  std::cout << "예시:\n";
  std::cout << "  " << prog << " ./time_sync_test\n";
  std::cout << "  " << prog << " ./time_sync_test --max_lag_ms 50\n";
  std::cout << "  " << prog << " ./time_sync_test --resample_hz 200 --no_lpf\n\n";
}

// -------------------------------------------------------
// 커맨드라인 파싱
// -------------------------------------------------------
static time_sync_calib::Config parse_args(int argc, char* argv[])
{
  if (argc < 2) {
    print_usage(argv[0]);
    throw std::invalid_argument("bag 경로가 필요합니다");
  }

  time_sync_calib::Config cfg;
  cfg.bag_path = argv[1];

  for (int i = 2; i < argc; ++i) {
    std::string arg = argv[i];

    if (arg == "--internal_topic" && i + 1 < argc) {
      cfg.internal_imu_topic = argv[++i];
    } else if (arg == "--external_topic" && i + 1 < argc) {
      cfg.external_imu_topic = argv[++i];
    } else if (arg == "--resample_hz" && i + 1 < argc) {
      cfg.resample_rate_hz = std::stod(argv[++i]);
    } else if (arg == "--max_lag_ms" && i + 1 < argc) {
      cfg.max_lag_sec = std::stod(argv[++i]) / 1000.0;
    } else if (arg == "--lpf_cutoff_hz" && i + 1 < argc) {
      cfg.lpf_cutoff_hz = std::stod(argv[++i]);
    } else if (arg == "--no_lpf") {
      cfg.apply_lpf = false;
    } else if (arg == "--no_parabola") {
      cfg.use_parabola_fit = false;
    } else if (arg == "--quiet") {
      cfg.verbose = false;
    } else {
      std::cerr << "알 수 없는 옵션: " << arg << std::endl;
    }
  }

  return cfg;
}

// -------------------------------------------------------
// main
// -------------------------------------------------------
int main(int argc, char* argv[])
{
  // 설정 파싱
  time_sync_calib::Config cfg;
  try {
    cfg = parse_args(argc, argv);
  } catch (const std::exception& e) {
    std::cerr << "오류: " << e.what() << std::endl;
    return 1;
  }

  if (cfg.verbose) {
    std::cout << "\n========================================\n";
    std::cout << "  IMU 시간동기 검증 (Cross-correlation)\n";
    std::cout << "========================================\n\n";
    std::cout << "설정:\n";
    std::cout << "  bag 경로       : " << cfg.bag_path          << "\n";
    std::cout << "  내부 IMU 토픽  : " << cfg.internal_imu_topic << "\n";
    std::cout << "  외부 IMU 토픽  : " << cfg.external_imu_topic << "\n";
    std::cout << "  리샘플링       : " << cfg.resample_rate_hz   << " Hz\n";
    std::cout << "  탐색 범위      : ±" << cfg.max_lag_sec * 1000.0 << " ms\n";
    std::cout << "  LPF            : "
              << (cfg.apply_lpf ? "ON " : "OFF")
              << " (" << cfg.lpf_cutoff_hz << " Hz)\n";
    std::cout << "  포물선 fitting : "
              << (cfg.use_parabola_fit ? "ON" : "OFF") << "\n\n";
  }

  // -------------------------------------------------------
  // Step 1: 데이터 로드
  // -------------------------------------------------------
  if (cfg.verbose) {
    std::cout << "Step 1: bag 파일 읽기...\n";
  }

  time_sync_calib::ImuData internal_imu, external_imu;

  time_sync_calib::BagReader reader(cfg);
  if (!reader.read(internal_imu, external_imu)) {
    std::cerr << "데이터 로드 실패" << std::endl;
    return 1;
  }

  // 데이터 품질 사전 확인
  if (internal_imu.duration() < 5.0) {
    std::cerr << "⚠️  내부 IMU 데이터가 너무 짧습니다 ("
              << internal_imu.duration() << "초). "
              << "최소 10초 이상을 권장합니다." << std::endl;
  }
  if (external_imu.duration() < 5.0) {
    std::cerr << "⚠️  외부 IMU 데이터가 너무 짧습니다 ("
              << external_imu.duration() << "초)." << std::endl;
  }

  // -------------------------------------------------------
  // Step 2: 전처리
  // -------------------------------------------------------
  if (cfg.verbose) {
    std::cout << "\nStep 2: 전처리 (리샘플링, LPF, DC 제거)...\n";
  }

  time_sync_calib::ResampledSignals signals;
  try {
    time_sync_calib::Preprocessor preprocessor(cfg);
    signals = preprocessor.process(internal_imu, external_imu);
  } catch (const std::exception& e) {
    std::cerr << "전처리 실패: " << e.what() << std::endl;
    return 1;
  }

  // -------------------------------------------------------
  // Step 3: Cross-correlation 계산
  // -------------------------------------------------------
  if (cfg.verbose) {
    std::cout << "\nStep 3: Cross-correlation 계산...\n";
  }

  time_sync_calib::XCorrResult result;
  try {
    time_sync_calib::XCorr xcorr(cfg);
    result = xcorr.compute(signals);
  } catch (const std::exception& e) {
    std::cerr << "Cross-correlation 실패: " << e.what() << std::endl;
    return 1;
  }

  // -------------------------------------------------------
  // Step 4: 결과 출력 및 저장
  // -------------------------------------------------------
  time_sync_calib::ResultWriter writer(cfg);
  writer.print_summary(result);
  writer.save_csv(result, signals, internal_imu, external_imu);

  // -------------------------------------------------------
  // Step 5: 판단 및 종료 코드
  // -------------------------------------------------------
  if (!result.is_reliable()) {
    std::cerr << "⚠️  신뢰도가 낮습니다. 회전 데이터를 다시 수집하세요.\n";
    return 2;
  }

  return 0;
}