#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <numeric>
#include <iomanip>

// 수학 상수
constexpr double PI = 3.14159265358979323846;

// 측정된 센서 데이터를 담는 구조체
struct Measurement {
    double gls_x;         // 바닥 마커 센서 X
    double gls_y;         // 바닥 마커 센서 Y
    double gls_heading;   // 바닥 마커 센서 헤딩 (라디안)
    double vslam_x;       // VSLAM 글로벌 X
    double vslam_y;       // VSLAM 글로벌 Y
    double vslam_heading; // VSLAM 글로벌 헤딩 (라디안)
};

// 계산된 외부 보정 파라미터를 담는 구조체
struct ExtrinsicParams {
    double delta_x;     // 평행 이동 X 오프셋 (VSLAM 기준)
    double delta_y;     // 평행 이동 Y 오프셋 (VSLAM 기준)
    double delta_heading; // 회전 오프셋 (VSLAM - GLS, 라디안)
};

// 잔차 및 통계를 담는 구조체
struct EvaluationResult {
    double mean_tx_residual;
    double mean_ty_residual;
    double mean_r_residual;
    double rmse_tx;
    double rmse_ty;
    double rmse_r;
    size_t count;
};

// === 헬퍼 함수: Degree를 Radian으로 변환 ===
double degrees_to_radians(double degrees) {
    return degrees * PI / 180.0;
}

// === 헬퍼 함수: CSV 파일 읽기 ===

std::vector<Measurement> read_csv(const std::string& filepath) {
    std::vector<Measurement> data;
    std::ifstream file(filepath);
    std::string line;

    if (!file.is_open()) {
        std::cerr << "오류: CSV 파일을 열 수 없습니다: " << filepath << std::endl;
        return data;
    }

    // 첫 번째 줄 (헤더) 건너뛰기
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string segment;
        Measurement m;
        std::vector<double> values;

        while (std::getline(ss, segment, ',')) {
            try {
                values.push_back(std::stod(segment));
            } catch (const std::exception& e) {
                std::cerr << "경고: 유효하지 않은 숫자 데이터 감지 및 건너뛰기: " << segment << std::endl;
                continue;
            }
        }

        if (values.size() >= 6) {
            m.gls_x = values[7];
            m.gls_y = values[8];
            // 헤딩 입력이 Degree이므로, Radian으로 변환하여 저장
            m.gls_heading = degrees_to_radians(values[9]);
            m.vslam_x = values[1];
            m.vslam_y = values[2];
            // 헤딩 입력이 Degree이므로, Radian으로 변환하여 저장
            m.vslam_heading = degrees_to_radians(values[3]);
            data.push_back(m);
        }
    }
    std::cout << "성공: 총 " << data.size() << "개의 측정 데이터를 불러왔습니다." << std::endl;
    return data;
}

// === 헬퍼 함수: 각도 정규화 ([-PI, PI] 범위로) ===
// 이 함수는 VSLAM의 [0, 2PI] (Degree 기준으로는 [0, 360]) 각도를 GLS와 동일한 [-PI, PI] 범위로 변환하는 데 사용됩니다.
double normalize_angle(double angle) {
    angle = std::fmod(angle + PI, 2.0 * PI);
    if (angle < 0) {
        angle += 2.0 * PI;
    }
    return angle - PI;
}

// === 1단계: 외부 보정 파라미터 계산 ===

ExtrinsicParams calculate_extrinsic_params(const std::vector<Measurement>& data) {
    ExtrinsicParams params = {0.0, 0.0, 0.0};
    if (data.empty()) return params;

    // 1. 헤딩 오프셋 (Delta Heading) 계산
    double sum_sin = 0.0;
    double sum_cos = 0.0;

    for (const auto& m : data) {
        // VSLAM 헤딩 (0~2PI)를 GLS와 동일한 [-PI, PI] 범위로 변환합니다.
        double normalized_vslam_heading = normalize_angle(m.vslam_heading);
        
        // VSLAM - GLS 의 각도 차이
        double diff = normalized_vslam_heading - m.gls_heading;
        sum_sin += std::sin(diff);
        sum_cos += std::cos(diff);
    }

    // 각도 차이의 평균을 atan2를 사용하여 안정적으로 계산
    params.delta_heading = std::atan2(sum_sin / data.size(), sum_cos / data.size());

    // 2. 평행 이동 오프셋 (Delta X, Delta Y) 계산
    double sum_tx = 0.0;
    double sum_ty = 0.0;
    
    double cos_dh = std::cos(params.delta_heading);
    double sin_dh = std::sin(params.delta_heading);

    for (const auto& m : data) {
        // Step 2-1: GLS 좌표를 Delta Heading만큼 회전시켜 VSLAM 좌표계 방향으로 정렬 (R * P_gls)
        
        // P_gls를 VSLAM 프레임으로 변환했을 때의 예상 위치 (회전 보상)
        double gls_rotated_x = m.gls_x * cos_dh - m.gls_y * sin_dh;
        double gls_rotated_y = m.gls_x * sin_dh + m.gls_y * cos_dh;

        // Step 2-2: VSLAM 글로벌 위치와 회전 보상된 GLS 위치의 차이(평행 이동)를 누적
        // t = P_vslam - (R * P_gls)
        double current_tx = m.vslam_x - gls_rotated_x;
        double current_ty = m.vslam_y - gls_rotated_y;

        sum_tx += current_tx;
        sum_ty += current_ty;
    }

    // 평행 이동 오프셋의 평균 계산
    params.delta_x = sum_tx / data.size();
    params.delta_y = sum_ty / data.size();

    return params;
}

// === 2단계: 보정 파라미터 적용 및 잔차 평가 ===

EvaluationResult evaluate_calibration(const std::vector<Measurement>& data, const ExtrinsicParams& params) {
    EvaluationResult result = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, data.size()};

    double sq_sum_tx_res = 0.0;
    double sq_sum_ty_res = 0.0;
    double sq_sum_r_res = 0.0;
    
    double sum_tx_res = 0.0;
    double sum_ty_res = 0.0;

    double cos_dh = std::cos(params.delta_heading);
    double sin_dh = std::sin(params.delta_heading);

    for (const auto& m : data) {
        // GLS 포즈에 외부 보정 파라미터 적용하여 VSLAM 추정 포즈 계산
        
        // 1. GLS 좌표에 회전 변환 적용
        double gls_calib_x = m.gls_x * cos_dh - m.gls_y * sin_dh;
        double gls_calib_y = m.gls_x * sin_dh + m.gls_y * cos_dh;

        // 2. 평행 이동 변환 적용
        gls_calib_x += params.delta_x;
        gls_calib_y += params.delta_y;

        // 3. 헤딩 변환 적용
        double gls_calib_heading = normalize_angle(m.gls_heading + params.delta_heading);

        // 잔차 계산 (VSLAM 실제 측정값 - 보정된 GLS 측정값)
        
        // 위치 잔차 (X, Y)
        double tx_residual = m.vslam_x - gls_calib_x;
        double ty_residual = m.vslam_y - gls_calib_y;
        
        // 회전 잔차 (Heading)
        // VSLAM 헤딩을 먼저 [-PI, PI]로 변환하여 보정된 GLS 헤딩과 비교합니다.
        double normalized_vslam_heading = normalize_angle(m.vslam_heading);
        double r_diff = normalize_angle(normalized_vslam_heading - gls_calib_heading);
        
        // 통계 누적
        sum_tx_res += tx_residual;
        sum_ty_res += ty_residual;
        
        sq_sum_tx_res += tx_residual * tx_residual;
        sq_sum_ty_res += ty_residual * ty_residual;
        sq_sum_r_res += r_diff * r_diff;
    }

    // 평균 잔차 (Mean Residual) - 이론적으로 0에 가까워야 함
    result.mean_tx_residual = sum_tx_res / result.count;
    result.mean_ty_residual = sum_ty_res / result.count;
    // 평균 회전 잔차 (회전은 이미 보정 과정에서 평균화했으므로, 제곱 평균으로만 평가)
    
    // RMSE (Root Mean Square Error)
    result.rmse_tx = std::sqrt(sq_sum_tx_res / result.count);
    result.rmse_ty = std::sqrt(sq_sum_ty_res / result.count);
    result.rmse_r = std::sqrt(sq_sum_r_res / result.count);

    return result;
}

// === 메인 함수 ===

int main() {
    // 사용자에게 파일 경로 입력 요청
    std::string filename = "calibration_data.csv";
    std::cout << "================================================" << std::endl;
    std::cout << "차량 센서 외부 보정 및 일관성 평가 프로그램" << std::endl;
    std::cout << "================================================" << std::endl;
    std::cout << "CSV 파일 경로를 입력하세요 (예: calibration_data.csv): ";
    // std::cin >> filename;
    
    // 테스트를 위해 파일명을 하드코딩합니다. 실제 사용 시 위 주석을 해제하십시오.
    filename = "calibration_data.csv"; 

    // 1. 데이터 로드
    std::vector<Measurement> data = read_csv(filename);

    if (data.size() < 2) {
        std::cerr << "오류: 보정을 위한 충분한 데이터(최소 2개)가 없습니다." << std::endl;
        return 1;
    }

    // 2. 외부 보정 파라미터 계산
    ExtrinsicParams params = calculate_extrinsic_params(data);

    // 3. 결과 출력
    std::cout << "\n================================================" << std::endl;
    std::cout << "1. 계산된 외부 보정 파라미터 (Extrinsic Parameters)" << std::endl;
    std::cout << "================================================" << std::endl;
    std::cout << std::fixed << std::setprecision(5);
    std::cout << "회전 오프셋 (Delta Heading, 라디안): " << params.delta_heading << std::endl;
    std::cout << "회전 오프셋 (Delta Heading, 도):     " << params.delta_heading * 180.0 / PI << " deg" << std::endl;
    std::cout << "평행 이동 오프셋 X (Delta X):        " << params.delta_x << " m" << std::endl;
    std::cout << "평행 이동 오프셋 Y (Delta Y):        " << params.delta_y << " m" << std::endl;

    // 4. 보정 파라미터 적용 및 잔차 평가
    EvaluationResult eval_result = evaluate_calibration(data, params);

    std::cout << "\n================================================" << std::endl;
    std::cout << "2. 잔차 및 일관성 평가 결과 (Residuals & Consistency)" << std::endl;
    std::cout << "(보정 파라미터를 적용한 후의 VSLAM 대비 오차)" << std::endl;
    std::cout << "================================================" << std::endl;
    
    // RMSE (일관성 평가 지표)
    std::cout << "[일관성 지표: RMSE (Root Mean Square Error)]" << std::endl;
    std::cout << "X 축 RMSE: " << eval_result.rmse_tx << " m" << std::endl;
    std::cout << "Y 축 RMSE: " << eval_result.rmse_ty << " m" << std::endl;
    std::cout << "헤딩 RMSE: " << eval_result.rmse_r << " rad (" << eval_result.rmse_r * 180.0 / PI << " deg)" << std::endl;
    std::cout << "\n" << std::endl;

    // 잔차 평균 (Mean Residual) - 보정의 성공 여부 확인용
    std::cout << "[보정 성공 지표: 평균 잔차 (Mean Residual)]" << std::endl;
    std::cout << "X 축 평균 잔차 (Mean X Residual): " << eval_result.mean_tx_residual << " m (0에 가까워야 함)" << std::endl;
    std::cout << "Y 축 평균 잔차 (Mean Y Residual): " << eval_result.mean_ty_residual << " m (0에 가까워야 함)" << std::endl;
    
    std::cout << "\n------------------------------------------------" << std::endl;
    std::cout << "평가 요약: RMSE 값이 작을수록 두 센서 측정치의 일관성이 높음을 의미합니다." << std::endl;
    std::cout << "------------------------------------------------" << std::endl;

    return 0;
}
