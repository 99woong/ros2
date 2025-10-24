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
    double gls_x;         // 변환된 GLS X (VSLAM 호환)
    double gls_y;         // 변환된 GLS Y (VSLAM 호환)
    double gls_heading;   // 변환된 GLS 헤딩 (VSLAM 호환, 라디안)
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

// === 헬퍼 함수: 각도를 [-PI, PI] 범위로 정규화 ===
double normalize_angle_to_pi_range(double angle) {
    // 2PI의 배수를 더하거나 빼서 [-PI, PI] 범위로 맞춥니다.
    angle = std::fmod(angle, 2.0 * PI);
    if (angle > PI) {
        angle -= 2.0 * PI;
    } else if (angle <= -PI) {
        angle += 2.0 * PI;
    }
    return angle;
}

// === 헬퍼 함수: CSV 파일 읽기 (GLS 데이터를 VSLAM 호환 프레임으로 변환) ===
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

    std::cout << "input data : " << std::endl;

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
            // 원본 데이터:
            // VSLAM (RHS, X-Forward): values[0], values[1], values[2] (Degree)
            // GLS (LHS, Y-Forward): values[3], values[4], values[5] (Degree)
            
            // 1. VSLAM 데이터는 그대로 사용 (VSLAM은 RHS, X-Forward이므로 기준이 됨)
            m.vslam_x = values[0];
            m.vslam_y = values[1];
            // ********** VSLAM Heading Degree -> Radian 변환 적용 **********
            m.vslam_heading = degrees_to_radians(values[2]); 

            // 2. GLS 데이터를 VSLAM 호환 좌표계(RHS, X-Forward)로 변환
            
            // 2-1. 위치 변환 (90도 CW 회전):
            // GLS Y (Forward) -> VSLAM X (Forward)
            // GLS X (Left) -> VSLAM -Y (Right)
            m.gls_x = values[4]; // GLS Y -> X'
            m.gls_y = -values[3]; // GLS X -> -Y'
            
            // 2-2. 헤딩 변환 (축 정렬 + 회전 규약 변환)
            // a) GLS 원본 헤딩 (Degree)을 Radian으로 변환
            double gls_heading_rad = degrees_to_radians(values[5]); // Degree -> Radian 변환 적용
            
            // b) 축 정렬: 90도 CW 회전 (RHS 기준: -PI/2)
            double gls_axis_aligned_heading = normalize_angle_to_pi_range(gls_heading_rad - PI/2.0); 
            
            // c) 회전 규약 변환 (LHS -> RHS): 각도의 부호를 반전
            m.gls_heading = normalize_angle_to_pi_range(-gls_axis_aligned_heading); 
            // =======================================================================

            data.push_back(m);
        }
        std::cout << m.vslam_x << " " << m.vslam_y << " " << m.vslam_heading << " " << m.gls_x <<" "<< m.gls_y <<" " << m.gls_heading << std::endl;
    }
    std::cout << "성공: 총 " << data.size() << "개의 측정 데이터를 불러왔습니다." << std::endl;
    return data;
}

// === 1단계: 외부 보정 파라미터 계산 ===
ExtrinsicParams calculate_extrinsic_params(const std::vector<Measurement>& data) {
    ExtrinsicParams params = {0.0, 0.0, 0.0};
    if (data.empty()) return params;

    // 1. 헤딩 오프셋 (Delta Heading) 계산
    double sum_sin = 0.0;
    double sum_cos = 0.0;

    for (const auto& m : data) {
        // VSLAM - GLS 의 각도 차이
        double diff = m.vslam_heading - m.gls_heading;
        
        // 각도 차이를 [-PI, PI] 범위로 정규화합니다.
        diff = normalize_angle_to_pi_range(diff);
        
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
        // Step 2-1: GLS 좌표를 Delta Heading만큼 회전시켜 VSLAM 좌표계 방향으로 정렬 
        // GLS 데이터가 이미 VSLAM과 동일한 RHS, X-Forward 기준으로 변환되었으므로, 
        // 표준 RHS 회전 행렬을 사용합니다.
        // RHS Rotation Matrix: [cos(h) -sin(h); sin(h) cos(h)]
        double gls_rotated_x = m.gls_x * cos_dh - m.gls_y * sin_dh;
        double gls_rotated_y = m.gls_x * sin_dh + m.gls_y * cos_dh;

        // Step 2-2: VSLAM 글로벌 위치와 회전 보상된 GLS 위치의 차이(평행 이동)를 누적
        // t = P_vslam - (R_RHS * P_gls)
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
        
        // 1. GLS 좌표에 회전 변환 적용 (RHS 행렬 사용)
        double gls_calib_x = m.gls_x * cos_dh - m.gls_y * sin_dh;
        double gls_calib_y = m.gls_x * sin_dh + m.gls_y * cos_dh;

        // 2. 평행 이동 변환 적용
        gls_calib_x += params.delta_x;
        gls_calib_y += params.delta_y;

        // 3. 헤딩 변환 적용
        // GLS 헤딩에 오프셋을 더함
        double gls_calib_heading = m.gls_heading + params.delta_heading;

        // 잔차 계산 (VSLAM 실제 측정값 - 보정된 GLS 측정값)
        
        // 위치 잔차 (X, Y)
        double tx_residual = m.vslam_x - gls_calib_x;
        double ty_residual = m.vslam_y - gls_calib_y;
        
        // 회전 잔차 (Heading)
        double r_diff = m.vslam_heading - gls_calib_heading;
        
        // 최종 잔차만 [-PI, PI]로 정규화하여 큰 오차를 방지
        r_diff = normalize_angle_to_pi_range(r_diff);
        
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
    
    // RMSE (Root Mean Square Error)
    result.rmse_tx = std::sqrt(sq_sum_tx_res / result.count);
    result.rmse_ty = std::sqrt(sq_sum_ty_res / result.count);
    result.rmse_r = std::sqrt(sq_sum_r_res / result.count);

    return result;
}

// === 메인 함수 ===

int main(int argc, char** argv) 
{
    std::string filename = "calibration_data.csv";
    if (argc > 1) {
        filename = argv[1];
    }

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
