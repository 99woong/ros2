#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <numeric>
#include <iomanip>
#include <algorithm> // for std::abs

// 수학 상수
constexpr double PI = 3.14159265358979323846;

// [참고] GLS 위치 데이터를 VSLAM 호환 좌표계로 변환할 때 스케일 보정 상수는 1.0으로 유지합니다.
constexpr double GLS_POSITION_CORRECTION_FACTOR = 1.0; 


// 측정된 센서 데이터를 담는 구조체 (Residuals도 저장하도록 확장)
struct Measurement {
    double gls_x;         // 변환된 GLS X (VSLAM 호환)
    double gls_y;         // 변환된 GLS Y (VSLAM 호환)
    double gls_heading;   // 변환된 GLS 헤딩 (VSLAM 호환, 라디안)
    double vslam_x;       // VSLAM 글로벌 X
    double vslam_y;       // VSLAM 글로벌 Y
    double vslam_heading; // VSLAM 글로벌 헤딩 (라디안)
    
    // 잔차 분석을 위한 필드 추가
    double tx_residual; // X축 잔차
    double ty_residual; // Y축 잔차
    double r_residual;  // 회전 잔차 (정규화된 절댓값)
};

// 계산된 외부 보정 파라미터를 담는 구조체
struct ExtrinsicParams {
    double delta_x;     // 평행 이동 X 오프셋 (VSLAM 기준)
    double delta_y;     // 평행 이동 Y 오프셋 (VSLAM 기준)
    double delta_heading; // 회전 오프셋 (VSLAM - GLS, 라디안)
    double scale_factor;  // GLS 스케일 팩터 (추가됨)
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
            // VSLAM Heading Degree -> Radian 변환 적용
            m.vslam_heading = degrees_to_radians(values[2]); 

            // 2. GLS 데이터를 VSLAM 호환 좌표계(RHS, X-Forward)로 변환
            
            // 2-1. 위치 변환 (90도 CW 회전) 및 스케일 보정 적용:
            // GLS Y (Forward) -> VSLAM X (Forward)
            m.gls_x = values[4] * GLS_POSITION_CORRECTION_FACTOR; // (GLS Y) * 1.0 -> X'
            
            // GLS X (Left) -> VSLAM Y (Left)
            // 사용자님의 지적에 따라 불필요한 '-' 부호 제거
            m.gls_y = values[3] * GLS_POSITION_CORRECTION_FACTOR; // (GLS X) * 1.0 -> Y'
            
            // 2-2. 헤딩 변환 (축 정렬 + 회전 규약 변환)
            // a) GLS 원본 헤딩 (Degree)을 Radian으로 변환
            double gls_heading_rad = degrees_to_radians(values[5]); // Degree -> Radian 변환 적용
            
            // b) 축 정렬: 90도 CW 회전 (RHS 기준: -PI/2)
            double gls_axis_aligned_heading = normalize_angle_to_pi_range(gls_heading_rad - PI/2.0); 
            
            // c) 회전 규약 변환 (LHS -> RHS): 각도의 부호를 반전
            // 이 로직은 좌표계 변환과 함께 복잡하게 얽혀있지만,
            // 우선 GLS Y -> VSLAM X, GLS X -> VSLAM Y 변환이 R_90deg_CCW 형태이므로,
            // 헤딩 변환도 R_90deg_CCW를 반영하도록 변경합니다. (이전 코드를 단순화)
            // 이전 복잡한 LHS/RHS 변환을 대신하여, 위치 변환에 대응하는 각도 변환을 사용
            // m.gls_heading = normalize_angle_to_pi_range(gls_heading_rad + PI/2.0); // 90도 CCW 회전
            
            // 다만, GLS와 VSLAM 간의 Heading 차이(delta_heading)는 GLS 헤딩 자체의 오프셋을 의미하므로,
            // GLS 헤딩 변환은 이전처럼 그대로 유지하여 Rotation R을 찾는 것이 더 안정적일 수 있습니다.
            // (GLS의 원래 헤딩 컨벤션을 90도 돌리고 부호를 뒤집는 방식)
            m.gls_heading = normalize_angle_to_pi_range(-gls_axis_aligned_heading); 
            // =======================================================================

            data.push_back(m);
        }
        // 데이터 출력 (잔차 필드는 0으로 초기화됨)
        std::cout << m.vslam_x << " " << m.vslam_y << " " << m.vslam_heading << " " << m.gls_x <<" "<< m.gls_y <<" " << m.gls_heading << std::endl;
    }
    std::cout << "성공: 총 " << data.size() << "개의 측정 데이터를 불러왔습니다." << std::endl;
    return data;
}

// === 1단계: 외부 보정 파라미터 계산 (Similarity Transformation: Scale S 계산 활성화) ===
ExtrinsicParams calculate_extrinsic_params(const std::vector<Measurement>& data) {
    // ExtrinsicParams 초기화
    ExtrinsicParams params = {0.0, 0.0, 0.0, 1.0}; 
    if (data.empty()) return params;

    // 1. 헤딩 오프셋 (Delta Heading) 계산 (이전과 동일하게 평균 각도 차이 사용)
    double sum_sin = 0.0;
    double sum_cos = 0.0;

    for (const auto& m : data) {
        double diff = m.vslam_heading - m.gls_heading;
        diff = normalize_angle_to_pi_range(diff);
        
        sum_sin += std::sin(diff);
        sum_cos += std::cos(diff);
    }

    params.delta_heading = std::atan2(sum_sin / data.size(), sum_cos / data.size());

    // 2. 데이터셋의 무게 중심(Centroid) 계산
    double gls_centroid_x = 0.0, gls_centroid_y = 0.0;
    double vslam_centroid_x = 0.0, vslam_centroid_y = 0.0;
    double count = static_cast<double>(data.size());

    for (const auto& m : data) {
        gls_centroid_x += m.gls_x;
        gls_centroid_y += m.gls_y;
        vslam_centroid_x += m.vslam_x;
        vslam_centroid_y += m.vslam_y;
    }

    gls_centroid_x /= count;
    gls_centroid_y /= count;
    vslam_centroid_x /= count;
    vslam_centroid_y /= count;

    // 3. 스케일 팩터 (S) 계산 (Similarity Transform)
    double cos_dh = std::cos(params.delta_heading);
    double sin_dh = std::sin(params.delta_heading);

    double numerator_s = 0.0;
    double denominator_s = 0.0;
    
    // S = Sum( V_hat dot G_hat_rot ) / Sum( ||G_hat||^2 )
    for (const auto& m : data) {
        // Centered GLS (G_hat)
        double gls_hat_x = m.gls_x - gls_centroid_x;
        double gls_hat_y = m.gls_y - gls_centroid_y;
        
        // Centered VSLAM (V_hat)
        double vslam_hat_x = m.vslam_x - vslam_centroid_x;
        double vslam_hat_y = m.vslam_y - vslam_centroid_y;

        // Rotated Centered GLS (G_hat_rot) - Rotated by Delta Heading
        double gls_hat_rotated_x = gls_hat_x * cos_dh - gls_hat_y * sin_dh;
        double gls_hat_rotated_y = gls_hat_x * sin_dh + gls_hat_y * cos_dh;

        // 분자: Sum of dot products (V_hat dot G_hat_rot)
        numerator_s += vslam_hat_x * gls_hat_rotated_x + vslam_hat_y * gls_hat_rotated_y;
        
        // 분모: Sum of squared norms of Centered GLS (||G_hat||^2)
        // Note: Rotation does not change the norm: ||G_hat_rot||^2 = ||G_hat||^2
        denominator_s += gls_hat_x * gls_hat_x + gls_hat_y * gls_hat_y; 
    }
    
    // Scale Factor calculation
    if (denominator_s > 1e-6) {
        params.scale_factor = numerator_s / denominator_s;
    } else {
        // 데이터가 한 점에 집중되어 있을 경우, S=1.0으로 안전하게 설정
        params.scale_factor = 1.0; 
    }

    // 4. 평행 이동 오프셋 (Delta X, Delta Y) 계산
    // T = V_centroid - S * R * G_centroid
    
    // GLS 무게 중심 좌표에 (Scale * Rotation) 적용
    double gls_scaled_rotated_x = params.scale_factor * (gls_centroid_x * cos_dh - gls_centroid_y * sin_dh);
    double gls_scaled_rotated_y = params.scale_factor * (gls_centroid_x * sin_dh + gls_centroid_y * cos_dh);

    // VSLAM 무게 중심과의 차이
    params.delta_x = vslam_centroid_x - gls_scaled_rotated_x;
    params.delta_y = vslam_centroid_y - gls_scaled_rotated_y;

    return params;
}

// === 2단계: 보정 파라미터 적용 및 잔차 평가 (Scale Factor 포함) ===
EvaluationResult evaluate_calibration(std::vector<Measurement>& data, const ExtrinsicParams& params) {
    EvaluationResult result = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, data.size()};

    double sq_sum_tx_res = 0.0;
    double sq_sum_ty_res = 0.0;
    double sq_sum_r_res = 0.0;
    
    double sum_tx_res = 0.0;
    double sum_ty_res = 0.0;

    double cos_dh = std::cos(params.delta_heading);
    double sin_dh = std::sin(params.delta_heading);
    double scale = params.scale_factor; // 계산된 스케일 팩터 사용

    for (auto& m : data) { 
        // GLS 포즈에 외부 보정 파라미터 적용하여 VSLAM 추정 포즈 계산
        
        // 1. GLS 좌표에 스케일 * 회전 변환 적용 
        // P_calib = R * (S * P_gls) + T
        
        // Step 1.1: GLS 위치에 스케일 적용 (S * P_gls)
        double gls_scaled_x = m.gls_x * scale;
        double gls_scaled_y = m.gls_y * scale;

        // Step 1.2: 회전 변환 적용 (R * (S * P_gls))
        double gls_calib_x = gls_scaled_x * cos_dh - gls_scaled_y * sin_dh;
        double gls_calib_y = gls_scaled_x * sin_dh + gls_scaled_y * cos_dh;

        // Step 2: 평행 이동 변환 적용 (+ T)
        gls_calib_x += params.delta_x;
        gls_calib_y += params.delta_y;

        // 잔차 계산 (VSLAM 실제 측정값 - 보정된 GLS 측정값)
        
        // 위치 잔차 (X, Y)
        double tx_residual = m.vslam_x - gls_calib_x;
        double ty_residual = m.vslam_y - gls_calib_y;
        
        // 회전 잔차 (Heading)
        double r_diff = m.vslam_heading - (m.gls_heading + params.delta_heading);
        r_diff = normalize_angle_to_pi_range(r_diff);
        
        // 잔차 결과를 Measurement 구조체에 저장
        m.tx_residual = tx_residual;
        m.ty_residual = ty_residual;
        m.r_residual = r_diff; // 정규화된 각도 잔차
        
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

// === 3단계: 최종 Inlier 잔차 CSV로 출력 ===
void save_inlier_residuals(const std::vector<Measurement>& data, const std::string& filename) {
    std::ofstream file(filename);
    file << std::fixed << std::setprecision(6);

    // CSV 헤더 (단위 포함)
    file << "GLS_X(m),GLS_Y(m),VSLAM_X(m),VSLAM_Y(m),Residual_X(m),Residual_Y(m),Residual_H(rad),Residual_H(deg)\n";

    for (const auto& m : data) {
        file << m.gls_x << ","
             << m.gls_y << ","
             << m.vslam_x << ","
             << m.vslam_y << ","
             << m.tx_residual << ","
             << m.ty_residual << ","
             << m.r_residual << ","
             << m.r_residual * 180.0 / PI << "\n";
    }

    std::cout << "성공: 최종 Inlier 데이터의 잔차를 '" << filename << "' 파일에 저장했습니다." << std::endl;
    file.close();
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
    
    // =========================================================================
    // 2. 외부 보정 파라미터 계산 (반복 보정 시작)
    // =========================================================================
    const int MAX_ITERATIONS = 3;
    const double OUTLIER_THRESHOLD_MULTIPLIER = 2.0; // 2-sigma (2.0 * RMSE) 기준
    
    std::vector<Measurement> current_data = data;
    ExtrinsicParams params = {0.0, 0.0, 0.0, 1.0}; // 초기화
    EvaluationResult final_eval_result;

    std::cout << "\n================================================" << std::endl;
    std::cout << "3. 반복적 이상치 제거 및 정제 (Iterative Refinement)" << std::endl;
    std::cout << "(2.0 x P_RMSE 및 R_RMSE를 초과하는 데이터 포인트를 제거)" << std::endl;
    std::cout << "================================================" << std::endl;

    for (int i = 0; i < MAX_ITERATIONS; ++i) {
        if (current_data.size() < 2) {
            std::cerr << "경고: 데이터 포인트 부족으로 반복 중단." << std::endl;
            break;
        }
        
        // 1. 현재 데이터셋으로 파라미터 계산 (Scale Factor S 추정)
        params = calculate_extrinsic_params(current_data);

        // 2. 현재 파라미터로 잔차 평가 및 RMSE 계산 (잔차 데이터가 current_data에 저장됨)
        EvaluationResult eval_result = evaluate_calibration(current_data, params); 
        final_eval_result = eval_result;
        
        // 3. 필터링 기준 설정 (RMSE 기반)
        // 2D 위치 잔차의 결합된 RMSE (P_RMSE) 계산
        double p_rmse = std::sqrt(eval_result.rmse_tx * eval_result.rmse_tx + eval_result.rmse_ty * eval_result.rmse_ty);
        
        double p_threshold = OUTLIER_THRESHOLD_MULTIPLIER * p_rmse; // Combined position threshold
        double r_threshold = OUTLIER_THRESHOLD_MULTIPLIER * eval_result.rmse_r;

        std::vector<Measurement> next_data;
        size_t rejected_count = 0;
        
        // 4. 이상치 제거 (Outlier Rejection)
        for (const auto& m : current_data) {
            // 위치 잔차 (2D Euclidean Distance)
            double p_residual = std::sqrt(m.tx_residual * m.tx_residual + m.ty_residual * m.ty_residual);
            double r_residual = std::abs(m.r_residual);

            // 필터링: P-Residual과 R-Residual 모두 기준을 충족해야 함
            if (p_residual <= p_threshold && r_residual <= r_threshold) {
                next_data.push_back(m);
            } else {
                rejected_count++;
            }
        }
        
        // 출력 업데이트: Y-RMSE 포함
        std::cout << "반복 " << i + 1 << ": 잔차 (RMSE_X: " << std::fixed << std::setprecision(4) << eval_result.rmse_tx * 1000.0 << " mm"
                  << ", RMSE_Y: " << eval_result.rmse_ty * 1000.0 << " mm" 
                  << ", RMSE_R: " << eval_result.rmse_r * 180.0 / PI << " deg) - "
                  << rejected_count << "개 이상치 제거됨 (" << next_data.size() << "개 남음)." << std::endl;

        if (next_data.size() == current_data.size()) {
            // 더 이상 이상치가 제거되지 않으면 반복 중단
            std::cout << "이상치 제거 없음. 정제 반복 종료." << std::endl;
            break; 
        }
        current_data = next_data;
    }
    std::cout << std::fixed << std::setprecision(5);
    // =========================================================================
    // 최종 결과 출력
    // =========================================================================

    // 4. 최종 보정 결과 출력
    std::cout << "\n================================================" << std::endl;
    std::cout << "4. 최종 계산된 외부 보정 파라미터 (Final Extrinsic Parameters)" << std::endl;
    std::cout << "================================================" << std::endl;
    std::cout << "회전 오프셋 (Delta Heading, 라디안): " << params.delta_heading << std::endl;
    std::cout << "회전 오프셋 (Delta Heading, 도):     " << params.delta_heading * 180.0 / PI << " deg" << std::endl;
    std::cout << "평행 이동 오프셋 X (Delta X):        " << params.delta_x << " m" << std::endl;
    std::cout << "평행 이동 오프셋 Y (Delta Y):        " << params.delta_y << " m" << std::endl;
    std::cout << "스케일 팩터 (Scale Factor, S):       " << params.scale_factor << " (계산된 값)" << std::endl; // Scale Factor (계산된 값)으로 표시

    // 5. 최종 잔차 평가
    std::cout << "\n================================================" << std::endl;
    std::cout << "5. 최종 잔차 및 일관성 평가 결과 (Final Residuals)" << std::endl;
    std::cout << "(최종 정제된 Inlier 데이터만 사용)" << std::endl;
    std::cout << "================================================" << std::endl;
    
    // RMSE (일관성 평가 지표)
    std::cout << "[일관성 지표: RMSE (Root Mean Square Error)]" << std::endl;
    double final_p_rmse = std::sqrt(final_eval_result.rmse_tx * final_eval_result.rmse_tx + final_eval_result.rmse_ty * final_eval_result.rmse_ty);
    std::cout << "X 축 RMSE: " << final_eval_result.rmse_tx << " m (" << final_eval_result.rmse_tx * 1000.0 << " mm)" << std::endl;
    std::cout << "Y 축 RMSE: " << final_eval_result.rmse_ty << " m (" << final_eval_result.rmse_ty * 1000.0 << " mm)" << std::endl;
    std::cout << "결합 위치 RMSE (P_RMSE): " << final_p_rmse << " m (" << final_p_rmse * 1000.0 << " mm)" << std::endl;
    std::cout << "헤딩 RMSE: " << final_eval_result.rmse_r << " rad (" << final_eval_result.rmse_r * 180.0 / PI << " deg)" << std::endl;
    std::cout << "\n" << std::endl;

    // 잔차 평균 (Mean Residual) - 보정의 성공 여부 확인용
    std::cout << "[보정 성공 지표: 평균 잔차 (Mean Residual)]" << std::endl;
    std::cout << "X 축 평균 잔차 (Mean X Residual): " << final_eval_result.mean_tx_residual << " m (0에 가까워야 함)" << std::endl;
    std::cout << "Y 축 평균 잔차 (Mean Y Residual): " << final_eval_result.mean_ty_residual << " m (0에 가까워야 함)" << std::endl;
    
    std::cout << "\n------------------------------------------------" << std::endl;
    std::cout << "평가 요약: " << data.size() << "개 중 " << final_eval_result.count << "개의 Inlier 데이터로 최종 보정됨." << std::endl;
    std::cout << "------------------------------------------------" << std::endl;

    // 6. 최종 Inlier 잔차 출력 추가
    save_inlier_residuals(current_data, "inlier_residuals.csv");
    
    return 0;
}
