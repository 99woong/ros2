#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <numeric>

// Eigen 라이브러리가 필요합니다.
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

// 데이터 구조체 정의
struct PoseData {
    double vloc_x, vloc_y, vloc_yaw;
    double gls_x, gls_y, gls_yaw;
    // -90도 보정된 GLS 데이터를 저장할 필드
    double gls_x_corr, gls_y_corr, gls_yaw_corr;
};

// 상수를 라디안으로 변환
constexpr double DEG_TO_RAD = M_PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / M_PI;

// 각도 정규화 (-180 ~ 180도)
double normalizeAngleDeg(double angle) {
    angle = fmod(angle + 180.0, 360.0);
    if (angle < 0) {
        angle += 360.0;
    }
    return angle - 180.0;
}

// CSV 파일 읽기 및 GLS 데이터 -90도 보정 적용
std::vector<PoseData> readAndCorrectCSV(const std::string& filename) {
    std::vector<PoseData> data;
    std::ifstream file(filename);
    std::string line;

    // 헤더 스킵
    if (std::getline(file, line)) { /* skip header */ }

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        PoseData d;
        int col = 0;
        
        // 데이터 파싱: vloc_x,vloc_y,vloc_yaw,gls_x,gls_y,gls_yaw
        while (std::getline(ss, cell, ',')) {
            try {
                if (col == 0) d.vloc_x = std::stod(cell);
                else if (col == 1) d.vloc_y = std::stod(cell);
                else if (col == 2) d.vloc_yaw = std::stod(cell);
                else if (col == 3) d.gls_x = std::stod(cell);
                else if (col == 4) d.gls_y = std::stod(cell);
                else if (col == 5) d.gls_yaw = std::stod(cell);
                
                col++;
                // 필요한 6개의 데이터만 읽음
                if (col >= 6) break; 
            } catch (const std::exception& e) {
                // 파싱 오류 발생 시 해당 라인 건너뛰기
                break;
            }
        }

        if (col >= 6) {
            // --- GLS 데이터 -90도 회전 보정 적용 (GLS 좌표계 X <-> Y 축 변경) ---
            // 로봇의 X축 (전방) = GLS의 Y축 (전방)
            // 로봇의 Y축 (좌측) = GLS의 -X축 (우측)
            // 위치: R(-90) * P = (gls_y, -gls_x)
            d.gls_x_corr = d.gls_y;
            d.gls_y_corr = -d.gls_x;
            
            // 헤딩: yaw_corr = yaw - 90.0 deg
            d.gls_yaw_corr = normalizeAngleDeg(d.gls_yaw - 90.0);

            data.push_back(d);
        }
    }
    return data;
}

// 레버 암 추정 및 잔차 계산 함수 (변화량 기반, -90도 보정 데이터 사용)
void estimateLeverArmAndEvaluate(const std::vector<PoseData>& data) {
    if (data.size() < 2) {
        cerr << "Error: Need at least two data points for increment-based estimation." << endl;
        return;
    }

    const int N = data.size();
    const int M = N - 1; // 변화량 샘플 개수

    // --- 1. 헤딩 오프셋 (Lever-Arm Theta) 추정 ---
    // (보정된 GLS Yaw 변화량과 VSLAM Yaw 변화량의 평균 차이)
    vector<double> yaw_delta_diff_rad;
    for (int i = 0; i < M; ++i) {
        // VSLAM Delta Yaw (CCW)
        double vloc_delta_yaw = normalizeAngleDeg(data[i+1].vloc_yaw - data[i].vloc_yaw) * DEG_TO_RAD;
        
        // 보정된 GLS100 Delta Yaw (CW -> CCW 변환을 통해 VSLAM과 동일한 규약으로 만듦)
        // 주의: GLS의 원본 Yaw는 CW/RHS일 수 있으므로, VSLAM과 동일한 CCW/RHS로 변환하는 과정을 유지함
        double gls_delta_yaw = normalizeAngleDeg((-data[i+1].gls_yaw_corr) - (-data[i].gls_yaw_corr)) * DEG_TO_RAD;

        yaw_delta_diff_rad.push_back(vloc_delta_yaw - gls_delta_yaw);
    }
    
    double lever_arm_yaw_rad = 0.0;
    for (double diff : yaw_delta_diff_rad) {
        lever_arm_yaw_rad += diff;
    }
    lever_arm_yaw_rad /= yaw_delta_diff_rad.size();
    
    // --- 2. 위치 레버 암 L 추정 (변화량 기반 선형 최소 제곱법) ---
    // Ax = b, x = [L_x, L_y]^T (미지수 2개)
    MatrixXd A(2 * M, 2);
    VectorXd b(2 * M);

    for (int i = 0; i < M; ++i) {
        const auto& d1 = data[i];   // t_i
        const auto& d2 = data[i+1]; // t_i+1

        // VSLAM Yaw (라디안)
        double yaw1 = d1.vloc_yaw * DEG_TO_RAD;
        double yaw2 = d2.vloc_yaw * DEG_TO_RAD;
        
        // 회전 행렬 성분
        double cosY1 = cos(yaw1); double sinY1 = sin(yaw1);
        double cosY2 = cos(yaw2); double sinY2 = sin(yaw2);

        // A_i = R(yaw_2) - R(yaw_1)
        double A11 = cosY2 - cosY1; double A12 = -sinY2 + sinY1;
        double A21 = sinY2 - sinY1; double A22 = cosY2 - cosY1;

        // A 행렬 (2x2 블록)
        A(2 * i, 0)     = A11; A(2 * i, 1)     = A12;
        A(2 * i + 1, 0) = A21; A(2 * i + 1, 1) = A22;

        // b 벡터: delta_P_v - delta_P_g_corr
        double delta_v_x = d2.vloc_x - d1.vloc_x;
        double delta_v_y = d2.vloc_y - d1.vloc_y;
        double delta_g_x_corr = d2.gls_x_corr - d1.gls_x_corr;
        double delta_g_y_corr = d2.gls_y_corr - d1.gls_y_corr;
        
        b(2 * i)     = delta_v_x - delta_g_x_corr;
        b(2 * i + 1) = delta_v_y - delta_g_y_corr;
    }

    // 최소 제곱 해: x = [L_x, L_y]^T
    Vector2d x = (A.transpose() * A).ldlt().solve(A.transpose() * b);

    // 결과 추출
    double lever_arm_x = x(0);
    double lever_arm_y = x(1);

    // --- 3. 결과 출력 ---
    cout << "## 📏 Lever-Arm (Offset) Estimation Results (90-deg Corrected LGS)" << endl;
    cout << "------------------------------------------------------------------------" << endl;
    cout << fixed << setprecision(6);
    
    // 헤딩 오프셋
    cout << "**Heading Delta Difference Offset (Theta):**" << endl;
    cout << "   - **Rad:** " << lever_arm_yaw_rad << " [rad]" << endl;
    cout << "   - **Deg:** " << lever_arm_yaw_rad * RAD_TO_DEG << " [deg]" << endl;
    
    // 레버 암 (L)
    cout << "**Physical Lever-Arm (L_x, L_y in Robot Frame):**" << endl;
    cout << "   - **L_x:** " << lever_arm_x << " [m]" << endl;
    cout << "   - **L_y:** " << lever_arm_y << " [m]" << endl;
    cout << "------------------------------------------------------------------------" << endl;


    // --- 4. 잔차 평가 (Residual Evaluation) ---
    // Global Origin Offset t_global 추정 및 최종 잔차 계산

    cout << "## 📈 Residual Evaluation (After L & Theta Alignment)" << endl;
    cout << "-------------------------------------------" << endl;
    
    // t_global 추정 (Lever-Arm 적용 후 평균 차이)
    double sum_tx = 0.0;
    double sum_ty = 0.0;

    for (const auto& d : data) {
        double vloc_yaw_rad = d.vloc_yaw * DEG_TO_RAD;
        
        // R(yaw_v) * L_robot
        double L_global_x = lever_arm_x * cos(vloc_yaw_rad) - lever_arm_y * sin(vloc_yaw_rad);
        double L_global_y = lever_arm_x * sin(vloc_yaw_rad) + lever_arm_y * cos(vloc_yaw_rad);

        // t_global_i = P_v - P_g_corr - R(yaw_v) * L
        sum_tx += d.vloc_x - d.gls_x_corr - L_global_x;
        sum_ty += d.vloc_y - d.gls_y_corr - L_global_y;
    }
    
    // 정렬된 데이터의 평균 글로벌 오프셋 (t_global)
    double global_offset_tx = sum_tx / N;
    double global_offset_ty = sum_ty / N;
    
    cout << "**Global Origin Offset (t_x, t_y) [G vs M, Derived]:**" << endl;
    cout << "   - **t_x:** " << global_offset_tx << " [m]" << endl;
    cout << "   - **t_y:** " << global_offset_ty << " [m]" << endl;


    // 최종 잔차 계산
    double sum_sq_res_x = 0.0; double sum_sq_res_y = 0.0; double sum_sq_res_yaw = 0.0;

    ofstream residual_file("residual_evaluation_90deg_corrected.csv");
    residual_file << fixed << setprecision(6);
    residual_file << "vloc_x,vloc_y,vloc_yaw,gls_x_aligned,gls_y_aligned,gls_yaw_aligned_deg,residual_x,residual_y,residual_yaw_deg" << endl;

    for (const auto& d : data) {
        double vloc_yaw_rad = d.vloc_yaw * DEG_TO_RAD;
        
        // 1. GLS100 위치 정렬: P_g_aligned = P_g_corr + t_global + R(yaw_v) * L_robot
        double L_global_x = lever_arm_x * cos(vloc_yaw_rad) - lever_arm_y * sin(vloc_yaw_rad);
        double L_global_y = lever_arm_x * sin(vloc_yaw_rad) + lever_arm_y * cos(vloc_yaw_rad);
        
        double gls_x_aligned = d.gls_x_corr + global_offset_tx + L_global_x;
        double gls_y_aligned = d.gls_y_corr + global_offset_ty + L_global_y;

        // 2. 헤딩 변환: yaw_g_aligned = -yaw_g_corr_cw + theta
        double gls_yaw_aligned_rad = -d.gls_yaw_corr * DEG_TO_RAD + lever_arm_yaw_rad;
        double gls_yaw_aligned_deg = gls_yaw_aligned_rad * RAD_TO_DEG;

        // 3. 잔차 계산 (VSLAM - Aligned GLS)
        double res_x = d.vloc_x - gls_x_aligned;
        double res_y = d.vloc_y - gls_y_aligned;
        double res_yaw_deg = normalizeAngleDeg(d.vloc_yaw - gls_yaw_aligned_deg);

        // 잔차 저장 및 통계 계산
        residual_file << d.vloc_x << "," << d.vloc_y << "," << d.vloc_yaw << ","
                      << gls_x_aligned << "," << gls_y_aligned << "," << gls_yaw_aligned_deg << ","
                      << res_x << "," << res_y << "," << res_yaw_deg << endl;
        
        sum_sq_res_x += res_x * res_x; sum_sq_res_y += res_y * res_y; sum_sq_res_yaw += res_yaw_deg * res_yaw_deg;
    }
    
    // 최종 통계
    double rmse_x = sqrt(sum_sq_res_x / N);
    double rmse_y = sqrt(sum_sq_res_y / N);
    double rmse_yaw_deg = sqrt(sum_sq_res_yaw / N);

    cout << "**RMSE (Root Mean Square Error) for Consistency:**" << endl;
    cout << "   - **X (m):** " << rmse_x << endl;
    cout << "   - **Y (m):** " << rmse_y << endl;
    cout << "   - **Yaw (deg):** " << rmse_yaw_deg << endl;

    cout << "-------------------------------------------" << endl;
    cout << "Residuals saved to 'residual_evaluation_90deg_corrected.csv'" << endl;
    residual_file.close();
}

int main() {
    // Eigen 라이브러리가 필요합니다.
    // 사용자가 업로드한 파일 이름을 사용합니다.
    std::string filename = "pose_log_20251024_112531_832.csv"; 
    
    // GLS 데이터에 -90도 보정을 적용하면서 CSV를 읽습니다.
    auto data = readAndCorrectCSV(filename);

    if (data.empty()) {
        cerr << "파일에서 데이터를 읽는 데 실패했거나 데이터가 충분하지 않습니다: " << filename << endl;
        return 1;
    }

    estimateLeverArmAndEvaluate(data);

    return 0;
}
