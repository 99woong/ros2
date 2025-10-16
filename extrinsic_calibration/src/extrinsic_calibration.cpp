#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <algorithm>
#include <Eigen/Dense>

using namespace Eigen;

// 2D Pose 구조체
struct Pose2D {
    double x, y, theta_deg;
    
    // Pose를 3x3 동차 변환 행렬로 변환
    Matrix3d toMatrix() const {
        double theta_rad = theta_deg * M_PI / 180.0;
        double c = std::cos(theta_rad);
        double s = std::sin(theta_rad);
        
        Matrix3d T = Matrix3d::Identity();
        T(0, 0) = c;   T(0, 1) = -s;  T(0, 2) = x;
        T(1, 0) = s;   T(1, 1) = c;   T(1, 2) = y;
        // T(2, 2) = 1 (이미 Identity)
        return T;
    }
    
    // 3x3 행렬을 Pose로 변환
    static Pose2D fromMatrix(const Matrix3d& T) {
        Pose2D p;
        p.x = T(0, 2);
        p.y = T(1, 2);
        p.theta_deg = std::atan2(T(1, 0), T(0, 0)) * 180.0 / M_PI;
        return p;
    }
};

// 측정 샘플
struct Measurement {
    Pose2D gls;    // GLS100 측정값 (마커 기준)
    Pose2D vslam;  // VSLAM 측정값 (맵 기준)
};

// 각도를 [-180, 180] 범위로 정규화
double normalizeAngle(double angle_deg) {
    while (angle_deg > 180.0) angle_deg -= 360.0;
    while (angle_deg < -180.0) angle_deg += 360.0;
    return angle_deg;
}

// 라디안 각도를 [-π, π] 범위로 정규화
double wrapToPi(double angle_rad) {
    double v = std::fmod(angle_rad + M_PI, 2.0 * M_PI);
    if (v < 0) v += 2.0 * M_PI;
    return v - M_PI;
}

// CSV 파일 로드
bool loadCSV(const std::string& filepath, std::vector<Measurement>& measurements) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open file " << filepath << "\n";
        return false;
    }
    
    std::string line;
    // 헤더 스킵
    std::getline(file, line);
    
    int line_num = 1;
    while (std::getline(file, line)) {
        line_num++;
        if (line.empty() || line[0] == '#') continue;
        
        std::stringstream ss(line);
        std::string token;
        std::vector<double> values;
        
        while (std::getline(ss, token, ',')) {
            try {
                values.push_back(std::stod(token));
            } catch (const std::exception& e) {
                std::cerr << "Warning: Invalid value at line " << line_num << "\n";
                break;
            }
        }
        
        if (values.size() >= 6) {
            Measurement m;
            m.gls.x = values[0];
            m.gls.y = values[1];
            m.gls.theta_deg = values[2];
            m.vslam.x = values[3];
            m.vslam.y = values[4];
            m.vslam.theta_deg = values[5];
            measurements.push_back(m);
        }
    }
    
    file.close();
    return !measurements.empty();
}

// Hand-Eye Calibration 수행
// AX = XB 문제를 풀어서 X (센서 간 변환) 계산
Matrix3d calibrateSensors(const std::vector<Measurement>& measurements) {
    size_t N = measurements.size();
    
    if (N < 3) {
        std::cerr << "Error: Need at least 3 measurements. Got " << N << "\n";
        return Matrix3d::Identity();
    }
    
    std::cout << "\n=== Calibration Process ===\n";
    std::cout << "Total measurements: " << N << "\n";
    
    // 1. 기준 포즈 설정 (첫 번째 측정)
    Matrix3d T_gls_base = measurements[0].gls.toMatrix();
    Matrix3d T_vslam_base = measurements[0].vslam.toMatrix();
    
    std::cout << "Base pose (measurement 1):\n";
    std::cout << "  GLS:   (" << measurements[0].gls.x << ", " 
              << measurements[0].gls.y << ", " 
              << measurements[0].gls.theta_deg << "°)\n";
    std::cout << "  VSLAM: (" << measurements[0].vslam.x << ", " 
              << measurements[0].vslam.y << ", " 
              << measurements[0].vslam.theta_deg << "°)\n";
    
    // 2. 모든 측정에 대해 상대 변환 계산
    std::vector<Matrix3d> A_list;  // GLS 상대 변환
    std::vector<Matrix3d> B_list;  // VSLAM 상대 변환
    
    Matrix3d T_gls_base_inv = T_gls_base.inverse();
    Matrix3d T_vslam_base_inv = T_vslam_base.inverse();
    
    std::cout << "\nComputing relative transforms:\n";
    for (size_t i = 1; i < N; ++i) {
        Matrix3d T_gls_i = measurements[i].gls.toMatrix();
        Matrix3d T_vslam_i = measurements[i].vslam.toMatrix();
        
        // A = T_base^-1 × T_i (기준에서 현재로의 상대 변환)
        Matrix3d A = T_gls_base_inv * T_gls_i;
        Matrix3d B = T_vslam_base_inv * T_vslam_i;
        
        A_list.push_back(A);
        B_list.push_back(B);
        
        Pose2D rel_gls = Pose2D::fromMatrix(A);
        Pose2D rel_vslam = Pose2D::fromMatrix(B);
        
        if (i <= 3) {  // 처음 3개만 출력
            std::cout << "  Pair " << i << ": GLS_rel(" 
                      << rel_gls.x << ", " << rel_gls.y << ", " 
                      << rel_gls.theta_deg << "°) -> VSLAM_rel("
                      << rel_vslam.x << ", " << rel_vslam.y << ", " 
                      << rel_vslam.theta_deg << "°)\n";
        }
    }
    
    // 3. 회전 각도 추정 (원형 평균)
    // B = X^-1 × A × X 관계에서 θ_B = θ_A + 2×θ_X (근사)
    // 실제로는 θ_B - θ_A = 센서 간 회전의 2배 (정확한 유도는 복잡)
    // 단순화: θ_X = (θ_B - θ_A) / 2 의 평균
    
    std::cout << "\n=== Step 1: Estimating Rotation ===\n";
    
    std::vector<double> theta_diffs;
    double sum_sin = 0.0, sum_cos = 0.0;
    
    for (size_t i = 0; i < A_list.size(); ++i) {
        double theta_A = std::atan2(A_list[i](1, 0), A_list[i](0, 0));
        double theta_B = std::atan2(B_list[i](1, 0), B_list[i](0, 0));
        double diff = wrapToPi(theta_B - theta_A);
        
        theta_diffs.push_back(diff);
        sum_sin += std::sin(diff);
        sum_cos += std::cos(diff);
    }
    
    // 원형 평균
    double theta_mean = std::atan2(sum_sin, sum_cos);
    
    // 2D에서는 센서 offset 각도가 상대 회전 차이와 관련
    // 단순 근사: 회전 차이의 절반 (경험적)
    double theta_calib = theta_mean / 2.0;
    
    std::cout << "Rotation angle differences (sample):\n";
    for (size_t i = 0; i < std::min(size_t(5), theta_diffs.size()); ++i) {
        std::cout << "  Pair " << (i+1) << ": " << (theta_diffs[i] * 180.0 / M_PI) << "°\n";
    }
    std::cout << "Circular mean: " << (theta_mean * 180.0 / M_PI) << "°\n";
    std::cout << "Estimated sensor rotation: " << (theta_calib * 180.0 / M_PI) << "°\n";
    
    // 4. 평행이동 추정 (최소자승)
    // B.t = R_X × A.t + t_X를 선형 시스템으로 풀기
    
    std::cout << "\n=== Step 2: Estimating Translation ===\n";
    
    double c = std::cos(theta_calib);
    double s = std::sin(theta_calib);
    Matrix2d R_calib;
    R_calib << c, -s,
               s,  c;
    
    // 최소자승을 위한 행렬 구성
    // M × [tx, ty]^T = b
    MatrixXd M(2 * A_list.size(), 2);
    VectorXd b(2 * A_list.size());
    
    for (size_t i = 0; i < A_list.size(); ++i) {
        Vector2d t_A(A_list[i](0, 2), A_list[i](1, 2));
        Vector2d t_B(B_list[i](0, 2), B_list[i](1, 2));
        
        // R_calib × t_A를 계산
        Vector2d R_t_A = R_calib * t_A;
        
        // 선형 방정식: t_B = R_t_A + t_calib
        // => t_calib = t_B - R_t_A
        
        M(2*i, 0) = 1.0;     M(2*i, 1) = 0.0;
        M(2*i+1, 0) = 0.0;   M(2*i+1, 1) = 1.0;
        
        b(2*i) = t_B(0) - R_t_A(0);
        b(2*i+1) = t_B(1) - R_t_A(1);
    }
    
    // 최소자승 해: (M^T M)^-1 M^T b
    Vector2d t_calib = (M.transpose() * M).inverse() * M.transpose() * b;
    
    std::cout << "Estimated translation: tx = " << t_calib(0) 
              << " m, ty = " << t_calib(1) << " m\n";
    
    // 5. 최종 변환 행렬 구성
    Matrix3d X = Matrix3d::Identity();
    X(0, 0) = c;            X(0, 1) = -s;           X(0, 2) = t_calib(0);
    X(1, 0) = s;            X(1, 1) = c;            X(1, 2) = t_calib(1);
    
    return X;
}

// Calibration 결과 검증
void validateCalibration(const std::vector<Measurement>& measurements,
                        const Matrix3d& X_calib,
                        std::vector<double>& errors_x,
                        std::vector<double>& errors_y,
                        std::vector<double>& errors_theta,
                        std::vector<double>& errors_dist) {
    
    std::cout << "\n=== Validation ===\n";
    std::cout << "Transforming GLS measurements to VSLAM frame...\n";
    
    for (size_t i = 0; i < measurements.size(); ++i) {
        // GLS 측정값을 VSLAM 좌표계로 변환
        Matrix3d T_gls = measurements[i].gls.toMatrix();
        Matrix3d T_vslam_predicted = X_calib * T_gls;
        
        // 실제 VSLAM 측정값
        Matrix3d T_vslam_actual = measurements[i].vslam.toMatrix();
        
        // 오차 계산
        double err_x = T_vslam_actual(0, 2) - T_vslam_predicted(0, 2);
        double err_y = T_vslam_actual(1, 2) - T_vslam_predicted(1, 2);
        double err_dist = std::sqrt(err_x * err_x + err_y * err_y);
        
        double theta_pred = std::atan2(T_vslam_predicted(1, 0), T_vslam_predicted(0, 0));
        double theta_actual = std::atan2(T_vslam_actual(1, 0), T_vslam_actual(0, 0));
        double err_theta = wrapToPi(theta_actual - theta_pred) * 180.0 / M_PI;
        
        errors_x.push_back(err_x);
        errors_y.push_back(err_y);
        errors_theta.push_back(err_theta);
        errors_dist.push_back(err_dist);
        
        if (i < 5) {  // 처음 5개만 출력
            std::cout << "  Sample " << (i+1) << ": error = (" 
                      << (err_x * 100) << ", " << (err_y * 100) << ") cm, "
                      << err_theta << "°, dist = " << (err_dist * 100) << " cm\n";
        }
    }
}

// 통계 계산
void computeStats(const std::vector<double>& data, 
                  double& mean, double& std_dev, double& rmse) {
    size_t N = data.size();
    if (N == 0) {
        mean = std_dev = rmse = 0.0;
        return;
    }
    
    // 평균
    mean = 0.0;
    for (double v : data) mean += v;
    mean /= N;
    
    // 표준편차와 RMSE
    double sum_sq_dev = 0.0;
    double sum_sq = 0.0;
    for (double v : data) {
        sum_sq_dev += (v - mean) * (v - mean);
        sum_sq += v * v;
    }
    
    std_dev = (N > 1) ? std::sqrt(sum_sq_dev / (N - 1)) : 0.0;
    rmse = std::sqrt(sum_sq / N);
}

// 결과를 CSV로 저장
void saveResults(const std::string& filepath,
                const std::vector<Measurement>& measurements,
                const Matrix3d& X_calib,
                const std::vector<double>& errors_x,
                const std::vector<double>& errors_y,
                const std::vector<double>& errors_theta,
                const std::vector<double>& errors_dist) {
    
    std::ofstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot create output file " << filepath << "\n";
        return;
    }
    
    Pose2D calib_pose = Pose2D::fromMatrix(X_calib);
    
    // 통계 계산
    double mean_x, std_x, rmse_x;
    double mean_y, std_y, rmse_y;
    double mean_theta, std_theta, rmse_theta;
    double mean_dist, std_dist, rmse_dist;
    
    computeStats(errors_x, mean_x, std_x, rmse_x);
    computeStats(errors_y, mean_y, std_y, rmse_y);
    computeStats(errors_theta, mean_theta, std_theta, rmse_theta);
    computeStats(errors_dist, mean_dist, std_dist, rmse_dist);
    
    // 헤더
    file << "# Sensor Calibration Results\n";
    file << "# GLS100 to VSLAM Camera Transform\n";
    file << "#\n";
    file << "# Calibration Parameters:\n";
    file << "# tx = " << calib_pose.x << " m\n";
    file << "# ty = " << calib_pose.y << " m\n";
    file << "# theta = " << calib_pose.theta_deg << " deg\n";
    file << "#\n";
    file << "# Validation Statistics:\n";
    file << "# X error - Mean: " << (mean_x*100) << " cm, Std: " << (std_x*100) 
         << " cm, RMSE: " << (rmse_x*100) << " cm\n";
    file << "# Y error - Mean: " << (mean_y*100) << " cm, Std: " << (std_y*100) 
         << " cm, RMSE: " << (rmse_y*100) << " cm\n";
    file << "# Theta error - Mean: " << mean_theta << " deg, Std: " << std_theta 
         << " deg, RMSE: " << rmse_theta << " deg\n";
    file << "# 2D Distance - Mean: " << (mean_dist*100) << " cm, Std: " << (std_dist*100) 
         << " cm, RMSE: " << (rmse_dist*100) << " cm\n";
    file << "#\n";
    
    // 데이터 테이블
    file << "sample,gls_x,gls_y,gls_theta,vslam_x,vslam_y,vslam_theta,"
         << "error_x_cm,error_y_cm,error_theta_deg,error_dist_cm\n";
    
    for (size_t i = 0; i < measurements.size(); ++i) {
        file << (i+1) << ","
             << measurements[i].gls.x << "," 
             << measurements[i].gls.y << "," 
             << measurements[i].gls.theta_deg << ","
             << measurements[i].vslam.x << "," 
             << measurements[i].vslam.y << "," 
             << measurements[i].vslam.theta_deg << ","
             << (errors_x[i] * 100) << "," 
             << (errors_y[i] * 100) << "," 
             << errors_theta[i] << ","
             << (errors_dist[i] * 100) << "\n";
    }
    
    file.close();
    std::cout << "\nResults saved to: " << filepath << "\n";
}

int main(int argc, char** argv) {
    std::cout << "=== GLS100-VSLAM Sensor Calibration ===\n";
    std::cout << "Estimate physical distance and angle between sensors\n\n";
    
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " input.csv [output.csv]\n";
        std::cout << "\nInput CSV format:\n";
        std::cout << "  x_gls,y_gls,theta_gls,x_vslam,y_vslam,theta_vslam\n";
        std::cout << "\nExample:\n";
        std::cout << "  1.05,1.05,1.0,2.05,2.20,5.0\n";
        std::cout << "  1.15,1.15,2.0,2.15,2.30,6.0\n";
        std::cout << "  ...\n";
        return 1;
    }
    
    std::string input_file = argv[1];
    std::string output_file = (argc >= 3) ? argv[2] : "calibration_results.csv";
    
    // 1. 데이터 로드
    std::vector<Measurement> measurements;
    if (!loadCSV(input_file, measurements)) {
        return 2;
    }
    
    std::cout << "Loaded " << measurements.size() << " measurements from " << input_file << "\n";
    
    if (measurements.size() < 3) {
        std::cerr << "Error: Need at least 3 measurements for calibration\n";
        return 3;
    }
    
    // 2. Calibration 수행
    Matrix3d X_calib = calibrateSensors(measurements);
    
    Pose2D calib_result = Pose2D::fromMatrix(X_calib);
    
    // 3. 결과 출력
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════╗\n";
    std::cout << "║   CALIBRATION RESULT                   ║\n";
    std::cout << "╠════════════════════════════════════════╣\n";
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "║  Translation (GLS → VSLAM):            ║\n";
    std::cout << "║    tx = " << std::setw(8) << calib_result.x << " m              ║\n";
    std::cout << "║    ty = " << std::setw(8) << calib_result.y << " m              ║\n";
    std::cout << std::setprecision(2);
    std::cout << "║  Rotation:                             ║\n";
    std::cout << "║    θ  = " << std::setw(8) << calib_result.theta_deg << " deg            ║\n";
    std::cout << "╚════════════════════════════════════════╝\n";
    
    // 4. 검증
    std::vector<double> errors_x, errors_y, errors_theta, errors_dist;
    validateCalibration(measurements, X_calib, errors_x, errors_y, errors_theta, errors_dist);
    
    // 5. 통계 출력
    double mean_x, std_x, rmse_x;
    double mean_y, std_y, rmse_y;
    double mean_theta, std_theta, rmse_theta;
    double mean_dist, std_dist, rmse_dist;
    
    computeStats(errors_x, mean_x, std_x, rmse_x);
    computeStats(errors_y, mean_y, std_y, rmse_y);
    computeStats(errors_theta, mean_theta, std_theta, rmse_theta);
    computeStats(errors_dist, mean_dist, std_dist, rmse_dist);
    
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════╗\n";
    std::cout << "║   VALIDATION STATISTICS                ║\n";
    std::cout << "╠════════════════════════════════════════╣\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "║  Position Error X:                     ║\n";
    std::cout << "║    RMSE = " << std::setw(6) << (rmse_x*100) << " cm                   ║\n";
    std::cout << "║    Std  = " << std::setw(6) << (std_x*100) << " cm                   ║\n";
    std::cout << "║                                        ║\n";
    std::cout << "║  Position Error Y:                     ║\n";
    std::cout << "║    RMSE = " << std::setw(6) << (rmse_y*100) << " cm                   ║\n";
    std::cout << "║    Std  = " << std::setw(6) << (std_y*100) << " cm                   ║\n";
    std::cout << "║                                        ║\n";
    std::cout << "║  Heading Error:                        ║\n";
    std::cout << "║    RMSE = " << std::setw(6) << rmse_theta << " deg                  ║\n";
    std::cout << "║    Std  = " << std::setw(6) << std_theta << " deg                  ║\n";
    std::cout << "║                                        ║\n";
    std::cout << "║  2D Distance Error:                    ║\n";
    std::cout << "║    RMSE = " << std::setw(6) << (rmse_dist*100) << " cm                   ║\n";
    std::cout << "╚════════════════════════════════════════╝\n";
    
    // 6. 결과 저장
    saveResults(output_file, measurements, X_calib, errors_x, errors_y, errors_theta, errors_dist);
    
    // 7. 품질 평가
    std::cout << "\n=== Quality Assessment ===\n";
    if (rmse_dist * 100 < 1.0) {
        std::cout << "✓ Excellent calibration! (< 1 cm RMSE)\n";
    } else if (rmse_dist * 100 < 2.0) {
        std::cout << "✓ Good calibration (< 2 cm RMSE)\n";
    } else if (rmse_dist * 100 < 5.0) {
        std::cout << "○ Acceptable calibration (< 5 cm RMSE)\n";
        std::cout << "  Consider adding more diverse measurements\n";
    } else {
        std::cout << "✗ Poor calibration (≥ 5 cm RMSE)\n";
        std::cout << "  Recommendations:\n";
        std::cout << "  1. Add more measurements with diverse poses\n";
        std::cout << "  2. Ensure measurements are stable (vehicle fully stopped)\n";
        std::cout << "  3. Check sensor synchronization\n";
        std::cout << "  4. Verify GLS marker detection quality\n";
    }
    
    return 0;
}