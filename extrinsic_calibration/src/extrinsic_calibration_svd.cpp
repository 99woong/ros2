// coordinate_aware_calibration.cpp
// GLS100과 VSLAM 간의 레버암 계산 (좌표계 차이 고려)
//
// GLS 좌표계: y축=전방, x축=우측, 헤딩=180°(전방), 범위[-180°,180°]
// VSLAM 좌표계: x축=전방, y축=좌측, 헤딩=0°(전방), 범위[0°,360°]
//
// 컴파일:
// g++ -std=c++17 -I/usr/include/eigen3 coordinate_aware_calibration.cpp -O2 -o coord_calib
//
// 사용법:
// ./coord_calib input.csv output.csv

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <algorithm>
#include <numeric>
#include <Eigen/Dense>
#include <Eigen/SVD>

using namespace Eigen;

struct Measurement {
    // 원본 데이터 (입력 그대로)
    double gls_x_raw, gls_y_raw, gls_theta_raw;
    double vslam_x_raw, vslam_y_raw, vslam_theta_raw;
    
    // 통일된 좌표계로 변환된 데이터
    double gls_x, gls_y, gls_theta_rad;
    double vslam_x, vslam_y, vslam_theta_rad;
};

double deg2rad(double deg) { return deg * M_PI / 180.0; }
double rad2deg(double rad) { return rad * 180.0 / M_PI; }

double normalizeAngle(double angle_rad) {
    while (angle_rad > M_PI) angle_rad -= 2 * M_PI;
    while (angle_rad < -M_PI) angle_rad += 2 * M_PI;
    return angle_rad;
}

// GLS 좌표를 표준 좌표계로 변환 (x=전방, y=좌측)
void convertGLStoStandard(double gls_x_in, double gls_y_in, double gls_theta_deg,
                          double& x_out, double& y_out, double& theta_rad_out) {
    // GLS: x=우측, y=전방 → 표준: x=전방, y=좌측
    x_out = -gls_y_in;      // GLS y축(전방) → 표준 x축(전방)
    y_out = gls_x_in;     // GLS x축(우측) → 표준 -y축 (좌측)
    
    // GLS 헤딩: 180°=전방 → 표준: 0°=전방
    // GLS 180° → 표준 0°
    // GLS 90° (좌측) → 표준 90°
    // GLS -90° (우측) → 표준 -90° (or 270°)
    // theta_rad_out = normalizeAngle(deg2rad(gls_theta_deg - 180.0));

    // (2) 헤딩 변환: GLS 180° = 표준 0° 으로 맞추기
    // 즉, θ_std = (θ_gls – 180° + 90°)
    theta_rad_out = normalizeAngle(deg2rad(gls_theta_deg + 90.0));
}

// VSLAM 좌표를 표준 좌표계로 변환 (x=전방, y=좌측)
void convertVSLAMtoStandard(double vslam_x_in, double vslam_y_in, double vslam_theta_deg,
                            double& x_out, double& y_out, double& theta_rad_out) {
    // VSLAM: x=전방, y=좌측 → 표준: x=전방, y=좌측 (이미 동일!)
    x_out = vslam_x_in;
    y_out = vslam_y_in;
    
    // VSLAM 헤딩: 0°=전방 → 표준: 0°=전방 (이미 동일!)
    // 단, 범위를 [0°,360°]에서 [-180°,180°]로 변환
    double theta_deg = vslam_theta_deg;
    if (theta_deg > 180.0) theta_deg -= 360.0;
    theta_rad_out = deg2rad(theta_deg);
}

bool loadCSV(const std::string& filepath, std::vector<Measurement>& data) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open " << filepath << "\n";
        return false;
    }
    
    std::string line;
    std::getline(file, line); // skip header
    
    int count = 0;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        
        std::stringstream ss(line);
        std::string token;
        std::vector<double> values;
        
        while (std::getline(ss, token, ',')) {
            try {
                // 공백 제거
                token.erase(0, token.find_first_not_of(" \t\r\n"));
                token.erase(token.find_last_not_of(" \t\r\n") + 1);
                if (!token.empty()) {
                    values.push_back(std::stod(token));
                }
            } catch (...) { break; }
        }
        
        if (values.size() >= 6) {
            Measurement m;
            m.gls_x_raw = values[1];
            m.gls_y_raw = values[2];
            m.gls_theta_raw = values[3];
            m.vslam_x_raw = values[7];
            m.vslam_y_raw = values[8];
            m.vslam_theta_raw = values[9];
            
            // 좌표계 변환
            convertGLStoStandard(m.gls_x_raw, m.gls_y_raw, m.gls_theta_raw,
                                m.gls_x, m.gls_y, m.gls_theta_rad);
            convertVSLAMtoStandard(m.vslam_x_raw, m.vslam_y_raw, m.vslam_theta_raw,
                                  m.vslam_x, m.vslam_y, m.vslam_theta_rad);
            
            data.push_back(m);
            
            if (++count <= 5) 
            {
                std::cout << "  " << count << ": GLS_raw(" << m.gls_x_raw << ", " 
                          << m.gls_y_raw << ", " << m.gls_theta_raw << "°)"
                          << " → std(" << m.gls_x << ", " << m.gls_y << ", " 
                          << rad2deg(m.gls_theta_rad) << "°)\n";
                std::cout << "       VSLAM_raw(" << m.vslam_x_raw << ", " 
                          << m.vslam_y_raw << ", " << m.vslam_theta_raw << "°)"
                          << " → std(" << m.vslam_x << ", " << m.vslam_y << ", " 
                          << rad2deg(m.vslam_theta_rad) << "°)\n";
            }
        }
    }
    
    file.close();
    return !data.empty();
}

struct CalibResult {
    double tx, ty, theta_rad;
    double rmse_pos, rmse_heading;
    std::vector<double> errors_x, errors_y, errors_theta, errors_dist;
};

CalibResult calibrateWithProcrustes(const std::vector<Measurement>& data) {
    size_t N = data.size();
    CalibResult result;
    
    std::cout << "\n=== Procrustes Analysis (Standard Coordinates) ===\n";
    std::cout << "Samples: " << N << "\n";
    
    // 1. 변환된 좌표로 행렬 구성
    MatrixXd GLS(N, 2), VSLAM(N, 2);
    VectorXd theta_gls(N), theta_vslam(N);
    
    for (size_t i = 0; i < N; ++i) {
        GLS(i, 0) = data[i].gls_x;
        GLS(i, 1) = data[i].gls_y;
        VSLAM(i, 0) = data[i].vslam_x;
        VSLAM(i, 1) = data[i].vslam_y;
        theta_gls(i) = data[i].gls_theta_rad;
        theta_vslam(i) = data[i].vslam_theta_rad;
    }
    
    // 2. 중심점
    Vector2d centroid_gls = GLS.colwise().mean();
    Vector2d centroid_vslam = VSLAM.colwise().mean();
    
    std::cout << "GLS centroid: (" << centroid_gls(0) << ", " << centroid_gls(1) << ") m\n";
    std::cout << "VSLAM centroid: (" << centroid_vslam(0) << ", " << centroid_vslam(1) << ") m\n";
    
    // 3. 중심화
    MatrixXd GLS_c = GLS.rowwise() - centroid_gls.transpose();
    MatrixXd VSLAM_c = VSLAM.rowwise() - centroid_vslam.transpose();
    
    // 4. 공분산 행렬
    Matrix2d H = GLS_c.transpose() * VSLAM_c;
    
    std::cout << "\nCovariance H:\n" << H << "\n";
    
    // 5. SVD로 최적 회전
    JacobiSVD<Matrix2d> svd(H, ComputeFullU | ComputeFullV);
    Matrix2d U = svd.matrixU();
    Matrix2d V = svd.matrixV();
    Matrix2d R = V * U.transpose();
    
    if (R.determinant() < 0) {
        std::cout << "Warning: Reflection detected, correcting...\n";
        V.col(1) *= -1;
        R = V * U.transpose();
    }
    
    double theta_from_pos = std::atan2(R(1, 0), R(0, 0));
    
    std::cout << "Rotation from position alignment: " << rad2deg(theta_from_pos) << "°\n";
    
    // 6. 평행이동
    Vector2d t = centroid_vslam - R * centroid_gls;
    
    std::cout << "Translation: tx = " << t(0) << " m, ty = " << t(1) << " m\n";
    
    // 7. 헤딩 오프셋 (원형 평균)
    double sum_sin = 0.0, sum_cos = 0.0;
    for (size_t i = 0; i < N; ++i) {
        double diff = normalizeAngle(theta_vslam(i) - theta_gls(i));
        sum_sin += std::sin(diff);
        sum_cos += std::cos(diff);
    }
    double heading_offset = std::atan2(sum_sin, sum_cos);
    
    std::cout << "Heading offset (circular mean): " << rad2deg(heading_offset) << "°\n";
    
    // 최종 회전 각도: 헤딩 오프셋 사용 (더 robust)
    double theta_final = heading_offset;
    
    // 회전 재계산
    double c = std::cos(theta_final);
    double s = std::sin(theta_final);
    Matrix2d R_final;
    R_final << c, -s, s, c;
    
    // 평행이동 재계산
    t = centroid_vslam - R_final * centroid_gls;
    
    std::cout << "\nFinal calibration:\n";
    std::cout << "  Rotation: " << rad2deg(theta_final) << "°\n";
    std::cout << "  Translation: (" << t(0) << ", " << t(1) << ") m\n";
    
    // 8. 검증
    std::cout << "\n=== Validation ===\n";
    
    for (size_t i = 0; i < N; ++i) {
        Vector2d gls_pos(data[i].gls_x, data[i].gls_y);
        Vector2d vslam_pred = R_final * gls_pos + t;
        
        double err_x = data[i].vslam_x - vslam_pred(0);
        double err_y = data[i].vslam_y - vslam_pred(1);
        double err_dist = std::sqrt(err_x * err_x + err_y * err_y);
        
        double theta_pred = data[i].gls_theta_rad + theta_final;
        double err_theta = normalizeAngle(data[i].vslam_theta_rad - theta_pred);
        
        result.errors_x.push_back(err_x);
        result.errors_y.push_back(err_y);
        result.errors_dist.push_back(err_dist);
        result.errors_theta.push_back(rad2deg(err_theta));
        
        if (i < 5) {
            std::cout << "  " << (i+1) << ": pos_err=(" << (err_x*100) << ", " 
                      << (err_y*100) << ")cm, heading_err=" << rad2deg(err_theta) 
                      << "°, dist=" << (err_dist*100) << "cm\n";
        }
    }
    
    // 9. 통계
    double sum_sq_pos = 0.0, sum_sq_heading = 0.0;
    for (size_t i = 0; i < N; ++i) {
        sum_sq_pos += result.errors_dist[i] * result.errors_dist[i];
        sum_sq_heading += (result.errors_theta[i] * M_PI / 180.0) * 
                          (result.errors_theta[i] * M_PI / 180.0);
    }
    
    result.tx = t(0);
    result.ty = t(1);
    result.theta_rad = theta_final;
    result.rmse_pos = std::sqrt(sum_sq_pos / N);
    result.rmse_heading = std::sqrt(sum_sq_heading / N) * 180.0 / M_PI;
    
    return result;
}

void computeStats(const std::vector<double>& data,
                  double& mean, double& std_dev, double& rmse) {
    size_t N = data.size();
    if (N == 0) { mean = std_dev = rmse = 0.0; return; }
    
    mean = std::accumulate(data.begin(), data.end(), 0.0) / N;
    
    double sum_sq_dev = 0.0, sum_sq = 0.0;
    for (double v : data) {
        sum_sq_dev += (v - mean) * (v - mean);
        sum_sq += v * v;
    }
    
    std_dev = (N > 1) ? std::sqrt(sum_sq_dev / (N - 1)) : 0.0;
    rmse = std::sqrt(sum_sq / N);
}

void saveResults(const std::string& filepath,
                const std::vector<Measurement>& data,
                const CalibResult& result) {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot create " << filepath << "\n";
        return;
    }
    
    double mean_x, std_x, rmse_x;
    double mean_y, std_y, rmse_y;
    double mean_theta, std_theta, rmse_theta;
    
    computeStats(result.errors_x, mean_x, std_x, rmse_x);
    computeStats(result.errors_y, mean_y, std_y, rmse_y);
    computeStats(result.errors_theta, mean_theta, std_theta, rmse_theta);
    
    file << "# Lever Arm Calibration (Coordinate-Aware)\n";
    file << "# GLS: x=right, y=forward, heading: 180°=forward, [-180°,180°]\n";
    file << "# VSLAM: x=forward, y=left, heading: 0°=forward, [0°,360°]\n";
    file << "#\n";
    file << "# Calibration (in standard frame: x=forward, y=left):\n";
    file << "# tx = " << result.tx << " m\n";
    file << "# ty = " << result.ty << " m\n";
    file << "# theta = " << rad2deg(result.theta_rad) << " deg\n";
    file << "#\n";
    file << "# Statistics:\n";
    file << "# Position RMSE = " << (result.rmse_pos * 100) << " cm\n";
    file << "# Heading RMSE = " << result.rmse_heading << " deg\n";
    file << "# X: mean=" << (mean_x*100) << "cm, std=" << (std_x*100) << "cm\n";
    file << "# Y: mean=" << (mean_y*100) << "cm, std=" << (std_y*100) << "cm\n";
    file << "# Heading: mean=" << mean_theta << "°, std=" << std_theta << "°\n";
    file << "#\n";
    
    file << "sample,gls_x,gls_y,gls_theta,vslam_x,vslam_y,vslam_theta,"
         << "error_x_cm,error_y_cm,error_theta_deg,error_dist_cm\n";
    
    for (size_t i = 0; i < data.size(); ++i) {
        file << (i+1) << ","
             << data[i].gls_x_raw << "," << data[i].gls_y_raw << "," 
             << data[i].gls_theta_raw << ","
             << data[i].vslam_x_raw << "," << data[i].vslam_y_raw << "," 
             << data[i].vslam_theta_raw << ","
             << (result.errors_x[i] * 100) << "," << (result.errors_y[i] * 100) << ","
             << result.errors_theta[i] << "," << (result.errors_dist[i] * 100) << "\n";
    }
    
    file.close();
}

int main(int argc, char** argv) {
    std::cout << "=== Coordinate-Aware Lever Arm Calibration ===\n";
    std::cout << "GLS: x=right, y=forward, heading=180°(fwd), [-180°,180°]\n";
    std::cout << "VSLAM: x=forward, y=left, heading=0°(fwd), [0°,360°]\n\n";
    
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " input.csv [output.csv]\n";
        return 1;
    }
    
    std::string input_file = argv[1];
    std::string output_file = (argc >= 3) ? argv[2] : "calibration_results.csv";
    
    // 데이터 로드 및 좌표 변환
    std::vector<Measurement> data;
    std::cout << "Loading and converting coordinates:\n";
    if (!loadCSV(input_file, data)) {
        return 2;
    }
    
    std::cout << "\nTotal: " << data.size() << " measurements\n";
    
    if (data.size() < 3) {
        std::cerr << "Error: Need at least 3 measurements\n";
        return 3;
    }
    
    // Calibration
    CalibResult result = calibrateWithProcrustes(data);
    
    // 결과 출력
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════╗\n";
    std::cout << "║   LEVER ARM (Standard Frame)           ║\n";
    std::cout << "╠════════════════════════════════════════╣\n";
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "║  Translation:                          ║\n";
    std::cout << "║    tx(fwd) = " << std::setw(7) << result.tx << " m (" 
              << std::setw(5) << std::setprecision(1) << (result.tx*100) << "cm)  ║\n";
    std::cout << "║    ty(left)= " << std::setw(7) << std::setprecision(4) << result.ty 
              << " m (" << std::setw(5) << std::setprecision(1) << (result.ty*100) << "cm)  ║\n";
    std::cout << std::setprecision(2);
    std::cout << "║  Rotation: " << std::setw(8) << rad2deg(result.theta_rad) << "°           ║\n";
    std::cout << "╠════════════════════════════════════════╣\n";
    std::cout << "║  Validation:                           ║\n";
    std::cout << "║    Position RMSE = " << std::setw(6) << (result.rmse_pos*100) 
              << " cm           ║\n";
    std::cout << "║    Heading RMSE  = " << std::setw(6) << result.rmse_heading 
              << "°            ║\n";
    std::cout << "╚════════════════════════════════════════╝\n";
    
    // 원본 좌표계로 레버암 해석
    std::cout << "\n=== Lever Arm in Original Frames ===\n";
    std::cout << "In GLS frame (x=right, y=forward):\n";
    // 표준 (x=fwd, y=left) → GLS (x=right, y=fwd)
    // 표준 tx(fwd) → GLS ty, 표준 ty(left) → GLS -tx
    double gls_lever_x = -result.ty;  // 표준 left → GLS right
    double gls_lever_y = result.tx;   // 표준 fwd → GLS fwd
    std::cout << "  VSLAM is at: x=" << gls_lever_x << "m (right), y=" 
              << gls_lever_y << "m (forward) from GLS\n";
    std::cout << "  = " << (gls_lever_x*100) << "cm right, " 
              << (gls_lever_y*100) << "cm forward\n";
    
    // 결과 저장
    saveResults(output_file, data, result);
    std::cout << "\nResults saved to: " << output_file << "\n";
    
    // 품질 평가
    std::cout << "\n=== Quality Assessment ===\n";
    if (result.rmse_pos * 100 < 2.0) {
        std::cout << "✓ Excellent! (< 2cm)\n";
    } else if (result.rmse_pos * 100 < 5.0) {
        std::cout << "✓ Good (< 5cm)\n";
    } else if (result.rmse_pos * 100 < 10.0) {
        std::cout << "○ Acceptable (< 10cm)\n";
    } else {
        std::cout << "✗ Poor (≥ 10cm) - Check measurements\n";
    }
    
    // 기대값과 비교
    std::cout << "\nExpected: VSLAM at +30cm forward, +6.5cm right from GLS\n";
    std::cout << "Measured: VSLAM at " << (gls_lever_y*100) << "cm forward, " 
              << (gls_lever_x*100) << "cm right from GLS\n";
    
    double diff_fwd = std::abs(gls_lever_y * 100 - 30.0);
    double diff_right = std::abs(gls_lever_x * 100 - 6.5);
    
    if (diff_fwd < 5.0 && diff_right < 5.0) {
        std::cout << "✓ Matches expected lever arm!\n";
    } else {
        std::cout << "⚠ Differs from expected by: " << diff_fwd << "cm(fwd), " 
                  << diff_right << "cm(right)\n";
    }
    
    return 0;
}