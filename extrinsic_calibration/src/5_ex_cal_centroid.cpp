#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <string>
#include <iomanip>
#include <numeric> // For std::accumulate

// M_PI 상수가 정의되어 있지 않을 경우를 대비하여 정의
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct Pose2D {
    double x;
    double y;
    double yaw_rad;  // radians으로 통일 (필드명 변경)
};

struct PosePair {
    Pose2D vslam;
    Pose2D gls;
};

struct CalibrationParams {
    double translation_x;
    double translation_y;
    double rotation;     // radians
    double yaw_offset;   // radians
};

// Normalize angle to [-pi, pi]
double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// Convert degrees to radians
double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

// Convert radians to degrees
double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}

// VSLAM 포즈를 GLS 좌표계로 변환
Pose2D transformVSLAMtoGLS(const Pose2D& vslam_pose, const CalibrationParams& calib) {
    Pose2D gls_pose;
    
    // 1. Translation 변환 (Rotation 적용 후)
    double cos_r = std::cos(calib.rotation);
    double sin_r = std::sin(calib.rotation);
    
    double x_rotated = cos_r * vslam_pose.x - sin_r * vslam_pose.y;
    double y_rotated = sin_r * vslam_pose.x + cos_r * vslam_pose.y;
    
    gls_pose.x = x_rotated + calib.translation_x;
    gls_pose.y = y_rotated + calib.translation_y;
    
    // 2. Yaw 변환
    // [수정된 핵심 로직]: loadData에서 방향을 통일했으므로 단순 덧셈으로 처리
    gls_pose.yaw_rad = normalizeAngle(vslam_pose.yaw_rad + calib.rotation + calib.yaw_offset);
    
    return gls_pose;
}

// 외부 보정 파라미터 계산
CalibrationParams calibrateExternalParameters(const std::vector<PosePair>& pairs) {
    if (pairs.empty()) {
        std::cerr << "Error: No pose pairs provided" << std::endl;
        return CalibrationParams{0, 0, 0, 0};
    }
    
    CalibrationParams calib;
    
    // Step 1: Yaw Offset 및 Frame Rotation 추정
    // 관계: Yaw_GLS = Yaw_VSLAM_preprocessed + Combined_Offset
    // Combined_Offset = Yaw_GLS - Yaw_VSLAM_preprocessed
    double sum_yaw_diff = 0.0;
    int valid_yaw_count = 0;
    
    for (const auto& pair : pairs) 
    {
        // Yaw는 이미 radians, 방향 통일됨
        double yaw_diff = normalizeAngle(pair.gls.yaw_rad - pair.vslam.yaw_rad);
        sum_yaw_diff += yaw_diff;
        valid_yaw_count++;
    }
    
    double avg_yaw_diff = sum_yaw_diff / valid_yaw_count;
    
    // calib.rotation은 총 Yaw 기준점 오프셋을 계산 (Yaw 방향 차이는 전처리로 제거됨)
    calib.rotation = avg_yaw_diff;
    calib.yaw_offset = 0.0;  // 3단계에서 잔차로 정제
    
    // Step 2: Centroid 기반 Translation 추정 (Rotation은 1단계 값 사용)
    double vslam_center_x = 0.0, vslam_center_y = 0.0;
    double gls_center_x = 0.0, gls_center_y = 0.0;
    
    for (const auto& pair : pairs) 
    {
        vslam_center_x += pair.vslam.x;
        vslam_center_y += pair.vslam.y;
        gls_center_x += pair.gls.x;
        gls_center_y += pair.gls.y;
    }
    
    vslam_center_x /= pairs.size();
    vslam_center_y /= pairs.size();
    gls_center_x /= pairs.size();
    gls_center_y /= pairs.size();
    
    // VSLAM 중심점을 Rotation으로 회전
    double cos_r = std::cos(calib.rotation);
    double sin_r = std::sin(calib.rotation);
    double vslam_center_x_rotated = cos_r * vslam_center_x - sin_r * vslam_center_y;
    double vslam_center_y_rotated = sin_r * vslam_center_x + cos_r * vslam_center_y;
    
    // Translation 계산: T = C_gls - R * C_vslam
    calib.translation_x = gls_center_x - vslam_center_x_rotated;
    calib.translation_y = gls_center_y - vslam_center_y_rotated;
    
    // Step 3: Yaw 잔차(Yaw Offset) 정제
    double sum_yaw_residual = 0.0;
    for (const auto& pair : pairs) {
        // Yaw_GLS - (Yaw_VSLAM_preprocessed + rotation)
        double predicted_gls_yaw = normalizeAngle(pair.vslam.yaw_rad + calib.rotation);
        double residual = normalizeAngle(pair.gls.yaw_rad - predicted_gls_yaw);
        sum_yaw_residual += residual;
    }
    calib.yaw_offset = sum_yaw_residual / pairs.size();
    
    return calib;
}

// RMS 오차 계산 및 출력
void calculateErrors(const std::vector<PosePair>& pairs, const CalibrationParams& calib) 
{
    double sum_x_sq_error = 0.0;
    double sum_y_sq_error = 0.0;
    double sum_yaw_sq_error = 0.0;
    
    // ... (출력 포맷 생략)
    
    for (size_t i = 0; i < pairs.size(); ++i) {
        Pose2D transformed = transformVSLAMtoGLS(pairs[i].vslam, calib);
        
        double x_error = transformed.x - pairs[i].gls.x;
        double y_error = transformed.y - pairs[i].gls.y;
        double yaw_error = normalizeAngle(transformed.yaw_rad - pairs[i].gls.yaw_rad);
        
        sum_x_sq_error += x_error * x_error;
        sum_y_sq_error += y_error * y_error;
        sum_yaw_sq_error += yaw_error * yaw_error;
        
        // 오차 출력 부분은 생략 (성능 지표 위주로 확인)
    }
    
    double rms_x = std::sqrt(sum_x_sq_error / pairs.size());
    double rms_y = std::sqrt(sum_y_sq_error / pairs.size());
    double rms_yaw = std::sqrt(sum_yaw_sq_error / pairs.size());
    
    std::cout << "\n=== RMS Errors ===" << std::endl;
    std::cout << "X RMS Error:   " << rms_x << " m" << std::endl;
    std::cout << "Y RMS Error:   " << rms_y << " m" << std::endl;
    std::cout << "Yaw RMS Error: " << rad2deg(rms_yaw) << " degrees" << std::endl;
    std::cout << "Position RMS:  " << std::sqrt(rms_x * rms_x + rms_y * rms_y) << " m" << std::endl;
}

// CSV 파일 읽기 및 전처리
std::vector<PosePair> readCSV(const std::string& filename) {
    std::vector<PosePair> pairs;
    std::ifstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open file " << filename << std::endl;
        return pairs;
    }
    
    std::string line;
    std::getline(file, line);  // Skip header
    
    while (std::getline(file, line)) 
    {
        std::stringstream ss(line);
        std::string token;
        PosePair pair;
        
        // VSLAM X, Y, Yaw (deg)
        std::getline(ss, token, ','); 
        pair.vslam.x = std::stod(token);
        std::getline(ss, token, ','); 
        pair.vslam.y = std::stod(token);
        std::getline(ss, token, ','); 
        double vslam_yaw_deg = std::stod(token);
        
        // GLS X, Y, Yaw (deg)
        std::getline(ss, token, ','); 
        pair.gls.x = std::stod(token);
        std::getline(ss, token, ','); 
        pair.gls.y = std::stod(token);
        std::getline(ss, token, ','); 
        double gls_yaw_deg = std::stod(token);

        // =========================================================
        // [핵심 수정: Yaw 방향 통일 전처리]
        // VSLAM Yaw를 GLS Yaw의 방향 (CW)과 일치시키기 위해 부호 반전
        pair.vslam.yaw_rad = deg2rad(-vslam_yaw_deg); 
        // GLS Yaw는 그대로 사용 (목표 방향, radians으로 변환)
        pair.gls.yaw_rad = deg2rad(gls_yaw_deg);
        // =========================================================
        
        pairs.push_back(pair);
    }
    
    file.close();
    return pairs;
}

int main(int argc, char* argv[]) 
{
    std::cout << std::fixed << std::setprecision(6);
    
    std::string filename = "pose_log_20251024_143111_930.csv";
    
    if (argc > 1) 
    {
        filename = argv[1];
    }
    
    std::cout << "=== GLS-VSLAM External Calibration (Pre-processed Yaw) ===" << std::endl;
    std::cout << "Reading file: " << filename << std::endl;
    
    // 1. 데이터 로드 및 전처리
    std::vector<PosePair> pairs = readCSV(filename);
    
    if (pairs.empty()) 
    {
        std::cerr << "Error: No valid data pairs found" << std::endl;
        return 1;
    }
    
    std::cout << "Loaded " << pairs.size() << " pose pairs" << std::endl;
    
    // 2. 캘리브레이션 수행
    CalibrationParams calib = calibrateExternalParameters(pairs);
    
    // 3. 결과 출력
    std::cout << "\n=== Calibration Parameters ===" << std::endl;
    std::cout << "Translation X: " << calib.translation_x << " m" << std::endl;
    std::cout << "Translation Y: " << calib.translation_y << " m" << std::endl;
    std::cout << "Frame Rotation: " << rad2deg(calib.rotation) << " degrees" << std::endl;
    std::cout << "Yaw Offset: " << rad2deg(calib.yaw_offset) << " degrees" << std::endl;
    
    // 4. 오차 계산 및 출력
    calculateErrors(pairs, calib);
    
    std::cout << "\n=== Transformation Formula (Yaw Pre-processed) ===" << std::endl;
    std::cout << "To transform VSLAM pose to GLS coordinate:" << std::endl;
    std::cout << "1. Pre-process VSLAM Yaw: VSLAM_yaw_rad = deg2rad(-VSLAM_yaw_deg)" << std::endl;
    std::cout << "2. Rotate VSLAM (x,y) by " << rad2deg(calib.rotation) << " degrees" << std::endl;
    std::cout << "3. Translate by (" << calib.translation_x << ", " << calib.translation_y << ")" << std::endl;
    std::cout << "4. GLS_yaw = VSLAM_yaw_rad + " << rad2deg(calib.rotation) << " + " << rad2deg(calib.yaw_offset) << std::endl;
    
    return 0;
}