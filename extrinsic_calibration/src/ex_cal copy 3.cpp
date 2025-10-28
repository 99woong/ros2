#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <string>
#include <iomanip>

struct Pose2D {
    double x;
    double y;
    double yaw;  // radians
};

struct PosePair {
    Pose2D vslam;
    Pose2D gls;
};

struct CalibrationParams {
    double translation_x;
    double translation_y;
    double rotation;  // radians
    double yaw_offset;  // radians
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

// Transform VSLAM pose to GLS coordinate system
Pose2D transformVSLAMtoGLS(const Pose2D& vslam_pose, const CalibrationParams& calib) {
    Pose2D gls_pose;
    
    // Apply rotation transformation (coordinate frame rotation)
    double cos_r = std::cos(calib.rotation);
    double sin_r = std::sin(calib.rotation);
    
    double x_rotated = cos_r * vslam_pose.x - sin_r * vslam_pose.y;
    double y_rotated = sin_r * vslam_pose.x + cos_r * vslam_pose.y;
    
    // Apply translation
    gls_pose.x = x_rotated + calib.translation_x;
    gls_pose.y = y_rotated + calib.translation_y;
    
    // Apply yaw transformation
    // VSLAM is CCW, GLS is CW, so we need to handle this
    gls_pose.yaw = normalizeAngle(-(vslam_pose.yaw + calib.rotation) + calib.yaw_offset);
    
    return gls_pose;
}

// Calculate calibration parameters using least squares approach
CalibrationParams calibrateExternalParameters(const std::vector<PosePair>& pairs) {
    if (pairs.empty()) {
        std::cerr << "Error: No pose pairs provided" << std::endl;
        return CalibrationParams{0, 0, 0, 0};
    }
    
    CalibrationParams calib;
    
    // Step 1: Estimate yaw offset and frame rotation
    // VSLAM: CCW positive, GLS: CW positive
    // Relationship: GLS_yaw = -VSLAM_yaw + rotation + yaw_offset
    // Rearranged: (rotation + yaw_offset) = GLS_yaw + VSLAM_yaw
    
    double sum_combined_offset = 0.0;
    int valid_yaw_count = 0;
    
    for (const auto& pair : pairs) {
        double vslam_yaw_rad = deg2rad(pair.vslam.yaw);
        double gls_yaw_rad = deg2rad(pair.gls.yaw);
        
        // GLS = -VSLAM + combined_offset (considering CW vs CCW)
        // combined_offset = GLS - (-VSLAM) = GLS + VSLAM
        double combined_offset = normalizeAngle(gls_yaw_rad - (-vslam_yaw_rad));
        sum_combined_offset += combined_offset;
        valid_yaw_count++;
    }
    
    double avg_combined_offset = sum_combined_offset / valid_yaw_count;
    
    // Initially assume rotation is the dominant part
    calib.rotation = avg_combined_offset;
    calib.yaw_offset = 0.0;  // Will be refined
    
    // Step 2: Estimate translation using centroid method
    double vslam_center_x = 0.0, vslam_center_y = 0.0;
    double gls_center_x = 0.0, gls_center_y = 0.0;
    
    for (const auto& pair : pairs) {
        vslam_center_x += pair.vslam.x;
        vslam_center_y += pair.vslam.y;
        gls_center_x += pair.gls.x;
        gls_center_y += pair.gls.y;
    }
    
    vslam_center_x /= pairs.size();
    vslam_center_y /= pairs.size();
    gls_center_x /= pairs.size();
    gls_center_y /= pairs.size();
    
    // Rotate VSLAM center
    double cos_r = std::cos(calib.rotation);
    double sin_r = std::sin(calib.rotation);
    double vslam_center_x_rotated = cos_r * vslam_center_x - sin_r * vslam_center_y;
    double vslam_center_y_rotated = sin_r * vslam_center_x + cos_r * vslam_center_y;
    
    calib.translation_x = gls_center_x - vslam_center_x_rotated;
    calib.translation_y = gls_center_y - vslam_center_y_rotated;
    
    // Step 3: Refine yaw offset
    double sum_yaw_residual = 0.0;
    for (const auto& pair : pairs) {
        double vslam_yaw_rad = deg2rad(pair.vslam.yaw);
        double gls_yaw_rad = deg2rad(pair.gls.yaw);
        double predicted_gls_yaw = normalizeAngle(-(vslam_yaw_rad + calib.rotation));
        double residual = normalizeAngle(gls_yaw_rad - predicted_gls_yaw);
        sum_yaw_residual += residual;
    }
    calib.yaw_offset = sum_yaw_residual / pairs.size();
    
    return calib;
}

// Calculate RMS errors after calibration
void calculateErrors(const std::vector<PosePair>& pairs, const CalibrationParams& calib) {
    double sum_x_error = 0.0;
    double sum_y_error = 0.0;
    double sum_yaw_error = 0.0;
    
    std::cout << "\n=== Error Analysis ===" << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Sample | X_err(m) | Y_err(m) | Yaw_err(deg)" << std::endl;
    std::cout << "-------|----------|----------|-------------" << std::endl;
    
    for (size_t i = 0; i < pairs.size(); ++i) {
        Pose2D transformed = transformVSLAMtoGLS(pairs[i].vslam, calib);
        
        double x_error = transformed.x - pairs[i].gls.x;
        double y_error = transformed.y - pairs[i].gls.y;
        double yaw_error = normalizeAngle(transformed.yaw - deg2rad(pairs[i].gls.yaw));
        
        sum_x_error += x_error * x_error;
        sum_y_error += y_error * y_error;
        sum_yaw_error += yaw_error * yaw_error;
        
        std::cout << std::setw(6) << i + 1 << " | "
                  << std::setw(8) << x_error << " | "
                  << std::setw(8) << y_error << " | "
                  << std::setw(11) << rad2deg(yaw_error) << std::endl;
    }
    
    double rms_x = std::sqrt(sum_x_error / pairs.size());
    double rms_y = std::sqrt(sum_y_error / pairs.size());
    double rms_yaw = std::sqrt(sum_yaw_error / pairs.size());
    
    std::cout << "\n=== RMS Errors ===" << std::endl;
    std::cout << "X RMS Error:   " << rms_x << " m" << std::endl;
    std::cout << "Y RMS Error:   " << rms_y << " m" << std::endl;
    std::cout << "Yaw RMS Error: " << rad2deg(rms_yaw) << " degrees" << std::endl;
    std::cout << "Position RMS:  " << std::sqrt(rms_x * rms_x + rms_y * rms_y) << " m" << std::endl;
}

// Read CSV file
std::vector<PosePair> readCSV(const std::string& filename) {
    std::vector<PosePair> pairs;
    std::ifstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open file " << filename << std::endl;
        return pairs;
    }
    
    std::string line;
    std::getline(file, line);  // Skip header
    
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;
        PosePair pair;
        
        std::getline(ss, token, ','); pair.vslam.x = std::stod(token);
        std::getline(ss, token, ','); pair.vslam.y = std::stod(token);
        std::getline(ss, token, ','); pair.vslam.yaw = std::stod(token);
        std::getline(ss, token, ','); pair.gls.x = std::stod(token);
        std::getline(ss, token, ','); pair.gls.y = std::stod(token);
        std::getline(ss, token, ','); pair.gls.yaw = std::stod(token);
        
        pairs.push_back(pair);
    }
    
    file.close();
    return pairs;
}

int main(int argc, char* argv[]) {
    std::string filename = "pose_log_20251024_143111_930.csv";
    
    if (argc > 1) {
        filename = argv[1];
    }
    
    std::cout << "=== GLS-VSLAM External Calibration ===" << std::endl;
    std::cout << "Reading file: " << filename << std::endl;
    
    std::vector<PosePair> pairs = readCSV(filename);
    
    if (pairs.empty()) {
        std::cerr << "Error: No valid data pairs found" << std::endl;
        return 1;
    }
    
    std::cout << "Loaded " << pairs.size() << " pose pairs" << std::endl;
    
    // Perform calibration
    CalibrationParams calib = calibrateExternalParameters(pairs);
    
    // Display results
    std::cout << "\n=== Calibration Parameters ===" << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Translation X: " << calib.translation_x << " m" << std::endl;
    std::cout << "Translation Y: " << calib.translation_y << " m" << std::endl;
    std::cout << "Frame Rotation: " << rad2deg(calib.rotation) << " degrees" << std::endl;
    std::cout << "Yaw Offset: " << rad2deg(calib.yaw_offset) << " degrees" << std::endl;
    
    // Calculate and display errors
    calculateErrors(pairs, calib);
    
    // Display transformation formula
    std::cout << "\n=== Transformation Formula ===" << std::endl;
    std::cout << "To transform VSLAM pose to GLS coordinate:" << std::endl;
    std::cout << "1. Rotate VSLAM (x,y) by " << rad2deg(calib.rotation) << " degrees" << std::endl;
    std::cout << "2. Translate by (" << calib.translation_x << ", " << calib.translation_y << ")" << std::endl;
    std::cout << "3. GLS_yaw = -(VSLAM_yaw + " << rad2deg(calib.rotation) << ") + " << rad2deg(calib.yaw_offset) << std::endl;
    
    return 0;
}