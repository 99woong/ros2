#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <string>
#include <iomanip>
#include <algorithm>

struct Pose2D {
    double x;
    double y;
    double yaw;  // degrees
};

struct PosePair {
    Pose2D vslam;
    Pose2D gls;
};

struct CalibrationParams {
    double offset_x;      // VSLAM offset from GLS in robot frame
    double offset_y;      
    double vslam_origin_x_in_gls;  
    double vslam_origin_y_in_gls;  
    double heading_offset_deg;
    bool coordinate_swap;  // If true, VSLAM uses swapped X-Y compared to GLS
};

// Normalize angle to [-180, 180]
double normalizeAngleDeg(double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
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

// Analyze data patterns
void analyzeDataPatterns(const std::vector<PosePair>& pairs) {
    std::cout << "\n=== Data Pattern Analysis ===" << std::endl;
    
    // Position statistics
    double vslam_x_min = 1e10, vslam_x_max = -1e10;
    double vslam_y_min = 1e10, vslam_y_max = -1e10;
    double gls_x_min = 1e10, gls_x_max = -1e10;
    double gls_y_min = 1e10, gls_y_max = -1e10;
    
    for (const auto& pair : pairs) {
        vslam_x_min = std::min(vslam_x_min, pair.vslam.x);
        vslam_x_max = std::max(vslam_x_max, pair.vslam.x);
        vslam_y_min = std::min(vslam_y_min, pair.vslam.y);
        vslam_y_max = std::max(vslam_y_max, pair.vslam.y);
        gls_x_min = std::min(gls_x_min, pair.gls.x);
        gls_x_max = std::max(gls_x_max, pair.gls.x);
        gls_y_min = std::min(gls_y_min, pair.gls.y);
        gls_y_max = std::max(gls_y_max, pair.gls.y);
    }
    
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\nPosition ranges:" << std::endl;
    std::cout << "VSLAM X: [" << vslam_x_min << ", " << vslam_x_max << "] range=" << (vslam_x_max - vslam_x_min) << "m" << std::endl;
    std::cout << "VSLAM Y: [" << vslam_y_min << ", " << vslam_y_max << "] range=" << (vslam_y_max - vslam_y_min) << "m" << std::endl;
    std::cout << "GLS X:   [" << gls_x_min << ", " << gls_x_max << "] range=" << (gls_x_max - gls_x_min) << "m" << std::endl;
    std::cout << "GLS Y:   [" << gls_y_min << ", " << gls_y_max << "] range=" << (gls_y_max - gls_y_min) << "m" << std::endl;
    
    // Check if coordinate axes might be swapped
    double vslam_x_range = vslam_x_max - vslam_x_min;
    double vslam_y_range = vslam_y_max - vslam_y_min;
    double gls_x_range = gls_x_max - gls_x_min;
    double gls_y_range = gls_y_max - gls_y_min;
    
    std::cout << "\nCoordinate axis analysis:" << std::endl;
    if (std::abs(vslam_x_range - gls_y_range) < 0.05 && std::abs(vslam_y_range - gls_x_range) < 0.05) {
        std::cout << "→ POSSIBLE COORDINATE SWAP: VSLAM X ≈ GLS Y, VSLAM Y ≈ GLS X" << std::endl;
    } else if (std::abs(vslam_x_range - gls_x_range) < 0.05 && std::abs(vslam_y_range - gls_y_range) < 0.05) {
        std::cout << "→ Coordinate axes appear aligned (no swap)" << std::endl;
    } else {
        std::cout << "→ Coordinate relationship unclear" << std::endl;
    }
    
    // Heading analysis
    std::cout << "\nHeading analysis:" << std::endl;
    std::cout << std::setw(8) << "Sample" << " | " 
              << std::setw(10) << "VSLAM(deg)" << " | " 
              << std::setw(10) << "GLS(deg)" << " | "
              << std::setw(10) << "Diff" << " | "
              << std::setw(10) << "GLS+VSLAM" << std::endl;
    std::cout << std::string(60, '-') << std::endl;
    
    double sum_diff = 0.0;
    double sum_sum = 0.0;
    
    for (size_t i = 0; i < std::min(size_t(10), pairs.size()); ++i) {
        double diff = normalizeAngleDeg(pairs[i].gls.yaw - pairs[i].vslam.yaw);
        double sum = normalizeAngleDeg(pairs[i].gls.yaw + pairs[i].vslam.yaw);
        
        sum_diff += diff;
        sum_sum += sum;
        
        std::cout << std::setw(8) << i+1 << " | " 
                  << std::setw(10) << pairs[i].vslam.yaw << " | " 
                  << std::setw(10) << pairs[i].gls.yaw << " | "
                  << std::setw(10) << diff << " | "
                  << std::setw(10) << sum << std::endl;
    }
    
    double avg_diff = sum_diff / std::min(size_t(10), pairs.size());
    double avg_sum = sum_sum / std::min(size_t(10), pairs.size());
    
    std::cout << "\nAverage GLS - VSLAM: " << avg_diff << " degrees" << std::endl;
    std::cout << "Average GLS + VSLAM: " << avg_sum << " degrees" << std::endl;
    
    if (std::abs(avg_sum) < 10.0) {
        std::cout << "→ GLS ≈ -VSLAM (opposite direction: CW vs CCW)" << std::endl;
    } else if (std::abs(avg_diff) < 10.0) {
        std::cout << "→ GLS ≈ VSLAM (same direction)" << std::endl;
    } else {
        std::cout << "→ Heading offset ≈ " << avg_diff << " degrees" << std::endl;
    }
    
    // Simple offset estimation
    std::cout << "\n=== Simple Offset Estimation ===" << std::endl;
    double avg_vslam_x = 0, avg_vslam_y = 0;
    double avg_gls_x = 0, avg_gls_y = 0;
    
    for (const auto& pair : pairs) {
        avg_vslam_x += pair.vslam.x;
        avg_vslam_y += pair.vslam.y;
        avg_gls_x += pair.gls.x;
        avg_gls_y += pair.gls.y;
    }
    
    avg_vslam_x /= pairs.size();
    avg_vslam_y /= pairs.size();
    avg_gls_x /= pairs.size();
    avg_gls_y /= pairs.size();
    
    std::cout << "Average positions:" << std::endl;
    std::cout << "VSLAM: (" << avg_vslam_x << ", " << avg_vslam_y << ")" << std::endl;
    std::cout << "GLS:   (" << avg_gls_x << ", " << avg_gls_y << ")" << std::endl;
    std::cout << "Simple difference (ignoring rotation):" << std::endl;
    std::cout << "  ΔX = " << (avg_gls_x - avg_vslam_x) << " m" << std::endl;
    std::cout << "  ΔY = " << (avg_gls_y - avg_vslam_y) << " m" << std::endl;
}

// Simple calibration assuming fixed offset
CalibrationParams simpleCalibration(const std::vector<PosePair>& pairs) {
    CalibrationParams calib;
    
    // Step 1: Determine heading relationship
    // GLS is CW, VSLAM is CCW, so GLS_heading = -VSLAM_heading + offset
    double sum_heading_combined = 0.0;
    for (const auto& pair : pairs) {
        double combined = normalizeAngleDeg(pair.gls.yaw + pair.vslam.yaw);
        sum_heading_combined += combined;
    }
    
    calib.heading_offset_deg = sum_heading_combined / pairs.size();
    
    // Step 2: Assume sensor offset is small initially
    calib.offset_x = 0.30;  // 30cm forward (initial guess)
    calib.offset_y = -0.06; // 6cm lateral (initial guess)
    
    // Step 3: Compute VSLAM origin in GLS frame
    // Without considering sensor offset first
    double sum_origin_x = 0.0;
    double sum_origin_y = 0.0;
    
    for (const auto& pair : pairs) {
        // Simple: GLS_pos ≈ VSLAM_pos + origin
        sum_origin_x += pair.gls.x - pair.vslam.x;
        sum_origin_y += pair.gls.y - pair.vslam.y;
    }
    
    calib.vslam_origin_x_in_gls = sum_origin_x / pairs.size();
    calib.vslam_origin_y_in_gls = sum_origin_y / pairs.size();
    
    return calib;
}

// Evaluate calibration
void evaluateCalibration(const std::vector<PosePair>& pairs, const CalibrationParams& calib) {
    std::cout << "\n=== Calibration Results ===" << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Heading offset (GLS = -VSLAM + offset): " << calib.heading_offset_deg << " deg" << std::endl;
    std::cout << "VSLAM origin in GLS: (" << calib.vslam_origin_x_in_gls << ", " << calib.vslam_origin_y_in_gls << ")" << std::endl;
    std::cout << "Sensor offset (initial guess): (" << calib.offset_x << ", " << calib.offset_y << ")" << std::endl;
    
    // Calculate residuals without sensor offset compensation
    std::vector<double> x_errors, y_errors, yaw_errors;
    
    std::cout << "\n=== Residual Errors (Ignoring Sensor Offset) ===" << std::endl;
    std::cout << std::setw(8) << "Sample" << " | " 
              << std::setw(10) << "X_err(m)" << " | " 
              << std::setw(10) << "Y_err(m)" << " | "
              << std::setw(12) << "Yaw_err(deg)" << std::endl;
    std::cout << std::string(50, '-') << std::endl;
    
    for (size_t i = 0; i < std::min(size_t(20), pairs.size()); ++i) {
        const auto& pair = pairs[i];
        
        // Predicted GLS position (simple model)
        double pred_gls_x = pair.vslam.x + calib.vslam_origin_x_in_gls;
        double pred_gls_y = pair.vslam.y + calib.vslam_origin_y_in_gls;
        double pred_gls_yaw = normalizeAngleDeg(-pair.vslam.yaw + calib.heading_offset_deg);
        
        double x_err = pred_gls_x - pair.gls.x;
        double y_err = pred_gls_y - pair.gls.y;
        double yaw_err = normalizeAngleDeg(pred_gls_yaw - pair.gls.yaw);
        
        x_errors.push_back(x_err);
        y_errors.push_back(y_err);
        yaw_errors.push_back(yaw_err);
        
        std::cout << std::setw(8) << i+1 << " | " 
                  << std::setw(10) << x_err << " | " 
                  << std::setw(10) << y_err << " | "
                  << std::setw(12) << yaw_err << std::endl;
    }
    
    // Statistics
    double sum_x = 0, sum_y = 0, sum_yaw = 0;
    double sum_x2 = 0, sum_y2 = 0, sum_yaw2 = 0;
    
    for (size_t i = 0; i < pairs.size(); ++i) {
        const auto& pair = pairs[i];
        double pred_gls_x = pair.vslam.x + calib.vslam_origin_x_in_gls;
        double pred_gls_y = pair.vslam.y + calib.vslam_origin_y_in_gls;
        double pred_gls_yaw = normalizeAngleDeg(-pair.vslam.yaw + calib.heading_offset_deg);
        
        double x_err = pred_gls_x - pair.gls.x;
        double y_err = pred_gls_y - pair.gls.y;
        double yaw_err = normalizeAngleDeg(pred_gls_yaw - pair.gls.yaw);
        
        sum_x += x_err;
        sum_y += y_err;
        sum_yaw += yaw_err;
        sum_x2 += x_err * x_err;
        sum_y2 += y_err * y_err;
        sum_yaw2 += yaw_err * yaw_err;
    }
    
    double mean_x = sum_x / pairs.size();
    double mean_y = sum_y / pairs.size();
    double mean_yaw = sum_yaw / pairs.size();
    
    double std_x = std::sqrt(sum_x2 / pairs.size() - mean_x * mean_x);
    double std_y = std::sqrt(sum_y2 / pairs.size() - mean_y * mean_y);
    double std_yaw = std::sqrt(sum_yaw2 / pairs.size() - mean_yaw * mean_yaw);
    
    std::cout << "\n=== Error Statistics ===" << std::endl;
    std::cout << "X:   Mean=" << std::setw(9) << mean_x << " m, Std=" << std::setw(9) << std_x << " m" << std::endl;
    std::cout << "Y:   Mean=" << std::setw(9) << mean_y << " m, Std=" << std::setw(9) << std_y << " m" << std::endl;
    std::cout << "Yaw: Mean=" << std::setw(9) << mean_yaw << " deg, Std=" << std::setw(9) << std_yaw << " deg" << std::endl;
    
    std::cout << "\n=== Interpretation ===" << std::endl;
    std::cout << "If mean errors are close to sensor offset (~0.3m), then the model is correct." << std::endl;
    std::cout << "If std deviations are small (<5cm, <2deg), VSLAM consistency is good." << std::endl;
    
    if (std_yaw < 2.0) {
        std::cout << "\n✓ Yaw consistency is GOOD (std < 2 degrees)" << std::endl;
    }
    
    if (std_x < 0.05 && std_y < 0.05) {
        std::cout << "✓ Position consistency is EXCELLENT (std < 5cm)" << std::endl;
    } else if (std_x < 0.10 && std_y < 0.10) {
        std::cout << "✓ Position consistency is GOOD (std < 10cm)" << std::endl;
    } else {
        std::cout << "✗ Position consistency needs improvement (std > 10cm)" << std::endl;
    }
}

int main(int argc, char* argv[]) {
    std::string filename = "pose_log_20251024_143111_930.csv";
    
    if (argc > 1) {
        filename = argv[1];
    }
    
    std::cout << "=== GLS-VSLAM Data Analysis Tool ===" << std::endl;
    std::cout << "Reading file: " << filename << std::endl;
    
    std::vector<PosePair> pairs = readCSV(filename);
    
    if (pairs.empty()) {
        std::cerr << "Error: No valid data pairs found" << std::endl;
        return 1;
    }
    
    std::cout << "Loaded " << pairs.size() << " pose pairs" << std::endl;
    
    // Analyze data patterns
    analyzeDataPatterns(pairs);
    
    // Simple calibration
    CalibrationParams calib = simpleCalibration(pairs);
    
    // Evaluate
    evaluateCalibration(pairs, calib);
    
    return 0;
}