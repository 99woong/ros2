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
    // Physical sensor offset (VSLAM relative to GLS in robot body frame)
    double offset_forward;   // Forward offset (positive = VSLAM is in front)
    double offset_lateral;   // Lateral offset (positive = VSLAM is to the right)
    
    // Map origin (where VSLAM's (0,0) is in GLS coordinate)
    double vslam_origin_gls_x;
    double vslam_origin_gls_y;
    
    // Heading offset
    double heading_offset_deg;
};

// Normalize angle to [-180, 180]
double normalizeAngleDeg(double angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}

double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}

// Transform VSLAM pose to GLS center position
// Key insight: VSLAM X-axis ≈ GLS Y-axis, VSLAM Y-axis ≈ GLS X-axis
Pose2D transformVSLAMtoGLS(const Pose2D& vslam, const CalibrationParams& calib) {
    Pose2D gls_predicted;
    
    // Step 1: Convert VSLAM heading to GLS heading
    // GLS_heading = -VSLAM_heading + offset
    double gls_heading_deg = normalizeAngleDeg(-vslam.yaw + calib.heading_offset_deg);
    double gls_heading_rad = deg2rad(gls_heading_deg);
    
    // Step 2: Convert VSLAM position to GLS coordinate (with axis swap)
    // VSLAM X → GLS Y direction
    // VSLAM Y → GLS X direction
    double vslam_sensor_gls_x = -vslam.y + calib.vslam_origin_gls_x;
    double vslam_sensor_gls_y = vslam.x + calib.vslam_origin_gls_y;
    
    // Step 3: Account for sensor offset from robot center
    // The sensor is offset from the center, so when robot rotates,
    // the sensor position changes relative to the center
    double cos_h = std::cos(gls_heading_rad);
    double sin_h = std::sin(gls_heading_rad);
    
    // Offset in robot body frame → offset in GLS frame
    double offset_gls_x = cos_h * calib.offset_lateral - sin_h * calib.offset_forward;
    double offset_gls_y = sin_h * calib.offset_lateral + cos_h * calib.offset_forward;
    
    // GLS center = VSLAM sensor position - offset
    gls_predicted.x = vslam_sensor_gls_x - offset_gls_x;
    gls_predicted.y = vslam_sensor_gls_y - offset_gls_y;
    gls_predicted.yaw = gls_heading_deg;
    
    return gls_predicted;
}

// Calibrate with proper axis swap
CalibrationParams calibrateWithAxisSwap(const std::vector<PosePair>& pairs) {
    CalibrationParams calib;
    
    // Step 1: Estimate heading offset
    double sum_heading = 0.0;
    for (const auto& pair : pairs) {
        // GLS = -VSLAM + offset
        double offset = normalizeAngleDeg(pair.gls.yaw + pair.vslam.yaw);
        sum_heading += offset;
    }
    calib.heading_offset_deg = sum_heading / pairs.size();
    
    // Step 2: Initial guess for sensor offset (from physical measurement)
    calib.offset_forward = 0.30;   // 30cm forward
    calib.offset_lateral = -0.06;  // 6cm to the left
    
    // Step 3: Estimate VSLAM origin with axis swap
    // GLS_x ≈ -VSLAM_y + origin_x
    // GLS_y ≈ VSLAM_x + origin_y
    double sum_origin_x = 0.0;
    double sum_origin_y = 0.0;
    
    for (const auto& pair : pairs) {
        sum_origin_x += pair.gls.x + pair.vslam.y;
        sum_origin_y += pair.gls.y - pair.vslam.x;
    }
    
    calib.vslam_origin_gls_x = sum_origin_x / pairs.size();
    calib.vslam_origin_gls_y = sum_origin_y / pairs.size();
    
    // Step 4: Refine sensor offset using optimization
    const int max_iter = 100;
    const double learning_rate = 0.001;
    
    for (int iter = 0; iter < max_iter; ++iter) {
        double grad_forward = 0.0;
        double grad_lateral = 0.0;
        double grad_origin_x = 0.0;
        double grad_origin_y = 0.0;
        double grad_heading = 0.0;
        
        double total_error = 0.0;
        
        for (const auto& pair : pairs) {
            Pose2D predicted = transformVSLAMtoGLS(pair.vslam, calib);
            
            double err_x = predicted.x - pair.gls.x;
            double err_y = predicted.y - pair.gls.y;
            double err_yaw = normalizeAngleDeg(predicted.yaw - pair.gls.yaw);
            
            total_error += err_x * err_x + err_y * err_y + 0.01 * err_yaw * err_yaw;
            
            double h_rad = deg2rad(predicted.yaw);
            double cos_h = std::cos(h_rad);
            double sin_h = std::sin(h_rad);
            
            // Gradients
            grad_forward += err_x * sin_h - err_y * cos_h;
            grad_lateral += err_x * (-cos_h) - err_y * (-sin_h);
            grad_origin_x += err_x;
            grad_origin_y += err_y;
            
            // Heading gradient (approximate)
            double d_x = err_x * (-sin_h * calib.offset_lateral - cos_h * calib.offset_forward);
            double d_y = err_y * (cos_h * calib.offset_lateral - sin_h * calib.offset_forward);
            grad_heading += (d_x + d_y + 0.01 * err_yaw) * deg2rad(1.0);
        }
        
        // Update
        calib.offset_forward -= learning_rate * grad_forward / pairs.size();
        calib.offset_lateral -= learning_rate * grad_lateral / pairs.size();
        calib.vslam_origin_gls_x -= learning_rate * grad_origin_x / pairs.size();
        calib.vslam_origin_gls_y -= learning_rate * grad_origin_y / pairs.size();
        calib.heading_offset_deg -= rad2deg(learning_rate * grad_heading / pairs.size());
        
        if (iter % 20 == 0) {
            std::cout << "Iteration " << iter << ", Error: " << total_error << std::endl;
        }
        
        if (total_error < 0.0001) break;
    }
    
    return calib;
}

// Evaluate calibration
void evaluateCalibration(const std::vector<PosePair>& pairs, const CalibrationParams& calib) {
    std::vector<double> x_errors, y_errors, yaw_errors;
    
    std::cout << "\n=== Calibration Results ===" << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    
    for (const auto& pair : pairs) {
        Pose2D predicted = transformVSLAMtoGLS(pair.vslam, calib);
        
        double err_x = predicted.x - pair.gls.x;
        double err_y = predicted.y - pair.gls.y;
        double err_yaw = normalizeAngleDeg(predicted.yaw - pair.gls.yaw);
        
        x_errors.push_back(err_x);
        y_errors.push_back(err_y);
        yaw_errors.push_back(err_yaw);
    }
    
    // Statistics
    double mean_x = 0, mean_y = 0, mean_yaw = 0;
    for (size_t i = 0; i < pairs.size(); ++i) {
        mean_x += x_errors[i];
        mean_y += y_errors[i];
        mean_yaw += yaw_errors[i];
    }
    mean_x /= pairs.size();
    mean_y /= pairs.size();
    mean_yaw /= pairs.size();
    
    double var_x = 0, var_y = 0, var_yaw = 0;
    for (size_t i = 0; i < pairs.size(); ++i) {
        var_x += (x_errors[i] - mean_x) * (x_errors[i] - mean_x);
        var_y += (y_errors[i] - mean_y) * (y_errors[i] - mean_y);
        var_yaw += (yaw_errors[i] - mean_yaw) * (yaw_errors[i] - mean_yaw);
    }
    
    double std_x = std::sqrt(var_x / pairs.size());
    double std_y = std::sqrt(var_y / pairs.size());
    double std_yaw = std::sqrt(var_yaw / pairs.size());
    
    double rms_x = std::sqrt(var_x / pairs.size() + mean_x * mean_x);
    double rms_y = std::sqrt(var_y / pairs.size() + mean_y * mean_y);
    double rms_yaw = std::sqrt(var_yaw / pairs.size() + mean_yaw * mean_yaw);
    
    std::cout << "\n=== Error Statistics ===" << std::endl;
    std::cout << "Position X:" << std::endl;
    std::cout << "  Mean = " << std::setw(10) << mean_x*1000 << " mm" << std::endl;
    std::cout << "  Std  = " << std::setw(10) << std_x*1000 << " mm" << std::endl;
    std::cout << "  RMS  = " << std::setw(10) << rms_x*1000 << " mm" << std::endl;
    
    std::cout << "\nPosition Y:" << std::endl;
    std::cout << "  Mean = " << std::setw(10) << mean_y*1000 << " mm" << std::endl;
    std::cout << "  Std  = " << std::setw(10) << std_y*1000 << " mm" << std::endl;
    std::cout << "  RMS  = " << std::setw(10) << rms_y*1000 << " mm" << std::endl;
    
    std::cout << "\nHeading (Yaw):" << std::endl;
    std::cout << "  Mean = " << std::setw(10) << mean_yaw << " deg" << std::endl;
    std::cout << "  Std  = " << std::setw(10) << std_yaw << " deg" << std::endl;
    std::cout << "  RMS  = " << std::setw(10) << rms_yaw << " deg" << std::endl;
    
    std::cout << "\n2D Position RMS: " << std::sqrt(rms_x*rms_x + rms_y*rms_y)*1000 << " mm" << std::endl;
    
    // Show first 20 samples
    std::cout << "\n=== Sample Errors (First 20) ===" << std::endl;
    std::cout << std::setw(6) << "ID" << " | "
              << std::setw(10) << "X_err(mm)" << " | "
              << std::setw(10) << "Y_err(mm)" << " | "
              << std::setw(10) << "Yaw(deg)" << std::endl;
    std::cout << std::string(50, '-') << std::endl;
    
    for (size_t i = 0; i < std::min(size_t(20), pairs.size()); ++i) {
        std::cout << std::setw(6) << i+1 << " | "
                  << std::setw(10) << x_errors[i]*1000 << " | "
                  << std::setw(10) << y_errors[i]*1000 << " | "
                  << std::setw(10) << yaw_errors[i] << std::endl;
    }
    
    // VSLAM Performance Assessment
    std::cout << "\n=== VSLAM Consistency Assessment ===" << std::endl;
    std::cout << "Standard deviation measures repeatability (lower = better)" << std::endl;
    std::cout << std::endl;
    
    int score = 0;
    if (std_x < 0.01 && std_y < 0.01) score += 2;
    else if (std_x < 0.03 && std_y < 0.03) score += 1;
    
    if (std_yaw < 1.0) score += 2;
    else if (std_yaw < 3.0) score += 1;
    
    if (score >= 4) {
        std::cout << "★★★★★ EXCELLENT" << std::endl;
        std::cout << "Position repeatability: " << std::max(std_x, std_y)*1000 << " mm" << std::endl;
        std::cout << "Heading repeatability: " << std_yaw << " degrees" << std::endl;
        std::cout << "Your VSLAM shows very high consistency!" << std::endl;
    } else if (score >= 3) {
        std::cout << "★★★★☆ VERY GOOD" << std::endl;
        std::cout << "Position repeatability: " << std::max(std_x, std_y)*1000 << " mm" << std::endl;
        std::cout << "Heading repeatability: " << std_yaw << " degrees" << std::endl;
        std::cout << "Your VSLAM shows good consistency." << std::endl;
    } else if (score >= 2) {
        std::cout << "★★★☆☆ GOOD" << std::endl;
        std::cout << "VSLAM consistency is acceptable for most applications." << std::endl;
    } else {
        std::cout << "★★☆☆☆ FAIR" << std::endl;
        std::cout << "VSLAM shows noticeable variance. Consider investigating:" << std::endl;
        std::cout << "- Lighting conditions" << std::endl;
        std::cout << "- Feature-rich environment" << std::endl;
        std::cout << "- Camera calibration" << std::endl;
    }
}

// Read CSV
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
    
    std::cout << "=== GLS-VSLAM Calibration (Final) ===" << std::endl;
    std::cout << "Reading: " << filename << std::endl;
    
    std::vector<PosePair> pairs = readCSV(filename);
    
    if (pairs.empty()) {
        std::cerr << "Error: No data" << std::endl;
        return 1;
    }
    
    std::cout << "Loaded " << pairs.size() << " samples" << std::endl;
    std::cout << "\nCalibrating..." << std::endl;
    
    CalibrationParams calib = calibrateWithAxisSwap(pairs);
    
    std::cout << "\n=== Calibration Parameters ===" << std::endl;
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Sensor offset (VSLAM from GLS in robot frame):" << std::endl;
    std::cout << "  Forward: " << calib.offset_forward*1000 << " mm" << std::endl;
    std::cout << "  Lateral: " << calib.offset_lateral*1000 << " mm (negative = left)" << std::endl;
    std::cout << "\nVSLAM map origin in GLS coordinate:" << std::endl;
    std::cout << "  X: " << calib.vslam_origin_gls_x << " m" << std::endl;
    std::cout << "  Y: " << calib.vslam_origin_gls_y << " m" << std::endl;
    std::cout << "\nHeading offset:" << std::endl;
    std::cout << "  GLS_heading = -VSLAM_heading + " << calib.heading_offset_deg << " deg" << std::endl;
    
    std::cout << "\nCoordinate transformation:" << std::endl;
    std::cout << "  VSLAM X-axis → GLS Y-axis" << std::endl;
    std::cout << "  VSLAM Y-axis → GLS X-axis (inverted)" << std::endl;
    
    evaluateCalibration(pairs, calib);
    
    return 0;
}