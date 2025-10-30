#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <string>
#include <iomanip>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct Pose2D {
    double x;
    double y;
    double yaw_rad;
};

struct PosePair {
    Pose2D vslam;
    Pose2D gls;
    double weight;
};

struct CalibrationParams {
    double map_origin_x;
    double map_origin_y;
    double rotation;
    double yaw_offset;
    double sensor_offset_forward;  // VSLAM forward of GLS (robot X)
    double sensor_offset_lateral;  // VSLAM lateral of GLS (robot Y)
};

double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}

// Transform with sensor offset consideration
Pose2D transformVSLAMtoGLS(const Pose2D& vslam_pose, const CalibrationParams& calib, 
                          bool use_sensor_offset = true) {
    Pose2D gls_pose;
    
    // Step 1: Robot heading in GLS frame
    double robot_heading_gls = normalizeAngle(-vslam_pose.yaw_rad + calib.rotation + calib.yaw_offset);
    
    // Step 2: VSLAM sensor position in GLS frame
    double cos_r = std::cos(calib.rotation);
    double sin_r = std::sin(calib.rotation);
    
    double vslam_sensor_x = cos_r * vslam_pose.x - sin_r * vslam_pose.y + calib.map_origin_x;
    double vslam_sensor_y = sin_r * vslam_pose.x + cos_r * vslam_pose.y + calib.map_origin_y;
    
    // Step 3: Compensate sensor offset to get GLS center
    if (use_sensor_offset) {
        // Robot heading determines offset direction in GLS frame
        double cos_h = std::cos(robot_heading_gls);
        double sin_h = std::sin(robot_heading_gls);
        
        // Transform sensor offset from robot frame to GLS frame
        // Robot forward in GLS = direction of robot_heading_gls
        double offset_x_gls = cos_h * calib.sensor_offset_forward - sin_h * calib.sensor_offset_lateral;
        double offset_y_gls = sin_h * calib.sensor_offset_forward + cos_h * calib.sensor_offset_lateral;
        
        gls_pose.x = vslam_sensor_x - offset_x_gls;
        gls_pose.y = vslam_sensor_y - offset_y_gls;
    } else {
        gls_pose.x = vslam_sensor_x;
        gls_pose.y = vslam_sensor_y;
    }
    
    gls_pose.yaw_rad = robot_heading_gls;
    
    return gls_pose;
}

double computeCost(const std::vector<PosePair>& pairs, const CalibrationParams& calib,
                   bool use_sensor_offset = true, bool use_weights = true) {
    double cost = 0.0;
    
    for (const auto& pair : pairs) {
        Pose2D predicted = transformVSLAMtoGLS(pair.vslam, calib, use_sensor_offset);
        
        double err_x = predicted.x - pair.gls.x;
        double err_y = predicted.y - pair.gls.y;
        double err_yaw = normalizeAngle(predicted.yaw_rad - pair.gls.yaw_rad);
        
        double weight = use_weights ? pair.weight : 1.0;
        cost += weight * (err_x * err_x + err_y * err_y + 0.01 * err_yaw * err_yaw);
    }
    
    return cost;
}

void computeJacobianAndResidual(const std::vector<PosePair>& pairs,
                                const CalibrationParams& calib,
                                std::vector<std::vector<double>>& J,
                                std::vector<double>& r,
                                bool use_sensor_offset = true,
                                bool use_weights = true) {
    int n_residuals = pairs.size() * 3;
    int n_params = use_sensor_offset ? 6 : 4;
    
    J.assign(n_residuals, std::vector<double>(n_params, 0.0));
    r.assign(n_residuals, 0.0);
    
    for (size_t i = 0; i < pairs.size(); ++i) {
        const auto& pair = pairs[i];
        Pose2D predicted = transformVSLAMtoGLS(pair.vslam, calib, use_sensor_offset);
        
        double err_x = predicted.x - pair.gls.x;
        double err_y = predicted.y - pair.gls.y;
        double err_yaw = normalizeAngle(predicted.yaw_rad - pair.gls.yaw_rad);
        
        double weight = use_weights ? std::sqrt(pair.weight) : 1.0;
        
        r[i * 3 + 0] = weight * err_x;
        r[i * 3 + 1] = weight * err_y;
        r[i * 3 + 2] = weight * err_yaw * 0.1;
        
        // Common terms
        double cos_r = std::cos(calib.rotation);
        double sin_r = std::sin(calib.rotation);
        double vx = pair.vslam.x;
        double vy = pair.vslam.y;
        
        double robot_heading = predicted.yaw_rad;
        double cos_h = std::cos(robot_heading);
        double sin_h = std::sin(robot_heading);
        
        // Jacobian w.r.t. map_origin_x
        J[i * 3 + 0][0] = weight * 1.0;
        J[i * 3 + 1][0] = 0.0;
        J[i * 3 + 2][0] = 0.0;
        
        // Jacobian w.r.t. map_origin_y
        J[i * 3 + 0][1] = 0.0;
        J[i * 3 + 1][1] = weight * 1.0;
        J[i * 3 + 2][1] = 0.0;
        
        // Jacobian w.r.t. rotation
        double dx_dr = -sin_r * vx - cos_r * vy;
        double dy_dr = cos_r * vx - sin_r * vy;
        
        if (use_sensor_offset) {
            // Additional terms from sensor offset rotation
            dx_dr += -(-sin_h * calib.sensor_offset_forward - cos_h * calib.sensor_offset_lateral);
            dy_dr += -(cos_h * calib.sensor_offset_forward - sin_h * calib.sensor_offset_lateral);
        }
        
        J[i * 3 + 0][2] = weight * dx_dr;
        J[i * 3 + 1][2] = weight * dy_dr;
        J[i * 3 + 2][2] = weight * 0.1 * 1.0;
        
        // Jacobian w.r.t. yaw_offset
        if (use_sensor_offset) {
            J[i * 3 + 0][3] = weight * (-sin_h * calib.sensor_offset_forward - cos_h * calib.sensor_offset_lateral);
            J[i * 3 + 1][3] = weight * (cos_h * calib.sensor_offset_forward - sin_h * calib.sensor_offset_lateral);
        } else {
            J[i * 3 + 0][3] = 0.0;
            J[i * 3 + 1][3] = 0.0;
        }
        J[i * 3 + 2][3] = weight * 0.1 * 1.0;
        
        // Jacobian w.r.t. sensor_offset_forward (if used)
        if (use_sensor_offset) {
            J[i * 3 + 0][4] = weight * (-cos_h);
            J[i * 3 + 1][4] = weight * (-sin_h);
            J[i * 3 + 2][4] = 0.0;
            
            // Jacobian w.r.t. sensor_offset_lateral
            J[i * 3 + 0][5] = weight * sin_h;
            J[i * 3 + 1][5] = weight * (-cos_h);
            J[i * 3 + 2][5] = 0.0;
        }
    }
}

// Matrix operations
std::vector<std::vector<double>> matrixTranspose(const std::vector<std::vector<double>>& A) {
    if (A.empty()) return {};
    int rows = A.size();
    int cols = A[0].size();
    std::vector<std::vector<double>> AT(cols, std::vector<double>(rows));
    
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            AT[j][i] = A[i][j];
        }
    }
    return AT;
}

std::vector<std::vector<double>> matrixMultiply(const std::vector<std::vector<double>>& A,
                                                const std::vector<std::vector<double>>& B) {
    int m = A.size();
    int n = B[0].size();
    int k = A[0].size();
    
    std::vector<std::vector<double>> C(m, std::vector<double>(n, 0.0));
    
    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            for (int p = 0; p < k; ++p) {
                C[i][j] += A[i][p] * B[p][j];
            }
        }
    }
    return C;
}

std::vector<double> matrixVectorMultiply(const std::vector<std::vector<double>>& A,
                                        const std::vector<double>& v) {
    int m = A.size();
    std::vector<double> result(m, 0.0);
    
    for (int i = 0; i < m; ++i) {
        for (size_t j = 0; j < v.size(); ++j) {
            result[i] += A[i][j] * v[j];
        }
    }
    return result;
}

std::vector<double> solveLinearSystem(std::vector<std::vector<double>> A, std::vector<double> b) {
    int n = A.size();
    
    for (int i = 0; i < n; ++i) {
        A[i].push_back(b[i]);
    }
    
    for (int i = 0; i < n; ++i) {
        int max_row = i;
        for (int k = i + 1; k < n; ++k) {
            if (std::abs(A[k][i]) > std::abs(A[max_row][i])) {
                max_row = k;
            }
        }
        std::swap(A[i], A[max_row]);
        
        double pivot = A[i][i];
        if (std::abs(pivot) < 1e-10) {
            pivot = 1e-10;
        }
        
        for (int j = i; j <= n; ++j) {
            A[i][j] /= pivot;
        }
        
        for (int k = 0; k < n; ++k) {
            if (k != i) {
                double factor = A[k][i];
                for (int j = i; j <= n; ++j) {
                    A[k][j] -= factor * A[i][j];
                }
            }
        }
    }
    
    std::vector<double> x(n);
    for (int i = 0; i < n; ++i) {
        x[i] = A[i][n];
    }
    
    return x;
}

CalibrationParams optimizeLM(const std::vector<PosePair>& pairs,
                             CalibrationParams initial,
                             bool use_sensor_offset = true,
                             int max_iter = 100,
                             double tol = 1e-8,
                             bool verbose = true) {
    CalibrationParams params = initial;
    double lambda = 0.01;
    double cost = computeCost(pairs, params, use_sensor_offset, true);
    
    if (verbose) {
        std::cout << "\n" << (use_sensor_offset ? "WITH" : "WITHOUT") 
                  << " Sensor Offset Optimization:" << std::endl;
        std::cout << "Initial cost: " << cost << std::endl;
    }
    
    for (int iter = 0; iter < max_iter; ++iter) {
        std::vector<std::vector<double>> J;
        std::vector<double> r;
        computeJacobianAndResidual(pairs, params, J, r, use_sensor_offset, true);
        
        auto JT = matrixTranspose(J);
        auto JTJ = matrixMultiply(JT, J);
        auto JTr = matrixVectorMultiply(JT, r);
        
        int n_params = use_sensor_offset ? 6 : 4;
        for (int i = 0; i < n_params; ++i) {
            JTJ[i][i] += lambda * (1.0 + JTJ[i][i]);
        }
        
        for (auto& val : JTr) val = -val;
        std::vector<double> delta = solveLinearSystem(JTJ, JTr);
        
        CalibrationParams new_params = params;
        new_params.map_origin_x += delta[0];
        new_params.map_origin_y += delta[1];
        new_params.rotation += delta[2];
        new_params.yaw_offset += delta[3];
        
        if (use_sensor_offset) {
            new_params.sensor_offset_forward += delta[4];
            new_params.sensor_offset_lateral += delta[5];
        }
        
        double new_cost = computeCost(pairs, new_params, use_sensor_offset, true);
        
        if (new_cost < cost) {
            params = new_params;
            double improvement = cost - new_cost;
            cost = new_cost;
            lambda *= 0.1;
            
            if (verbose && (iter % 10 == 0 || improvement < tol)) {
                std::cout << "Iter " << std::setw(3) << iter 
                          << ": cost = " << cost 
                          << ", improvement = " << improvement << std::endl;
            }
            
            if (improvement < tol) {
                if (verbose) std::cout << "Converged at iteration " << iter << std::endl;
                break;
            }
        } else {
            lambda *= 10.0;
            if (lambda > 1e10) break;
        }
    }
    
    if (verbose) std::cout << "Final cost: " << cost << std::endl;
    return params;
}

CalibrationParams getInitialGuess(const std::vector<PosePair>& pairs) {
    CalibrationParams calib;
    
    double sum_yaw = 0.0;
    for (const auto& pair : pairs) {
        sum_yaw += normalizeAngle(pair.gls.yaw_rad - pair.vslam.yaw_rad);
    }
    calib.rotation = sum_yaw / pairs.size();
    calib.yaw_offset = 0.0;
    
    double vslam_cx = 0.0, vslam_cy = 0.0;
    double gls_cx = 0.0, gls_cy = 0.0;
    
    for (const auto& pair : pairs) {
        vslam_cx += pair.vslam.x;
        vslam_cy += pair.vslam.y;
        gls_cx += pair.gls.x;
        gls_cy += pair.gls.y;
    }
    
    vslam_cx /= pairs.size();
    vslam_cy /= pairs.size();
    gls_cx /= pairs.size();
    gls_cy /= pairs.size();
    
    double cos_r = std::cos(calib.rotation);
    double sin_r = std::sin(calib.rotation);
    double vslam_cx_rot = cos_r * vslam_cx - sin_r * vslam_cy;
    double vslam_cy_rot = sin_r * vslam_cx + cos_r * vslam_cy;
    
    calib.map_origin_x = gls_cx - vslam_cx_rot;
    calib.map_origin_y = gls_cy - vslam_cy_rot;
    
    // Initial sensor offset guess
    calib.sensor_offset_forward = 0.30;  // 30cm
    calib.sensor_offset_lateral = 0.06;  // 6cm
    
    return calib;
}

// RANSAC-based outlier detection
std::vector<bool> detectOutliersRANSAC(const std::vector<PosePair>& pairs,
                                       bool use_sensor_offset = true,
                                       int max_iterations = 500,
                                       double inlier_threshold = 0.010) { // 10mm
    int n = pairs.size();
    std::vector<bool> best_inliers(n, false);
    int best_inlier_count = 0;
    CalibrationParams best_model;
    
    std::cout << "\n=== RANSAC Outlier Detection ===" << std::endl;
    std::cout << "Max iterations: " << max_iterations << std::endl;
    std::cout << "Inlier threshold: " << inlier_threshold * 1000 << " mm" << std::endl;
    
    std::srand(std::time(nullptr));
    
    // Minimum samples needed (6 parameters if using sensor offset, 4 otherwise)
    int min_samples = use_sensor_offset ? 8 : 6;
    
    for (int iter = 0; iter < max_iterations; ++iter) {
        // Randomly select minimum samples
        std::vector<int> sample_indices;
        std::vector<bool> selected(n, false);
        
        while (sample_indices.size() < min_samples) {
            int idx = std::rand() % n;
            if (!selected[idx]) {
                sample_indices.push_back(idx);
                selected[idx] = true;
            }
        }
        
        // Create subset
        std::vector<PosePair> subset;
        for (int idx : sample_indices) {
            subset.push_back(pairs[idx]);
        }
        
        // Fit model on subset
        CalibrationParams initial = getInitialGuess(subset);
        CalibrationParams model;
        
        try {
            model = optimizeLM(subset, initial, use_sensor_offset, 50, 1e-8, false); // quiet mode
        } catch (...) {
            continue; // Skip if optimization fails
        }
        
        // Count inliers
        std::vector<bool> current_inliers(n, false);
        int inlier_count = 0;
        
        for (int i = 0; i < n; ++i) {
            Pose2D predicted = transformVSLAMtoGLS(pairs[i].vslam, model, use_sensor_offset);
            double err_x = predicted.x - pairs[i].gls.x;
            double err_y = predicted.y - pairs[i].gls.y;
            double error = std::sqrt(err_x * err_x + err_y * err_y);
            
            if (error < inlier_threshold) {
                current_inliers[i] = true;
                inlier_count++;
            }
        }
        
        // Update best model
        if (inlier_count > best_inlier_count) {
            best_inlier_count = inlier_count;
            best_inliers = current_inliers;
            best_model = model;
            
            if (iter % 100 == 0 || inlier_count > n * 0.9) {
                std::cout << "Iter " << iter << ": " << inlier_count << "/" << n << " inliers" << std::endl;
            }
        }
        
        // Early termination if very good
        if (inlier_count > n * 0.95) {
            std::cout << "Early termination at iter " << iter << std::endl;
            break;
        }
    }
    
    std::cout << "Final: " << best_inlier_count << "/" << n << " inliers" << std::endl;
    std::cout << "Outliers detected: " << (n - best_inlier_count) << std::endl;
    
    return best_inliers;
}

// Statistical outlier detection (MAD-based)
std::vector<bool> detectOutliersMAD(const std::vector<PosePair>& pairs,
                                    const CalibrationParams& calib,
                                    bool use_sensor_offset = true,
                                    double threshold_sigma = 3.0) {
    int n = pairs.size();
    std::vector<double> errors(n);
    
    // Compute errors
    for (int i = 0; i < n; ++i) {
        Pose2D predicted = transformVSLAMtoGLS(pairs[i].vslam, calib, use_sensor_offset);
        double err_x = predicted.x - pairs[i].gls.x;
        double err_y = predicted.y - pairs[i].gls.y;
        errors[i] = std::sqrt(err_x * err_x + err_y * err_y);
    }
    
    // Compute median
    std::vector<double> sorted_errors = errors;
    std::sort(sorted_errors.begin(), sorted_errors.end());
    double median = sorted_errors[n / 2];
    
    // Compute MAD (Median Absolute Deviation)
    std::vector<double> abs_deviations(n);
    for (int i = 0; i < n; ++i) {
        abs_deviations[i] = std::abs(errors[i] - median);
    }
    std::sort(abs_deviations.begin(), abs_deviations.end());
    double mad = abs_deviations[n / 2];
    
    // Mark outliers (using modified Z-score)
    // Modified Z-score = 0.6745 * (x - median) / MAD
    // Threshold typically 3.5 (equivalent to 3 sigma for normal distribution)
    double threshold = median + threshold_sigma * 1.4826 * mad;
    
    std::vector<bool> inliers(n);
    int outlier_count = 0;
    
    std::cout << "\n=== MAD-based Outlier Detection ===" << std::endl;
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Median error: " << median * 1000 << " mm" << std::endl;
    std::cout << "MAD: " << mad * 1000 << " mm" << std::endl;
    std::cout << "Threshold: " << threshold * 1000 << " mm (" << threshold_sigma << " sigma)" << std::endl;
    
    for (int i = 0; i < n; ++i) {
        inliers[i] = (errors[i] < threshold);
        if (!inliers[i]) {
            std::cout << "Sample " << (i+1) << ": error = " << errors[i] * 1000 
                      << " mm (OUTLIER)" << std::endl;
            outlier_count++;
        }
    }
    
    std::cout << "Outliers detected: " << outlier_count << "/" << n << std::endl;
    
    return inliers;
}

// CalibrationParams optimizeLM(const std::vector<PosePair>& pairs,
//                              CalibrationParams initial,
//                              bool use_sensor_offset = true,
//                              int max_iter = 100,
//                              double tol = 1e-8,
//                              bool verbose = true) {
//     CalibrationParams params = initial;
//     double lambda = 0.01;
//     double cost = computeCost(pairs, params, use_sensor_offset, true);
    
//     if (verbose) {
//         std::cout << "\n" << (use_sensor_offset ? "WITH" : "WITHOUT") 
//                   << " Sensor Offset Optimization:" << std::endl;
//         std::cout << "Initial cost: " << cost << std::endl;
//     }
    
//     for (int iter = 0; iter < max_iter; ++iter) {
//         std::vector<std::vector<double>> J;
//         std::vector<double> r;
//         computeJacobianAndResidual(pairs, params, J, r, use_sensor_offset, true);
        
//         auto JT = matrixTranspose(J);
//         auto JTJ = matrixMultiply(JT, J);
//         auto JTr = matrixVectorMultiply(JT, r);
        
//         int n_params = use_sensor_offset ? 6 : 4;
//         for (int i = 0; i < n_params; ++i) {
//             JTJ[i][i] += lambda * (1.0 + JTJ[i][i]);
//         }
        
//         for (auto& val : JTr) val = -val;
//         std::vector<double> delta = solveLinearSystem(JTJ, JTr);
        
//         CalibrationParams new_params = params;
//         new_params.map_origin_x += delta[0];
//         new_params.map_origin_y += delta[1];
//         new_params.rotation += delta[2];
//         new_params.yaw_offset += delta[3];
        
//         if (use_sensor_offset) {
//             new_params.sensor_offset_forward += delta[4];
//             new_params.sensor_offset_lateral += delta[5];
//         }
        
//         double new_cost = computeCost(pairs, new_params, use_sensor_offset, true);
        
//         if (new_cost < cost) {
//             params = new_params;
//             double improvement = cost - new_cost;
//             cost = new_cost;
//             lambda *= 0.1;
            
//             if (verbose && (iter % 10 == 0 || improvement < tol)) {
//                 std::cout << "Iter " << std::setw(3) << iter 
//                           << ": cost = " << cost 
//                           << ", improvement = " << improvement << std::endl;
//             }
            
//             if (improvement < tol) {
//                 if (verbose) std::cout << "Converged at iteration " << iter << std::endl;
//                 break;
//             }
//         } else {
//             lambda *= 10.0;
//             if (lambda > 1e10) break;
//         }
//     }
    
//     if (verbose) std::cout << "Final cost: " << cost << std::endl;
//     return params;
// }

// CalibrationParams getInitialGuess(const std::vector<PosePair>& pairs) {
//     CalibrationParams calib;
    
//     double sum_yaw = 0.0;
//     for (const auto& pair : pairs) {
//         sum_yaw += normalizeAngle(pair.gls.yaw_rad - pair.vslam.yaw_rad);
//     }
//     calib.rotation = sum_yaw / pairs.size();
//     calib.yaw_offset = 0.0;
    
//     double vslam_cx = 0.0, vslam_cy = 0.0;
//     double gls_cx = 0.0, gls_cy = 0.0;
    
//     for (const auto& pair : pairs) {
//         vslam_cx += pair.vslam.x;
//         vslam_cy += pair.vslam.y;
//         gls_cx += pair.gls.x;
//         gls_cy += pair.gls.y;
//     }
    
//     vslam_cx /= pairs.size();
//     vslam_cy /= pairs.size();
//     gls_cx /= pairs.size();
//     gls_cy /= pairs.size();
    
//     double cos_r = std::cos(calib.rotation);
//     double sin_r = std::sin(calib.rotation);
//     double vslam_cx_rot = cos_r * vslam_cx - sin_r * vslam_cy;
//     double vslam_cy_rot = sin_r * vslam_cx + cos_r * vslam_cy;
    
//     calib.map_origin_x = gls_cx - vslam_cx_rot;
//     calib.map_origin_y = gls_cy - vslam_cy_rot;
    
//     // Initial sensor offset guess
//     calib.sensor_offset_forward = 0.30;  // 30cm
//     calib.sensor_offset_lateral = 0.06;  // 6cm
    
//     return calib;
// }

void calculateErrors(const std::vector<PosePair>& pairs, const CalibrationParams& calib,
                    bool use_sensor_offset, const std::string& title,
                    const std::vector<bool>* inlier_mask = nullptr) {
    std::vector<double> x_errors, y_errors;
    std::vector<double> pos_errors;
    int count = 0;
    
    for (size_t i = 0; i < pairs.size(); ++i) {
        if (inlier_mask && !(*inlier_mask)[i]) continue;
        
        Pose2D predicted = transformVSLAMtoGLS(pairs[i].vslam, calib, use_sensor_offset);
        double err_x = predicted.x - pairs[i].gls.x;
        double err_y = predicted.y - pairs[i].gls.y;
        double err_pos = std::sqrt(err_x * err_x + err_y * err_y);
        
        x_errors.push_back(err_x);
        y_errors.push_back(err_y);
        pos_errors.push_back(err_pos);
        count++;
    }
    
    if (x_errors.empty()) {
        std::cout << "\n=== " << title << " ===" << std::endl;
        std::cout << "No data after filtering!" << std::endl;
        return;
    }
    
    double sum_x = 0, sum_y = 0, sum_x2 = 0, sum_y2 = 0;
    for (size_t i = 0; i < x_errors.size(); ++i) {
        sum_x += x_errors[i];
        sum_y += y_errors[i];
        sum_x2 += x_errors[i] * x_errors[i];
        sum_y2 += y_errors[i] * y_errors[i];
    }
    
    double mean_x = sum_x / count;
    double mean_y = sum_y / count;
    double std_x = std::sqrt(sum_x2 / count - mean_x * mean_x);
    double std_y = std::sqrt(sum_y2 / count - mean_y * mean_y);
    double pos_std = std::sqrt(std_x * std_x + std_y * std_y);
    
    // Max error
    double max_err = *std::max_element(pos_errors.begin(), pos_errors.end());
    
    std::cout << "\n=== " << title << " ===" << std::endl;
    std::cout << std::fixed << std::setprecision(3);
    if (inlier_mask) {
        std::cout << "Samples: " << count << " inliers / " << pairs.size() << " total" << std::endl;
    }
    std::cout << "X: Mean = " << mean_x * 1000 << " mm, Std = " << std_x * 1000 << " mm" << std::endl;
    std::cout << "Y: Mean = " << mean_y * 1000 << " mm, Std = " << std_y * 1000 << " mm" << std::endl;
    std::cout << "Position Std (2D): " << pos_std * 1000 << " mm" << std::endl;
    std::cout << "Max error: " << max_err * 1000 << " mm" << std::endl;
}

std::vector<PosePair> readCSV(const std::string& filename) {
    std::vector<PosePair> pairs;
    std::ifstream file(filename);
    
    if (!file.is_open()) return pairs;
    
    std::string line;
    std::getline(file, line);
    
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;
        PosePair pair;
        
        std::getline(ss, token, ','); pair.vslam.x = std::stod(token);
        std::getline(ss, token, ','); pair.vslam.y = std::stod(token);
        std::getline(ss, token, ','); double vslam_yaw_deg = std::stod(token);
        std::getline(ss, token, ','); pair.gls.x = std::stod(token);
        std::getline(ss, token, ','); pair.gls.y = std::stod(token);
        std::getline(ss, token, ','); double gls_yaw_deg = std::stod(token);
        
        std::getline(ss, token, ','); double vslam_std_x = std::stod(token);
        std::getline(ss, token, ','); double vslam_std_y = std::stod(token);
        std::getline(ss, token, ',');
        std::getline(ss, token, ','); double gls_std_x = std::stod(token);
        std::getline(ss, token, ','); double gls_std_y = std::stod(token);
        
        pair.vslam.yaw_rad = deg2rad(-vslam_yaw_deg);
        pair.gls.yaw_rad = deg2rad(gls_yaw_deg);
        
        double total_std = vslam_std_x + vslam_std_y + gls_std_x + gls_std_y;
        pair.weight = (total_std > 0.0001) ? 1.0 / (total_std * total_std) : 1.0;
        
        pairs.push_back(pair);
    }
    
    file.close();
    return pairs;
}

int main(int argc, char* argv[]) {
    std::string filename = "pose_log_20251029_125739_763.csv";
    if (argc > 1) filename = argv[1];
    
    std::cout << "╔═══════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  SENSOR OFFSET + OUTLIER REJECTION               ║" << std::endl;
    std::cout << "╚═══════════════════════════════════════════════════╝" << std::endl;
    
    std::vector<PosePair> pairs = readCSV(filename);
    if (pairs.empty()) return 1;
    
    std::cout << "\nLoaded " << pairs.size() << " samples" << std::endl;
    
    CalibrationParams initial = getInitialGuess(pairs);
    
    // ============================================================
    // Test 1: WITHOUT sensor offset (baseline)
    // ============================================================
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "TEST 1: WITHOUT Sensor Offset (Baseline)" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    CalibrationParams calib_no_offset = optimizeLM(pairs, initial, false, 100, 1e-9, true);
    calculateErrors(pairs, calib_no_offset, false, "Results WITHOUT Sensor Offset");
    
    // ============================================================
    // Test 2: WITH sensor offset (no outlier removal)
    // ============================================================
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "TEST 2: WITH Sensor Offset (All data)" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    CalibrationParams calib_with_offset = optimizeLM(pairs, initial, true, 100, 1e-9, true);
    
    std::cout << "\n=== Optimized Sensor Offset ===" << std::endl;
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Forward: " << calib_with_offset.sensor_offset_forward * 1000 << " mm" << std::endl;
    std::cout << "Lateral: " << calib_with_offset.sensor_offset_lateral * 1000 << " mm" << std::endl;
    
    calculateErrors(pairs, calib_with_offset, true, "Results WITH Sensor Offset (All data)");
    
    // ============================================================
    // Test 3: MAD-based outlier detection
    // ============================================================
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "TEST 3: WITH Sensor Offset + MAD Outlier Removal" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    std::vector<bool> inliers_mad = detectOutliersMAD(pairs, calib_with_offset, true, 2.5);
    
    // Create inlier subset
    std::vector<PosePair> pairs_mad_inliers;
    for (size_t i = 0; i < pairs.size(); ++i) {
        if (inliers_mad[i]) {
            pairs_mad_inliers.push_back(pairs[i]);
        }
    }
    
    // Re-optimize with inliers only
    CalibrationParams calib_mad = optimizeLM(pairs_mad_inliers, calib_with_offset, true, 100, 1e-9, true);
    
    std::cout << "\n=== Re-optimized Sensor Offset (MAD inliers) ===" << std::endl;
    std::cout << "Forward: " << calib_mad.sensor_offset_forward * 1000 << " mm" << std::endl;
    std::cout << "Lateral: " << calib_mad.sensor_offset_lateral * 1000 << " mm" << std::endl;
    
    calculateErrors(pairs, calib_mad, true, "Results WITH Sensor Offset + MAD", &inliers_mad);
    
    // ============================================================
    // Test 4: RANSAC-based outlier detection
    // ============================================================
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "TEST 4: WITH Sensor Offset + RANSAC Outlier Removal" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    
    std::vector<bool> inliers_ransac = detectOutliersRANSAC(pairs, true, 300, 0.008); // 8mm threshold
    
    // Create inlier subset
    std::vector<PosePair> pairs_ransac_inliers;
    for (size_t i = 0; i < pairs.size(); ++i) {
        if (inliers_ransac[i]) {
            pairs_ransac_inliers.push_back(pairs[i]);
        }
    }
    
    // Re-optimize with inliers only
    CalibrationParams calib_ransac = optimizeLM(pairs_ransac_inliers, calib_with_offset, true, 100, 1e-9, true);
    
    std::cout << "\n=== Re-optimized Sensor Offset (RANSAC inliers) ===" << std::endl;
    std::cout << "Forward: " << calib_ransac.sensor_offset_forward * 1000 << " mm" << std::endl;
    std::cout << "Lateral: " << calib_ransac.sensor_offset_lateral * 1000 << " mm" << std::endl;
    
    calculateErrors(pairs, calib_ransac, true, "Results WITH Sensor Offset + RANSAC", &inliers_ransac);
    
    // ============================================================
    // Summary
    // ============================================================
    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "SUMMARY" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    std::cout << "\nPosition Std comparison:" << std::endl;
    std::cout << "1. Baseline (no offset):        Compare with Test 1 output" << std::endl;
    std::cout << "2. With offset (all data):      Compare with Test 2 output" << std::endl;
    std::cout << "3. With offset + MAD:           Compare with Test 3 output" << std::endl;
    std::cout << "4. With offset + RANSAC:        Compare with Test 4 output" << std::endl;
    std::cout << "\nBest method is the one with lowest Position Std!" << std::endl;
    
    return 0;
}