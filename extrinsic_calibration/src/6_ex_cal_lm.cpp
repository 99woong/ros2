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
   double weight;  // For weighted optimization
};


struct CalibrationParams {
   double translation_x;
   double translation_y;
   double rotation;
   double yaw_offset;
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


Pose2D transformVSLAMtoGLS(const Pose2D& vslam_pose, const CalibrationParams& calib) {
   Pose2D gls_pose;
  
   double cos_r = std::cos(calib.rotation);
   double sin_r = std::sin(calib.rotation);
  
   double x_rotated = cos_r * vslam_pose.x - sin_r * vslam_pose.y;
   double y_rotated = sin_r * vslam_pose.x + cos_r * vslam_pose.y;
  
   gls_pose.x = x_rotated + calib.translation_x;
   gls_pose.y = y_rotated + calib.translation_y;
   gls_pose.yaw_rad = normalizeAngle(vslam_pose.yaw_rad + calib.rotation + calib.yaw_offset);
  
   return gls_pose;
}


// Compute cost (sum of squared errors)
double computeCost(const std::vector<PosePair>& pairs, const CalibrationParams& calib,
                  bool use_weights = true) {
   double cost = 0.0;
  
   for (const auto& pair : pairs) {
       Pose2D predicted = transformVSLAMtoGLS(pair.vslam, calib);
      
       double err_x = predicted.x - pair.gls.x;
       double err_y = predicted.y - pair.gls.y;
       double err_yaw = normalizeAngle(predicted.yaw_rad - pair.gls.yaw_rad);
      
       double weight = use_weights ? pair.weight : 1.0;
      
       // Weighted sum of squared errors
       cost += weight * (err_x * err_x + err_y * err_y + 0.01 * err_yaw * err_yaw);
   }
  
   return cost;
}


// Compute Jacobian matrix and residual vector
void computeJacobianAndResidual(const std::vector<PosePair>& pairs,
                               const CalibrationParams& calib,
                               std::vector<std::vector<double>>& J,
                               std::vector<double>& r,
                               bool use_weights = true) {
   int n_residuals = pairs.size() * 3;  // x, y, yaw for each pair
   int n_params = 4;  // tx, ty, rotation, yaw_offset
  
   J.assign(n_residuals, std::vector<double>(n_params, 0.0));
   r.assign(n_residuals, 0.0);
  
   for (size_t i = 0; i < pairs.size(); ++i) {
       const auto& pair = pairs[i];
       Pose2D predicted = transformVSLAMtoGLS(pair.vslam, calib);
      
       double err_x = predicted.x - pair.gls.x;
       double err_y = predicted.y - pair.gls.y;
       double err_yaw = normalizeAngle(predicted.yaw_rad - pair.gls.yaw_rad);
      
       double weight = use_weights ? std::sqrt(pair.weight) : 1.0;
      
       // Residuals
       r[i * 3 + 0] = weight * err_x;
       r[i * 3 + 1] = weight * err_y;
       r[i * 3 + 2] = weight * err_yaw * 0.1;  // Scale yaw error
      
       // Jacobian computation
       double cos_r = std::cos(calib.rotation);
       double sin_r = std::sin(calib.rotation);
       double vx = pair.vslam.x;
       double vy = pair.vslam.y;
      
       // Derivatives w.r.t. translation_x
       J[i * 3 + 0][0] = weight * 1.0;
       J[i * 3 + 1][0] = 0.0;
       J[i * 3 + 2][0] = 0.0;
      
       // Derivatives w.r.t. translation_y
       J[i * 3 + 0][1] = 0.0;
       J[i * 3 + 1][1] = weight * 1.0;
       J[i * 3 + 2][1] = 0.0;
      
       // Derivatives w.r.t. rotation
       J[i * 3 + 0][2] = weight * (-sin_r * vx - cos_r * vy);
       J[i * 3 + 1][2] = weight * (cos_r * vx - sin_r * vy);
       J[i * 3 + 2][2] = weight * 0.1 * 1.0;
      
       // Derivatives w.r.t. yaw_offset
       J[i * 3 + 0][3] = 0.0;
       J[i * 3 + 1][3] = 0.0;
       J[i * 3 + 2][3] = weight * 0.1 * 1.0;
   }
}


// Matrix operations (simple implementations)
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


// Solve linear system using Gauss-Jordan elimination
std::vector<double> solveLinearSystem(std::vector<std::vector<double>> A, std::vector<double> b) {
   int n = A.size();
  
   // Augment matrix
   for (int i = 0; i < n; ++i) {
       A[i].push_back(b[i]);
   }
  
   // Forward elimination
   for (int i = 0; i < n; ++i) {
       // Find pivot
       int max_row = i;
       for (int k = i + 1; k < n; ++k) {
           if (std::abs(A[k][i]) > std::abs(A[max_row][i])) {
               max_row = k;
           }
       }
       std::swap(A[i], A[max_row]);
      
       // Make diagonal 1
       double pivot = A[i][i];
       if (std::abs(pivot) < 1e-10) {
           pivot = 1e-10;  // Avoid division by zero
       }
      
       for (int j = i; j <= n; ++j) {
           A[i][j] /= pivot;
       }
      
       // Eliminate column
       for (int k = 0; k < n; ++k) {
           if (k != i) {
               double factor = A[k][i];
               for (int j = i; j <= n; ++j) {
                   A[k][j] -= factor * A[i][j];
               }
           }
       }
   }
  
   // Extract solution
   std::vector<double> x(n);
   for (int i = 0; i < n; ++i) {
       x[i] = A[i][n];
   }
  
   return x;
}


// Levenberg-Marquardt optimization
CalibrationParams optimizeLevenbergMarquardt(const std::vector<PosePair>& pairs,
                                           CalibrationParams initial_params,
                                           int max_iterations = 100,
                                           double tolerance = 1e-8,
                                           bool use_weights = true) {
   CalibrationParams params = initial_params;
   double lambda = 0.01;  // Damping parameter
   double cost = computeCost(pairs, params, use_weights);
  
   std::cout << "\nLevenberg-Marquardt Optimization:" << std::endl;
   std::cout << "Initial cost: " << cost << std::endl;
  
   for (int iter = 0; iter < max_iterations; ++iter) {
       // Compute Jacobian and residual
       std::vector<std::vector<double>> J;
       std::vector<double> r;
       computeJacobianAndResidual(pairs, params, J, r, use_weights);
      
       // Compute J^T * J and J^T * r
       auto JT = matrixTranspose(J);
       auto JTJ = matrixMultiply(JT, J);
       auto JTr = matrixVectorMultiply(JT, r);
      
       // Add damping (J^T * J + lambda * I)
       for (int i = 0; i < 4; ++i) {
           JTJ[i][i] += lambda * (1.0 + JTJ[i][i]);
       }
      
       // Solve (J^T * J + lambda * I) * delta = -J^T * r
       for (auto& val : JTr) val = -val;
       std::vector<double> delta = solveLinearSystem(JTJ, JTr);
      
       // Update parameters
       CalibrationParams new_params = params;
       new_params.translation_x += delta[0];
       new_params.translation_y += delta[1];
       new_params.rotation += delta[2];
       new_params.yaw_offset += delta[3];
      
       // Compute new cost
       double new_cost = computeCost(pairs, new_params, use_weights);
      
       // Accept or reject update
       if (new_cost < cost) {
           // Accept update
           params = new_params;
           double improvement = cost - new_cost;
           cost = new_cost;
           lambda *= 0.1;  // Decrease damping
          
           if (iter % 10 == 0 || improvement < tolerance) {
               std::cout << "Iter " << std::setw(3) << iter
                         << ": cost = " << std::setw(12) << cost
                         << ", improvement = " << improvement
                         << ", lambda = " << lambda << std::endl;
           }
          
           if (improvement < tolerance) {
               std::cout << "Converged at iteration " << iter << std::endl;
               break;
           }
       } else {
           // Reject update, increase damping
           lambda *= 10.0;
           if (lambda > 1e10) {
               std::cout << "Lambda too large, stopping" << std::endl;
               break;
           }
       }
   }
  
   std::cout << "Final cost: " << cost << std::endl;
   return params;
}


// Initial guess using closed-form solution
CalibrationParams getInitialGuess(const std::vector<PosePair>& pairs) {
   CalibrationParams calib;
  
   // Yaw offset
   double sum_yaw = 0.0;
   for (const auto& pair : pairs) {
       sum_yaw += normalizeAngle(pair.gls.yaw_rad - pair.vslam.yaw_rad);
   }
   calib.rotation = sum_yaw / pairs.size();
   calib.yaw_offset = 0.0;
  
   // Translation
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
  
   calib.translation_x = gls_cx - vslam_cx_rot;
   calib.translation_y = gls_cy - vslam_cy_rot;
  
   return calib;
}


void calculateDetailedErrors(const std::vector<PosePair>& pairs, const CalibrationParams& calib) {
   std::vector<double> x_errors, y_errors, yaw_errors;
  
   std::cout << "\n=== Error Analysis (First 20 samples) ===" << std::endl;
   std::cout << std::fixed << std::setprecision(3);
   std::cout << std::setw(6) << "ID" << " | "
             << std::setw(10) << "X_err(mm)" << " | "
             << std::setw(10) << "Y_err(mm)" << " | "
             << std::setw(10) << "Pos(mm)" << " | "
             << std::setw(10) << "Yaw(deg)" << std::endl;
   std::cout << std::string(60, '-') << std::endl;
  
   for (size_t i = 0; i < pairs.size(); ++i) 
//    for (size_t i = 0; i < 40; ++i) 
   {
       Pose2D predicted = transformVSLAMtoGLS(pairs[i].vslam, calib);
      
       double err_x = predicted.x - pairs[i].gls.x;
       double err_y = predicted.y - pairs[i].gls.y;
       double err_pos = std::sqrt(err_x * err_x + err_y * err_y);
       double err_yaw = normalizeAngle(predicted.yaw_rad - pairs[i].gls.yaw_rad);
      
       x_errors.push_back(err_x);
       y_errors.push_back(err_y);
       yaw_errors.push_back(err_yaw);
      
    //    if (i < 20) {
           std::cout << std::setw(6) << i + 1 << " | "
                     << std::setw(10) << err_x * 1000 << " | "
                     << std::setw(10) << err_y * 1000 << " | "
                     << std::setw(10) << err_pos * 1000 << " | "
                     << std::setw(10) << rad2deg(err_yaw) << std::endl;
    //    }
   }
  
   // Statistics
   double sum_x = 0, sum_y = 0, sum_yaw = 0;
   double sum_x2 = 0, sum_y2 = 0, sum_yaw2 = 0;
  
   for (size_t i = 0; i < x_errors.size(); ++i) {
       sum_x += x_errors[i];
       sum_y += y_errors[i];
       sum_yaw += yaw_errors[i];
       sum_x2 += x_errors[i] * x_errors[i];
       sum_y2 += y_errors[i] * y_errors[i];
       sum_yaw2 += yaw_errors[i] * yaw_errors[i];
   }
  
   double mean_x = sum_x / x_errors.size();
   double mean_y = sum_y / x_errors.size();
   double mean_yaw = sum_yaw / x_errors.size();
  
   double std_x = std::sqrt(sum_x2 / x_errors.size() - mean_x * mean_x);
   double std_y = std::sqrt(sum_y2 / x_errors.size() - mean_y * mean_y);
   double std_yaw = std::sqrt(sum_yaw2 / x_errors.size() - mean_yaw * mean_yaw);
  
   double rms_x = std::sqrt(sum_x2 / x_errors.size());
   double rms_y = std::sqrt(sum_y2 / x_errors.size());
   double rms_yaw = std::sqrt(sum_yaw2 / x_errors.size());
  
   std::cout << "\n=== Statistics ===" << std::endl;
   std::cout << "X:   Mean = " << mean_x * 1000 << " mm, Std = " << std_x * 1000 << " mm, RMS = " << rms_x * 1000 << " mm" << std::endl;
   std::cout << "Y:   Mean = " << mean_y * 1000 << " mm, Std = " << std_y * 1000 << " mm, RMS = " << rms_y * 1000 << " mm" << std::endl;
   std::cout << "Yaw: Mean = " << rad2deg(mean_yaw) << " deg, Std = " << rad2deg(std_yaw) << " deg, RMS = " << rad2deg(rms_yaw) << " deg" << std::endl;
   std::cout << "Position RMS (2D): " << std::sqrt(rms_x * rms_x + rms_y * rms_y) * 1000 << " mm" << std::endl;
   std::cout << "Position Std (2D): " << std::sqrt(std_x * std_x + std_y * std_y) * 1000 << " mm" << std::endl;
  
   double pos_std = std::sqrt(std_x * std_x + std_y * std_y) * 1000;
  
   std::cout << "\n=== VSLAM Performance ===" << std::endl;
   if (pos_std < 10.0) {
       std::cout << "★★★★★ EXCELLENT - Sub-centimeter!" << std::endl;
   } else if (pos_std < 20.0) {
       std::cout << "★★★★☆ VERY GOOD" << std::endl;
   } else if (pos_std < 30.0) {
       std::cout << "★★★☆☆ GOOD" << std::endl;
   } else {
       std::cout << "★★☆☆☆ FAIR" << std::endl;
   }
}


std::vector<PosePair> readCSV(const std::string& filename) 
{
   std::vector<PosePair> pairs;
   std::ifstream file(filename);
  
   if (!file.is_open()) 
   {
       std::cerr << "Error: Cannot open file " << filename << std::endl;
       return pairs;
   }
  
   std::string line;
   std::getline(file, line);
  
   while (std::getline(file, line)) 
   {
       std::stringstream ss(line);
       std::string token;
       PosePair pair;
      
       std::getline(ss, token, ','); pair.vslam.x = std::stod(token);
       std::getline(ss, token, ','); pair.vslam.y = std::stod(token);
       std::getline(ss, token, ','); double vslam_yaw_deg = std::stod(token);
       std::getline(ss, token, ','); pair.gls.x = std::stod(token);
       std::getline(ss, token, ','); pair.gls.y = std::stod(token);
       std::getline(ss, token, ','); double gls_yaw_deg = std::stod(token);
      
       // Read stddev for weighting
       std::getline(ss, token, ','); double vslam_std_x = std::stod(token);
       std::getline(ss, token, ','); double vslam_std_y = std::stod(token);
       std::getline(ss, token, ','); // skip vslam_std_yaw
       std::getline(ss, token, ','); double gls_std_x = std::stod(token);
       std::getline(ss, token, ','); double gls_std_y = std::stod(token);
      
      
       pair.vslam.yaw_rad = deg2rad(-vslam_yaw_deg);
       pair.gls.yaw_rad = deg2rad(gls_yaw_deg);
      
       // Compute weight (inverse of uncertainty)
       double total_std = vslam_std_x + vslam_std_y + gls_std_x + gls_std_y;
       pair.weight = (total_std > 0.0001) ? 1.0 / (total_std * total_std) : 1.0;
      
       pairs.push_back(pair);
   }
  
   file.close();
   return pairs;
}




int main(int argc, char* argv[]) 
{
   std::string filename = "pose_log_20251024_143111_930.csv";
  
   if (argc > 1) 
   {
       filename = argv[1];
   }
  
   std::cout << "╔═══════════════════════════════════════════════════╗" << std::endl;
   std::cout << "║  GLS-VSLAM NONLINEAR OPTIMIZATION CALIBRATION     ║" << std::endl;
   std::cout << "╚═══════════════════════════════════════════════════╝" << std::endl;
  
   std::vector<PosePair> pairs = readCSV(filename);
  
   if (pairs.empty()) 
   {
       std::cerr << "Error: No data" << std::endl;
       return 1;
   }
  
   std::cout << "\nLoaded " << pairs.size() << " samples" << std::endl;
  
   // Get initial guess
   CalibrationParams initial = getInitialGuess(pairs);
  
   std::cout << "\n=== Initial Guess (Closed-form) ===" << std::endl;
   std::cout << std::fixed << std::setprecision(6);
   std::cout << "Translation: (" << initial.translation_x << ", " << initial.translation_y << ")" << std::endl;
   std::cout << "Rotation: " << rad2deg(initial.rotation) << " degrees" << std::endl;
   std::cout << "Yaw Offset: " << rad2deg(initial.yaw_offset) << " degrees" << std::endl;
  
   double initial_cost = computeCost(pairs, initial, false);
   std::cout << "Initial RMS: " << std::sqrt(initial_cost / (pairs.size() * 2)) * 1000 << " mm" << std::endl;
  
   // Optimize with Levenberg-Marquardt
   CalibrationParams optimized = optimizeLevenbergMarquardt(pairs, initial, 100, 1e-9, true);
  
   std::cout << "\n=== Optimized Parameters ===" << std::endl;
   std::cout << "Translation: (" << optimized.translation_x << ", " << optimized.translation_y << ")" << std::endl;
   std::cout << "Rotation: " << rad2deg(optimized.rotation) << " degrees" << std::endl;
   std::cout << "Yaw Offset: " << rad2deg(optimized.yaw_offset) << " degrees" << std::endl;
  
   // Calculate errors
   calculateDetailedErrors(pairs, optimized);
  
   return 0;
}
