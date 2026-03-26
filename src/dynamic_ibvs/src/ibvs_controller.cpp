#include "dynamic_ibvs/ibvs_controller.hpp"
#include <cmath>
#include <ros/ros.h>

// color definitions for console output
#define COLOR_GREEN "\033[1;32m"
#define COLOR_YELLOW "\033[1;33m"
#define COLOR_BLUE "\033[1;34m"
#define COLOR_PURPLE "\033[1;35m"
#define COLOR_RESET "\033[0m"

static const float EPSILON = 1e-6f;
static const int NUM_FEATURES = 4; // Square marker with 4 corners

IBVSController::IBVSController()
    : camera_initialized_(false), desired_area_(0), desired_centroid_x_(0),
      desired_centroid_y_(0), current_centroid_x_(0), current_centroid_y_(0),
      current_normalized_area_(0), error_norm_max_(1.0f),
      first_computation_(false) {
  // Initialize matrices
  jacobian_ = -1.0 * arma::diagmat(arma::ones<arma::vec>(4));
  desired_features_image_ = arma::zeros<arma::mat>(4, 2);
  desired_features_normalized_ = arma::zeros<arma::mat>(4, 2);
  centered_moments_desired_ = arma::zeros<arma::mat>(4, 4);
  centered_moments_current_ = arma::zeros<arma::mat>(4, 4);
  error_ = arma::zeros<arma::vec>(4);
  control_velocity_ = arma::zeros<arma::vec>(4);
}

void IBVSController::setGainLimits(float lambda_min, float lambda_max) {
  lambda_min_ = lambda_min;
  lambda_max_ = lambda_max;
  ROS_INFO("IBVS: Gain limits set - lambda_min: %.2f, lambda_max: %.2f",
           lambda_min_, lambda_max_);
  return;
}

void IBVSController::setActiveCamera(const CameraIntrinsics &intrinsics) {
  camera_ = intrinsics;
  camera_initialized_ = true;
  ROS_INFO(
      "IBVS: Camera intrinsics set - fx: %.2f, fy: %.2f, u0: %.2f, v0: %.2f",
      camera_.f_x, camera_.f_y, camera_.u_0, camera_.v_0);
}

void IBVSController::setDesiredFeatures(const VisualFeatures &desired_features,
                                        float depth) {
  if (!camera_initialized_) {
    ROS_WARN("IBVS: Camera not initialized. Cannot set desired features.");
    return;
  }

  if (desired_features.x.size() != NUM_FEATURES ||
      desired_features.y.size() != NUM_FEATURES) {
    ROS_ERROR("IBVS: Invalid number of features. Expected %d, got %zu",
              NUM_FEATURES, desired_features.x.size());
    return;
  }

  // Store desired features in image coordinates
  for (int i = 0; i < NUM_FEATURES; ++i) {
    desired_features_image_(i, 0) = desired_features.x[i];
    desired_features_image_(i, 1) = desired_features.y[i];
  }

  // Normalize to normalized image coordinates
  for (int i = 0; i < NUM_FEATURES; ++i) {
    float x_norm = (desired_features_image_(i, 0) - camera_.u_0) / camera_.f_x;
    float y_norm = (desired_features_image_(i, 1) - camera_.v_0) / camera_.f_y;
    desired_features_normalized_(i, 0) = x_norm;
    desired_features_normalized_(i, 1) = y_norm;
  }

  // Compute desired moments
  centered_moments_desired_ =
      computeCenteredMoments(desired_features_normalized_);

  // Extract centroid
  extractCentroid(desired_features_normalized_, desired_centroid_x_,
                  desired_centroid_y_);

  // Compute desired area
  desired_area_ =
      arma::accu(centered_moments_desired_(arma::span(2, 2), arma::span(0, 2)));

  ROS_INFO("IBVS: Desired features set - centroid: (%.3f, %.3f), area: %.3f",
           desired_centroid_x_, desired_centroid_y_, desired_area_);
}

IBVSController::ControlOutput
IBVSController::computeControlLaw(const VisualFeatures &current_features,
                                  float depth_estimate) {

  ControlOutput output = {0, 0, 0, 0, 0};

  if (!camera_initialized_) {
    ROS_WARN("IBVS: Camera not initialized");
    return output;
  }

  if (current_features.x.size() != NUM_FEATURES ||
      current_features.y.size() != NUM_FEATURES) {
    ROS_ERROR("IBVS: Invalid number of current features");
    return output;
  }

  // Convert to normalized image coordinates
  arma::mat current_normalized(NUM_FEATURES, 2);
  for (int i = 0; i < NUM_FEATURES; ++i) {
    float x_norm = (current_features.x[i] - camera_.u_0) / camera_.f_x;
    float y_norm = (current_features.y[i] - camera_.v_0) / camera_.f_y;
    current_normalized(i, 0) = x_norm;
    current_normalized(i, 1) = y_norm;
  }

  // Compute current moments
  centered_moments_current_ = computeCenteredMoments(current_normalized);

  // Extract centroid
  extractCentroid(current_normalized, current_centroid_x_, current_centroid_y_);

  // Compute current area
  float current_area =
      arma::accu(centered_moments_current_(arma::span(2, 2), arma::span(0, 2)));
  current_normalized_area_ =
      depth_estimate *
      std::sqrt(desired_area_ / std::max(current_area, EPSILON));

  // Compute Jacobian
  computeJacobian(current_area, desired_area_, current_centroid_x_,
                  current_centroid_y_, depth_estimate);

  // Compute error
  computeError(current_normalized, depth_estimate);

  // Compute error norm
  output.error_norm = arma::norm(error_, 2);

  // Compute adaptive gain
  if (!first_computation_) {
    error_norm_max_ = output.error_norm;
    first_computation_ = true;
  }
  float lambda = computeAdaptiveGain(output.error_norm);

  // Compute pseudo-inverse of Jacobian
  jacobian_pinv_ = arma::pinv(jacobian_);

  // Compute control velocity: v = -lambda * J^+ * e
  control_velocity_ = -lambda * jacobian_pinv_ * error_;

  // Extract control outputs
  output.v_x = control_velocity_(0);
  output.v_y = control_velocity_(1);
  output.v_z = control_velocity_(2);
  output.omega_yaw = control_velocity_(3);

  ROS_INFO_THROTTLE(
      1.0,
      COLOR_YELLOW
      "IBVS: Error norm: %.4f, Lambda: %.3f, Velocity: [%.3f, %.3f, "
      "%.3f, %.3f]" COLOR_RESET,
      output.error_norm, lambda, output.v_x, output.v_y, output.v_z,
      output.omega_yaw);

  return output;
}

arma::mat IBVSController::computeCenteredMoments(const arma::mat &features) {
  arma::mat centered_moments = arma::zeros<arma::mat>(4, 4);

  // Compute centroid
  float centroid_x = 0, centroid_y = 0;
  extractCentroid(features, centroid_x, centroid_y);

  // Center the features
  arma::mat centered_features = arma::zeros<arma::mat>(NUM_FEATURES, 2);
  for (int i = 0; i < NUM_FEATURES; ++i) {
    centered_features(i, 0) = features(i, 0) - centroid_x;
    centered_features(i, 1) = features(i, 1) - centroid_y;
  }

  // Compute moments m_ij = sum(x^i * y^j)
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      double moment_sum = 0;
      for (int k = 0; k < NUM_FEATURES; ++k) {
        moment_sum += std::pow(centered_features(k, 0), i) *
                      std::pow(centered_features(k, 1), j);
      }
      centered_moments(i, j) = moment_sum;
    }
  }

  return centered_moments;
}

void IBVSController::extractCentroid(const arma::mat &features,
                                     float &centroid_x, float &centroid_y) {
  centroid_x = 0;
  centroid_y = 0;
  for (int i = 0; i < NUM_FEATURES; ++i) {
    centroid_x += features(i, 0);
    centroid_y += features(i, 1);
  }
  centroid_x /= NUM_FEATURES;
  centroid_y /= NUM_FEATURES;
}

void IBVSController::computeJacobian(float current_area, float desired_area,
                                     float current_centroid_x,
                                     float current_centroid_y, float depth) {
  jacobian_ = -1.0 * arma::diagmat(arma::ones<arma::vec>(4));

  // Normalized area
  float area_normalized =
      depth * std::sqrt(desired_area / std::max(current_area, EPSILON));

  // Jacobian entries for rotation
  jacobian_(0, 3) = current_centroid_y;  // x error vs rotation
  jacobian_(1, 3) = -current_centroid_x; // y error vs rotation

  // For combined moments
  float area_avg = (current_area + desired_area) / (2.0f * NUM_FEATURES);
  jacobian_(2, 3) = area_normalized * current_centroid_x;
  jacobian_(3, 3) = -area_normalized * current_centroid_x;
}

void IBVSController::computeError(const arma::mat &current_features,
                                  float depth) {
  error_.zeros();

  // Centroid error
  error_(0) = current_normalized_area_ * current_centroid_x_ -
              (depth * desired_centroid_x_);
  error_(1) = current_normalized_area_ * current_centroid_y_ -
              (depth * desired_centroid_y_);

  // Area error
  float current_area =
      arma::accu(centered_moments_current_(arma::span(2, 2), arma::span(0, 2)));
  error_(2) = current_normalized_area_ - depth;

  // Orientation error (using second-order moments)
  float u20 = centered_moments_current_(2, 0) / NUM_FEATURES;
  float u02 = centered_moments_current_(0, 2) / NUM_FEATURES;
  float u20_d = centered_moments_desired_(2, 0) / NUM_FEATURES;
  float u02_d = centered_moments_desired_(0, 2) / NUM_FEATURES;

  float alpha = 0.5f * std::atan2(2.0f * u20, u02 - u20);
  float alpha_d = 0.5f * std::atan2(2.0f * u20_d, u02_d - u20_d);

  error_(3) = alpha - alpha_d;
}

float IBVSController::computeAdaptiveGain(float current_error_norm) {

  float error_ratio =
      std::min(current_error_norm / std::max(error_norm_max_, EPSILON), 1.0f);
  float lambda = lambda_max_ - (lambda_max_ - lambda_min_) * error_ratio;

  return lambda;
}

void IBVSController::reset() {
  first_computation_ = false;
  error_norm_max_ = 1.0f;
  error_.zeros();
  control_velocity_.zeros();
  ROS_INFO("IBVS: Controller reset");
}
