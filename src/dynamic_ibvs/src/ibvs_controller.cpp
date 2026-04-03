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
      first_computation_(false), kd_gain_(0.0f), has_prev_error_(false) {
  // Initialize matrices
  jacobian_ = -1.0 * arma::diagmat(arma::ones<arma::vec>(4));
  desired_features_image_ = arma::zeros<arma::mat>(4, 2);
  desired_features_normalized_ = arma::zeros<arma::mat>(4, 2);
  centered_moments_desired_ = arma::zeros<arma::mat>(4, 4);
  centered_moments_current_ = arma::zeros<arma::mat>(4, 4);
  error_ = arma::zeros<arma::vec>(4);
  control_velocity_ = arma::zeros<arma::vec>(4);
  prev_error_ = arma::zeros<arma::vec>(4);
  error_derivative_ = arma::zeros<arma::vec>(4);
}

void IBVSController::setGainLimits(float lambda_min, float lambda_max,
                                   float kd_gain) {
  lambda_min_ = lambda_min;
  lambda_max_ = lambda_max;
  kd_gain_ = kd_gain;
  ROS_INFO("IBVS: Gain limits set - lambda_min: %.2f, lambda_max: %.2f, "
           "kd: %.3f",
           lambda_min_, lambda_max_, kd_gain_);
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

  // FIX 1: Reset controller state on new desired features (e.g. ID 999->0
  // handoff). Without this, the derivative term fires immediately with a
  // stale prev_error_ from the old marker, amplifying the transition spike.
  reset();
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

  // ── Derivative term (PD control) ──────────────────────────────────────
  arma::vec derivative_contribution = arma::zeros<arma::vec>(4);

  if (kd_gain_ > EPSILON && has_prev_error_) {
    float dt = (ros::Time::now() - prev_error_stamp_).toSec();
    if (dt > 1e-4f && dt < 1.0f) {
      arma::vec raw_derivative = (error_ - prev_error_) / dt;

      // Exponential low-pass filter: ė_filtered = α·ė_raw + (1-α)·ė_prev
      error_derivative_ = DERIV_FILTER_ALPHA * raw_derivative +
                          (1.0f - DERIV_FILTER_ALPHA) * error_derivative_;

      derivative_contribution = -kd_gain_ * jacobian_pinv_ * error_derivative_;
    }
  }

  // Store current error for next iteration
  prev_error_ = error_;
  prev_error_stamp_ = ros::Time::now();
  has_prev_error_ = true;

  // Compute control velocity: v = -λ * J⁺ * e  -  kd * J⁺ * ė
  control_velocity_ =
      -lambda * jacobian_pinv_ * error_ + derivative_contribution;

  // Extract control outputs
  output.v_x = control_velocity_(0);
  output.v_y = control_velocity_(1);
  output.v_z = control_velocity_(2);
  output.omega_yaw = control_velocity_(3);

  ROS_INFO_THROTTLE(
      1.0,
      COLOR_YELLOW "IBVS: err=%.4f λ=%.3f kd=%.3f ė_norm=%.3f vel=[%.3f, %.3f, "
                   "%.3f, %.3f]" COLOR_RESET,
      output.error_norm, lambda, kd_gain_, arma::norm(error_derivative_, 2),
      output.v_x, output.v_y, output.v_z, output.omega_yaw);

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

  // Jacobian entries for rotation (yaw coupling into x/y centroid error)
  jacobian_(0, 3) = current_centroid_y;  // x error vs yaw
  jacobian_(1, 3) = -current_centroid_x; // y error vs yaw

  // FIX 2: area_avg was computed but never used; Jacobian(3,3) was a copy of
  // Jacobian(2,3) with wrong sign. Row 2 = area error (depth control),
  // Row 3 = orientation error (yaw control). They should be independent.
  // area_normalized scales the area row; orientation row stays -1 (identity).
  jacobian_(2, 2) = -area_normalized; // area error drives Z velocity
  // jacobian_(3,3) already set to -1 by diagmat init — correct for yaw
}

void IBVSController::computeError(const arma::mat &current_features,
                                  float depth) {
  error_.zeros();

  // Centroid error (scaled by normalized area to be depth-invariant)
  error_(0) = current_normalized_area_ * current_centroid_x_ -
              (depth * desired_centroid_x_);
  error_(1) = current_normalized_area_ * current_centroid_y_ -
              (depth * desired_centroid_y_);

  // FIX 3: Area error should be dimensionless ratio, not depth-dependent.
  // Old: error_(2) = current_normalized_area_ - depth
  //   → only zero when depth==0, never converges correctly.
  // New: error_(2) = sqrt(Ad/Ac) - 1
  //   → zero exactly when current area == desired area, independent of depth.
  float current_area =
      arma::accu(centered_moments_current_(arma::span(2, 2), arma::span(0, 2)));
  float area_ratio = std::sqrt(desired_area_ / std::max(current_area, EPSILON));
  error_(2) = area_ratio - 1.0f;

  // FIX 4: Orientation error — standard formula uses the cross-moment u11,
  // not u20 in the atan2 numerator. Old code used u20 twice, giving a
  // wrong angle that introduced systematic yaw error and caused x/y
  // oscillations via Jacobian cross-coupling.
  //
  // Correct formula: α = 0.5 * atan2(2*μ₁₁, μ₂₀ - μ₀₂)
  float u11 = centered_moments_current_(1, 1) / NUM_FEATURES; // cross-moment
  float u20 = centered_moments_current_(2, 0) / NUM_FEATURES;
  float u02 = centered_moments_current_(0, 2) / NUM_FEATURES;

  float u11_d = centered_moments_desired_(1, 1) / NUM_FEATURES;
  float u20_d = centered_moments_desired_(2, 0) / NUM_FEATURES;
  float u02_d = centered_moments_desired_(0, 2) / NUM_FEATURES;

  float alpha   = 0.5f * std::atan2(2.0f * u11,   u20   - u02);
  float alpha_d = 0.5f * std::atan2(2.0f * u11_d, u20_d - u02_d);

  error_(3) = alpha - alpha_d;
}

float IBVSController::computeAdaptiveGain(float current_error_norm) {
  // FIX 5: Gain was inverted — λ_max was given when error≈0, λ_min when
  // error was large. This is the opposite of desired behaviour (you want
  // aggressive correction when far away, gentle when close).
  // Corrected: λ = λ_min + (λ_max - λ_min) * error_ratio
  //   → λ_max when error is large, λ_min when error≈0.
  float error_ratio =
      std::min(current_error_norm / std::max(error_norm_max_, EPSILON), 1.0f);
  float lambda = lambda_min_ + (lambda_max_ - lambda_min_) * error_ratio;

  return lambda;
}

void IBVSController::reset() {
  first_computation_ = false;
  error_norm_max_ = 1.0f;
  error_.zeros();
  control_velocity_.zeros();
  prev_error_.zeros();
  error_derivative_.zeros();
  has_prev_error_ = false;
  ROS_INFO("IBVS: Controller reset");
}
