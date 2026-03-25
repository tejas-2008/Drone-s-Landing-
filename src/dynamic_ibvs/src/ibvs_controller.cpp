#include "dynamic_ibvs/ibvs_controller.hpp"
#include <cmath>
#include <ros/ros.h>

#define COLOR_YELLOW "\033[1;33m"
#define COLOR_RESET  "\033[0m"

static const float EPSILON      = 1e-6f;
static const int   NUM_FEATURES = 4;

// ─────────────────────────────────────────────────────────────────────────────
IBVSController::IBVSController()
    : camera_initialized_(false), desired_area_(0),
      desired_centroid_x_(0), desired_centroid_y_(0),
      current_centroid_x_(0), current_centroid_y_(0),
      current_normalized_area_(0), error_norm_max_(1.0f),
      first_computation_(false)
{
    jacobian_                 = -1.0 * arma::diagmat(arma::ones<arma::vec>(4));
    desired_features_image_   = arma::zeros<arma::mat>(4, 2);
    desired_features_normalized_ = arma::zeros<arma::mat>(4, 2);
    centered_moments_desired_ = arma::zeros<arma::mat>(4, 4);
    centered_moments_current_ = arma::zeros<arma::mat>(4, 4);
    error_           = arma::zeros<arma::vec>(4);
    control_velocity_= arma::zeros<arma::vec>(4);
}

// ─────────────────────────────────────────────────────────────────────────────
void IBVSController::setActiveCamera(const CameraIntrinsics &intrinsics)
{
    camera_ = intrinsics;
    camera_initialized_ = true;
    ROS_INFO("IBVS: Camera set — fx:%.2f fy:%.2f u0:%.2f v0:%.2f",
             camera_.f_x, camera_.f_y, camera_.u_0, camera_.v_0);
}

// ─────────────────────────────────────────────────────────────────────────────
void IBVSController::setDesiredFeatures(const VisualFeatures &desired_features,
                                        float depth)
{
    if (!camera_initialized_)
    {
        ROS_WARN("IBVS: Camera not initialised. Cannot set desired features.");
        return;
    }
    if ((int)desired_features.x.size() != NUM_FEATURES ||
        (int)desired_features.y.size() != NUM_FEATURES)
    {
        ROS_ERROR("IBVS: Expected %d features, got %zu",
                  NUM_FEATURES, desired_features.x.size());
        return;
    }

    for (int i = 0; i < NUM_FEATURES; ++i)
    {
        desired_features_image_(i, 0) = desired_features.x[i];
        desired_features_image_(i, 1) = desired_features.y[i];
        desired_features_normalized_(i, 0) =
            (desired_features.x[i] - camera_.u_0) / camera_.f_x;
        desired_features_normalized_(i, 1) =
            (desired_features.y[i] - camera_.v_0) / camera_.f_y;
    }

    centered_moments_desired_ =
        computeCenteredMoments(desired_features_normalized_);
    extractCentroid(desired_features_normalized_,
                    desired_centroid_x_, desired_centroid_y_);
    desired_area_ =
        arma::accu(centered_moments_desired_(arma::span(2,2), arma::span(0,2)));
}

// ─────────────────────────────────────────────────────────────────────────────
IBVSController::ControlOutput
IBVSController::computeControlLaw(const VisualFeatures &current_features,
                                  float depth_estimate,
                                  const MarkerVelocity &ff)
{
    ControlOutput output = {0, 0, 0, 0, 0};

    if (!camera_initialized_)
    { ROS_WARN("IBVS: Camera not initialised."); return output; }

    if ((int)current_features.x.size() != NUM_FEATURES ||
        (int)current_features.y.size() != NUM_FEATURES)
    { ROS_ERROR("IBVS: Invalid feature count."); return output; }

    // ── 1. Normalise current pixel coordinates ──────────────────────────────
    arma::mat current_normalized(NUM_FEATURES, 2);
    for (int i = 0; i < NUM_FEATURES; ++i)
    {
        current_normalized(i, 0) =
            (current_features.x[i] - camera_.u_0) / camera_.f_x;
        current_normalized(i, 1) =
            (current_features.y[i] - camera_.v_0) / camera_.f_y;
    }

    // ── 2. Moments, centroid, area ───────────────────────────────────────────
    centered_moments_current_ = computeCenteredMoments(current_normalized);
    extractCentroid(current_normalized, current_centroid_x_, current_centroid_y_);

    float current_area =
        arma::accu(centered_moments_current_(arma::span(2,2), arma::span(0,2)));
    current_normalized_area_ =
        depth_estimate * std::sqrt(desired_area_ / std::max(current_area, EPSILON));

    // ── 3. Jacobian & error ──────────────────────────────────────────────────
    computeJacobian(current_area, desired_area_,
                    current_centroid_x_, current_centroid_y_, depth_estimate);
    computeError(current_normalized, depth_estimate);

    output.error_norm = arma::norm(error_, 2);

    // ── 4. Adaptive gain ─────────────────────────────────────────────────────
    if (!first_computation_)
    {
        error_norm_max_  = output.error_norm;
        first_computation_ = true;
    }
    float lambda = computeAdaptiveGain(output.error_norm);

    // ── 5. Pseudo-inverse ────────────────────────────────────────────────────
    jacobian_pinv_ = arma::pinv(jacobian_);

    // ── 6. Feedback term:  v_fb = -λ J⁺ e ───────────────────────────────────
    control_velocity_ = -lambda * jacobian_pinv_ * error_;

    // ── 7. Feedforward term ──────────────────────────────────────────────────
    // The Kalman filter in MarkerDetector estimates the image-plane velocity of
    // the marker centroid and area.  Adding it here lets the drone *lead* the
    // target instead of always lagging one control period behind.
    //
    // The sign convention matches the feedback law:
    //   - if the marker is moving right  (+vx_centroid), we need to move right
    //     → positive v_x feedforward
    //   - if the marker is growing  (+v_area), we need to descend to follow
    //     → positive v_z feedforward (camera gets closer)
    //
    // A tunable blending weight (ff_gain) lets you dial in how aggressively
    // the feedforward contributes.  Start at 0.3 and increase for faster targets.
    constexpr float ff_gain = 0.35f;

    control_velocity_(0) += ff_gain * ff.vx_centroid; // x translate
    control_velocity_(1) += ff_gain * ff.vy_centroid; // y translate
    control_velocity_(2) += ff_gain * ff.v_area;      // z (depth)
    control_velocity_(3) += ff_gain * ff.v_alpha;     // yaw

    // ── 8. Extract outputs ───────────────────────────────────────────────────
    output.v_x       = control_velocity_(0);
    output.v_y       = control_velocity_(1);
    output.v_z       = control_velocity_(2);
    output.omega_yaw = control_velocity_(3);

    ROS_INFO_THROTTLE(1.0,
        COLOR_YELLOW "IBVS: err=%.4f λ=%.3f  v=[%.3f %.3f %.3f %.3f]  "
                     "ff=[%.3f %.3f %.3f %.3f]" COLOR_RESET,
        output.error_norm, lambda,
        output.v_x, output.v_y, output.v_z, output.omega_yaw,
        ff.vx_centroid, ff.vy_centroid, ff.v_area, ff.v_alpha);

    return output;
}

// ─────────────────────────────────────────────────────────────────────────────
arma::mat IBVSController::computeCenteredMoments(const arma::mat &features)
{
    arma::mat cm = arma::zeros<arma::mat>(4, 4);
    float cx = 0, cy = 0;
    extractCentroid(features, cx, cy);

    arma::mat cf(NUM_FEATURES, 2);
    for (int i = 0; i < NUM_FEATURES; ++i)
    {
        cf(i, 0) = features(i, 0) - cx;
        cf(i, 1) = features(i, 1) - cy;
    }

    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
        {
            double s = 0;
            for (int k = 0; k < NUM_FEATURES; ++k)
                s += std::pow(cf(k,0), i) * std::pow(cf(k,1), j);
            cm(i, j) = s;
        }
    return cm;
}

// ─────────────────────────────────────────────────────────────────────────────
void IBVSController::extractCentroid(const arma::mat &features,
                                     float &cx, float &cy)
{
    cx = cy = 0;
    for (int i = 0; i < NUM_FEATURES; ++i)
    { cx += features(i,0); cy += features(i,1); }
    cx /= NUM_FEATURES;
    cy /= NUM_FEATURES;
}

// ─────────────────────────────────────────────────────────────────────────────
void IBVSController::computeJacobian(float current_area, float desired_area,
                                     float cx, float cy, float depth)
{
    jacobian_ = -1.0 * arma::diagmat(arma::ones<arma::vec>(4));
    float a_n = depth * std::sqrt(desired_area / std::max(current_area, EPSILON));
    jacobian_(0, 3) =  cy;
    jacobian_(1, 3) = -cx;
    jacobian_(2, 3) =  a_n * cx;
    jacobian_(3, 3) = -a_n * cx;
}

// ─────────────────────────────────────────────────────────────────────────────
void IBVSController::computeError(const arma::mat &current_features, float depth)
{
    error_.zeros();
    error_(0) = current_normalized_area_ * current_centroid_x_
                - depth * desired_centroid_x_;
    error_(1) = current_normalized_area_ * current_centroid_y_
                - depth * desired_centroid_y_;

    float ca = arma::accu(
        centered_moments_current_(arma::span(2,2), arma::span(0,2)));
    error_(2) = current_normalized_area_ - depth;

    float u20   = centered_moments_current_(2, 0) / NUM_FEATURES;
    float u02   = centered_moments_current_(0, 2) / NUM_FEATURES;
    float u20_d = centered_moments_desired_(2, 0)  / NUM_FEATURES;
    float u02_d = centered_moments_desired_(0, 2)  / NUM_FEATURES;

    float alpha   = 0.5f * std::atan2(2.f * u20,   u02   - u20);
    float alpha_d = 0.5f * std::atan2(2.f * u20_d, u02_d - u20_d);
    error_(3) = alpha - alpha_d;
}

// ─────────────────────────────────────────────────────────────────────────────
float IBVSController::computeAdaptiveGain(float current_error_norm)
{
    // Gain is high when error is large, tapers to lambda_min near the target.
    float lambda_min  = 0.5f;
    float lambda_max  = 1.25f;
    float error_ratio = std::min(
        current_error_norm / std::max(error_norm_max_, EPSILON), 1.0f);
    return lambda_max - (lambda_max - lambda_min) * error_ratio;
}

// ─────────────────────────────────────────────────────────────────────────────
void IBVSController::reset()
{
    first_computation_ = false;
    error_norm_max_    = 1.0f;
    error_.zeros();
    control_velocity_.zeros();
    ROS_INFO("IBVS: Controller reset.");
}
