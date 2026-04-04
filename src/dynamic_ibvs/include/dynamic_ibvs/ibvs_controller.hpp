#ifndef IBVS_CONTROLLER_HPP
#define IBVS_CONTROLLER_HPP

#include <armadillo>
#include <memory>
#include <ros/ros.h>
#include <vector>

/**
 * @class IBVSController
 * @brief Image-Based Visual Servoing controller using visual moments
 *
 * This class implements the core IBVS control law based on visual moments
 * and marker corner features. It computes control velocities to minimize
 * visual feature error.
 */
class IBVSController {
public:
  /**
   * @struct CameraIntrinsics
   * @brief Camera calibration parameters
   */
  struct CameraIntrinsics {
    float f_x; ///< Focal length in X direction
    float f_y; ///< Focal length in Y direction
    float u_0; ///< Principal point X coordinate
    float v_0; ///< Principal point Y coordinate

    CameraIntrinsics() : f_x(0), f_y(0), u_0(0), v_0(0) {}
  };

  /**
   * @struct VisualFeatures
   * @brief Visual feature point data
   */
  struct VisualFeatures {
    std::vector<float> x; ///< X coordinates (pixel)
    std::vector<float> y; ///< Y coordinates (pixel)
  };

  /**
   * @struct ControlOutput
   * @brief Generated control velocities
   */
  struct ControlOutput {
    float v_x;        ///< Linear velocity X
    float v_y;        ///< Linear velocity Y
    float v_z;        ///< Linear velocity Z
    float omega_yaw;  ///< Angular velocity (yaw)
    float error_norm; ///< L2 norm of feature error
  };

public:
  IBVSController();
  virtual ~IBVSController() = default;

  /**
   * @brief Initialize camera intrinsics
   * @param intrinsics Camera calibration parameters
   */
  void setActiveCamera(const CameraIntrinsics &intrinsics);

  void setGainLimits(float lambda_min, float lambda_max, float kd_gain = 0.0f,
                     float ki_gain = 0.0f, float integral_windup_limit = 0.5f);

  /**
   * @brief Set desired visual feature positions
   * @param desired_features Desired feature point coordinates
   * @param depth Desired depth for normalization
   */
  void setDesiredFeatures(const VisualFeatures &desired_features, float depth);

  /**
   * @brief Compute control law for current visual features
   * @param current_features Current marker corner coordinates
   * @param depth_estimate Current depth estimate
   * @return ControlOutput with computed velocities and error
   */
  ControlOutput computeControlLaw(const VisualFeatures &current_features,
                                  float depth_estimate);

  /**
   * @brief Get current visual moments
   * @return 4x4 centered moments matrix
   */
  arma::mat getCurrentMoments() const { return centered_moments_current_; }

  /**
   * @brief Get current feature error
   * @return 4-element error vector [x_err, y_err, area_err, orientation_err]
   */
  arma::vec getCurrentError() const { return error_; }

  // ── Diagnostics getters ────────────────────────────────────────────────
  float getCurrentCentroidX() const { return current_centroid_x_; }
  float getCurrentCentroidY() const { return current_centroid_y_; }
  float getDesiredCentroidX() const { return desired_centroid_x_; }
  float getDesiredCentroidY() const { return desired_centroid_y_; }

  /**
   * @brief Reset controller state
   */
  void reset();

private:
  // Camera intrinsics
  CameraIntrinsics camera_;
  bool camera_initialized_;

  // Desired feature data
  arma::mat desired_features_image_; ///< Desired features in image coordinates
  arma::mat
      desired_features_normalized_; ///< Desired features normalized by depth
  arma::mat
      centered_moments_desired_; ///< Centered moments for desired features
  float desired_area_;           ///< Desired feature area
  float desired_centroid_x_;     ///< Desired centroid X
  float desired_centroid_y_;     ///< Desired centroid Y

  // Current feature data
  arma::mat
      centered_moments_current_;  ///< Centered moments for current features
  float current_centroid_x_;      ///< Current centroid X
  float current_centroid_y_;      ///< Current centroid Y
  float current_normalized_area_; ///< Current normalized area

  float lambda_max_, lambda_min_;

  // Error and control
  arma::vec error_;            ///< Feature error vector
  arma::mat jacobian_;         ///< Visual Jacobian matrix
  arma::mat jacobian_pinv_;    ///< Pseudo-inverse of Jacobian
  arma::vec control_velocity_; ///< Computed control velocity

  // Control parameters
  float error_norm_max_;   ///< Initial error norm for gain adaptation
  bool first_computation_; ///< First computation flag for error normalization

  // Derivative (D) term
  float kd_gain_;                    ///< Derivative gain (0 = pure P, >0 = PD)
  arma::vec prev_error_;             ///< Previous error vector for ė computation
  arma::vec error_derivative_;       ///< Filtered error derivative (low-pass)
  ros::Time prev_error_stamp_;       ///< Timestamp of previous error
  bool has_prev_error_;              ///< True after first computeControlLaw call
  static constexpr float DERIV_FILTER_ALPHA = 0.15f; ///< Low-pass filter coefficient for ė

  // Integral (I) term
  float ki_gain_;                    ///< Integral gain (0 = no integral action)
  float integral_windup_limit_;      ///< L2 norm cap on integral accumulator
  arma::vec error_integral_;         ///< Accumulated error integral

  /**
   * @brief Compute centered moments for given feature points
   * @param features Feature points (4x2 matrix)
   * @return 4x4 centered moments matrix
   */
  arma::mat computeCenteredMoments(const arma::mat &features);

  /**
   * @brief Extract centroid from features
   * @param features Feature points (4x2 matrix)
   * @param centroid_x Output centroid X coordinate
   * @param centroid_y Output centroid Y coordinate
   */
  void extractCentroid(const arma::mat &features, float &centroid_x,
                       float &centroid_y);

  /**
   * @brief Compute visual Jacobian matrix
   * @param current_area Current area estimate
   * @param desired_area Desired area estimate
   * @param current_centroid_x Current centroid X
   * @param current_centroid_y Current centroid Y
   * @param depth Current depth estimate
   */
  void computeJacobian(float current_area, float desired_area,
                       float current_centroid_x, float current_centroid_y,
                       float depth);

  /**
   * @brief Compute feature error
   * @param current_features Current feature points (normalized)
   * @param depth Current depth estimate
   */
  void computeError(const arma::mat &current_features, float depth);

  /**
   * @brief Compute time-varying adaptive gain
   * @param current_error_norm Current error norm
   * @return Adaptive gain value
   */
  float computeAdaptiveGain(float current_error_norm);
};

#endif // IBVS_CONTROLLER_HPP
