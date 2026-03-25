#ifndef IBVS_CONTROLLER_HPP
#define IBVS_CONTROLLER_HPP

#include <armadillo>
#include <vector>
#include <memory>

/**
 * @class IBVSController
 * @brief Image-Based Visual Servoing controller using visual moments.
 *
 * Changes vs. original:
 *  - MarkerVelocity struct carries Kalman-estimated image-plane velocity
 *    from MarkerDetector.
 *  - computeControlLaw() accepts an optional MarkerVelocity feedforward term.
 *    When provided, the estimated marker velocity is added to the feedback
 *    output so the drone leads the target rather than chasing it.
 *  - setDesiredFeatures() may be called every tick (dynamic desired mode).
 */
class IBVSController {
public:
    // ── Structs ───────────────────────────────────────────────────────────────

    struct CameraIntrinsics {
        float f_x, f_y;     ///< Focal lengths
        float u_0, v_0;     ///< Principal point
        CameraIntrinsics() : f_x(0), f_y(0), u_0(0), v_0(0) {}
    };

    struct VisualFeatures {
        std::vector<float> x;   ///< Pixel u coordinates (4 corners)
        std::vector<float> y;   ///< Pixel v coordinates (4 corners)
    };

    /**
     * @struct MarkerVelocity
     * @brief Kalman-estimated image-plane velocity of the tracked marker.
     *
     * All quantities are in normalised image coordinates per second.
     * Supplied by MarkerDetector::getMarkerVelocity() and injected into
     * computeControlLaw() as a feedforward term.
     */
    struct MarkerVelocity {
        float vx_centroid = 0.f; ///< Centroid velocity in X  [norm/s]
        float vy_centroid = 0.f; ///< Centroid velocity in Y  [norm/s]
        float v_area      = 0.f; ///< Rate of change of area  [norm/s]
        float v_alpha     = 0.f; ///< Rate of change of angle [rad/s]
    };

    struct ControlOutput {
        float v_x;          ///< Linear velocity X
        float v_y;          ///< Linear velocity Y
        float v_z;          ///< Linear velocity Z
        float omega_yaw;    ///< Angular velocity (yaw)
        float error_norm;   ///< L2 norm of feature error
    };

    // ── Public interface ──────────────────────────────────────────────────────

    IBVSController();
    virtual ~IBVSController() = default;

    /** Set camera intrinsics (call once after camera info is received). */
    void setActiveCamera(const CameraIntrinsics& intrinsics);

    /**
     * Set desired visual feature positions.
     * May be called every control tick when dynamic_desired mode is active —
     * the desired corners stay at image centre so the error always reflects
     * "how far is the marker from the centre right now".
     */
    void setDesiredFeatures(const VisualFeatures& desired_features, float depth);

    /**
     * Compute one step of the IBVS control law.
     *
     * @param current_features  Current marker corner pixel coordinates.
     * @param depth_estimate    Estimated camera-to-marker distance [m].
     * @param ff                Optional Kalman feedforward term.  Pass a
     *                          zero-initialised struct when not available.
     * @return ControlOutput    Saturate before publishing to MAVROS.
     */
    ControlOutput computeControlLaw(const VisualFeatures& current_features,
                                    float depth_estimate,
                                    const MarkerVelocity& ff = MarkerVelocity{});

    /** Current 4x4 centred moments matrix (for diagnostics). */
    arma::mat getCurrentMoments() const { return centered_moments_current_; }

    /** Current 4-element error vector [x, y, area, orientation]. */
    arma::vec getCurrentError()   const { return error_; }

    /** Reset adaptive gain state (call after marker loss or mode change). */
    void reset();

private:
    static const int NUM_FEATURES = 4;

    CameraIntrinsics camera_;
    bool camera_initialized_;

    arma::mat desired_features_image_;
    arma::mat desired_features_normalized_;
    arma::mat centered_moments_desired_;
    float desired_area_;
    float desired_centroid_x_, desired_centroid_y_;

    arma::mat centered_moments_current_;
    float current_centroid_x_, current_centroid_y_;
    float current_normalized_area_;

    arma::vec error_;
    arma::mat jacobian_;
    arma::mat jacobian_pinv_;
    arma::vec control_velocity_;

    float error_norm_max_;
    bool  first_computation_;

    arma::mat computeCenteredMoments(const arma::mat& features);
    void      extractCentroid(const arma::mat& features,
                              float& centroid_x, float& centroid_y);
    void      computeJacobian(float current_area, float desired_area,
                              float cx, float cy, float depth);
    void      computeError(const arma::mat& current_features, float depth);
    float     computeAdaptiveGain(float current_error_norm);
};

#endif // IBVS_CONTROLLER_HPP
