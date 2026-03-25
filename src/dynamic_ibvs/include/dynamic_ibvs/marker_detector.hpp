#ifndef MARKER_DETECTOR_HPP
#define MARKER_DETECTOR_HPP

#include <ros/ros.h>
#include <fiducial_msgs/FiducialArray.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <vector>
#include <memory>
#include <unordered_map>

// MarkerVelocity lives in IBVSController, but we only need the struct here.
// Forward-declare the class and include the real header only in the .cpp to
// avoid a circular dependency.
#include "dynamic_ibvs/ibvs_controller.hpp"

/**
 * @class MarkerDetector
 * @brief Handles ArUco marker detection, visual feature extraction, and
 *        per-marker Kalman filtering for velocity estimation.
 *
 * Changes vs. original:
 *  - Each detected marker has its own 8-state constant-velocity Kalman filter
 *    tracking [cx, vx, cy, vy, area, v_area, alpha, v_alpha] in normalised
 *    image coordinates.
 *  - getMarkerVelocity(id) exposes the estimated velocity half of the state
 *    as an IBVSController::MarkerVelocity for direct use as a feedforward term.
 *  - MarkerData gains a ros::Time stamp field so the KF can compute dt.
 */
class MarkerDetector {
public:
    // ── Structs ───────────────────────────────────────────────────────────────

    struct MarkerData {
        int id;
        float x0, y0;   ///< Corner 0 (pixels)
        float x1, y1;   ///< Corner 1 (pixels)
        float x2, y2;   ///< Corner 2 (pixels)
        float x3, y3;   ///< Corner 3 (pixels)
        ros::Time stamp; ///< Header timestamp of the detection message
    };

    // ── Construction ──────────────────────────────────────────────────────────

    explicit MarkerDetector(ros::NodeHandle* nh);
    virtual ~MarkerDetector() = default;

    // ── Detection accessors ───────────────────────────────────────────────────

    /** Return the marker with the given ID, or nullptr if not visible. */
    std::shared_ptr<MarkerData> getMarkerById(int marker_id) const;

    /** Return all currently detected markers. */
    std::vector<std::shared_ptr<MarkerData>> getAllMarkers() const;

    /** Number of markers visible in the last detection callback. */
    int  getMarkerCount()          const { return static_cast<int>(detected_markers_.size()); }

    /** True if the marker with marker_id is in the current detection list. */
    bool isMarkerDetected(int marker_id) const;

    // ── Kalman velocity accessor ──────────────────────────────────────────────

    /**
     * Return the Kalman-estimated image-plane velocity for the given marker.
     * Returns a zero-initialised MarkerVelocity if the marker has never been
     * seen or has been seen only once (not enough data for velocity).
     *
     * Units: normalised image coordinates per second.
     */
    IBVSController::MarkerVelocity getMarkerVelocity(int marker_id) const;

    // ── Camera intrinsics ─────────────────────────────────────────────────────

    bool  hasCameraInfo() const { return camera_info_received_; }
    float getCameraFx()   const { return camera_f_x_; }
    float getCameraFy()   const { return camera_f_y_; }
    float getCameraU0()   const { return camera_u_0_; }
    float getCameraV0()   const { return camera_v_0_; }

private:
    // ── Kalman filter state (one per marker ID) ───────────────────────────────
    //
    // State vector (8x1):
    //   x = [cx, vx, cy, vy, area, v_area, alpha, v_alpha]
    //
    // where cx/cy are normalised centroid coords, area is normalised marker
    // area (shoelace / fx*fy), and alpha is the orientation angle [rad].
    // Velocities are the corresponding time derivatives.
    //
    // All arithmetic uses plain C arrays (8x8 row-major) so the header stays
    // independent of Armadillo.
    struct MarkerKF {
        bool      initialised = false;
        double    x[8]  = {};    ///< State estimate
        double    P[64] = {};    ///< Covariance (row-major 8x8)
        ros::Time last_update;

        // Convenience getters
        double cx()     const { return x[0]; }
        double vx()     const { return x[1]; }
        double cy()     const { return x[2]; }
        double vy()     const { return x[3]; }
        double area()   const { return x[4]; }
        double varea()  const { return x[5]; }
        double alpha()  const { return x[6]; }
        double valpha() const { return x[7]; }
    };

    // ── Internal KF helpers (implemented in marker_detector.cpp) ─────────────

    void updateKalman(int id,
                      float cx_meas, float cy_meas,
                      float area_meas, float alpha_meas,
                      ros::Time stamp);

    void kfPredict(MarkerKF& kf, double dt);
    void kfUpdate (MarkerKF& kf,
                   double cx_m, double cy_m,
                   double area_m, double alpha_m);

    // ── Geometry helpers ──────────────────────────────────────────────────────

    static float computeCentroid_x     (const MarkerData& m);
    static float computeCentroid_y     (const MarkerData& m);
    static float computeNormalisedArea (const MarkerData& m, float fx, float fy);
    static float computeAlpha          (const MarkerData& m);

    // ── ROS callbacks ─────────────────────────────────────────────────────────

    void fiducialVerticesCallback(
        const fiducial_msgs::FiducialArray::ConstPtr& msg);
    void fiducialTransformsCallback(
        const fiducial_msgs::FiducialTransformArray::ConstPtr& msg);
    void cameraInfoCallback(
        const sensor_msgs::CameraInfo::ConstPtr& msg);
    void imageCallback(
        const sensor_msgs::Image::ConstPtr& msg);

    // ── Members ───────────────────────────────────────────────────────────────

    ros::Subscriber fiducial_vertices_sub_;
    ros::Subscriber fiducial_transforms_sub_;
    ros::Subscriber camera_info_sub_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber     image_sub_;
    image_transport::Publisher      image_pub_;

    std::vector<std::shared_ptr<MarkerData>> detected_markers_;
    std::unordered_map<int, MarkerKF>        kalman_filters_;

    float camera_f_x_, camera_f_y_;
    float camera_u_0_, camera_v_0_;
    bool  camera_info_received_;
};

#endif // MARKER_DETECTOR_HPP
