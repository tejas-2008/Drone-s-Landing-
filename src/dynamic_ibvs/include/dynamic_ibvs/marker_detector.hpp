#ifndef MARKER_DETECTOR_HPP
#define MARKER_DETECTOR_HPP

#include <ros/ros.h>
#include <fiducial_msgs/FiducialArray.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <vector>
#include <memory>
#include <mutex>  // FIX: thread safety for detected_markers_

/**
 * @class MarkerDetector
 * @brief Handles marker detection and visual feature extraction
 *
 * Subscribes to ArUco marker detection topics and provides detected
 * marker corner positions for IBVS control.
 */
class MarkerDetector {
public:
    /**
     * @struct MarkerData
     * @brief Detected marker information
     */
    struct MarkerData {
        int id;             ///< Marker ID
        float x0, y0;       ///< Corner 0 position (pixels)
        float x1, y1;       ///< Corner 1 position (pixels)
        float x2, y2;       ///< Corner 2 position (pixels)
        float x3, y3;       ///< Corner 3 position (pixels)
        float depth = 0.0f; ///< FIX: depth (m) populated from fiducial_transforms
        ros::Time stamp;    ///< FIX: detection timestamp for stale rejection
    };

public:
    explicit MarkerDetector(ros::NodeHandle* nh);
    virtual ~MarkerDetector() = default;

    /**
     * @brief Get detected marker by ID
     * @param marker_id ID of the marker to retrieve
     * @return Pointer to MarkerData if found and fresh, nullptr if not found or stale
     */
    std::shared_ptr<MarkerData> getMarkerById(int marker_id) const;

    /**
     * @brief Get all currently detected, non-stale markers
     * @return Vector of fresh MarkerData pointers
     */
    std::vector<std::shared_ptr<MarkerData>> getAllMarkers() const;

    /**
     * @brief Get number of currently detected (fresh) markers
     * @return Number of detected markers
     */
    int getMarkerCount() const {
        return static_cast<int>(getAllMarkers().size());
    }

    /**
     * @brief Check if specific marker is detected and fresh
     * @param marker_id ID to check
     * @return true if marker is currently detected and within timeout
     */
    bool isMarkerDetected(int marker_id) const;

    /**
     * @brief Check if camera info has been received
     * @return true if camera intrinsics are available
     */
    bool hasCameraInfo() const { return camera_info_received_; }

    /**
     * @brief Get camera focal length X
     */
    float getCameraFx() const { return camera_f_x_; }

    /**
     * @brief Get camera focal length Y
     */
    float getCameraFy() const { return camera_f_y_; }

    /**
     * @brief Get camera principal point U
     */
    float getCameraU0() const { return camera_u_0_; }

    /**
     * @brief Get camera principal point V
     */
    float getCameraV0() const { return camera_v_0_; }

private:
    // ── ROS interfaces ────────────────────────────────────────────────────
    ros::Subscriber fiducial_vertices_sub_;
    ros::Subscriber fiducial_transforms_sub_;
    ros::Subscriber camera_info_sub_;   ///< Shut down after first receipt
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    // ── Marker state ──────────────────────────────────────────────────────
    std::vector<std::shared_ptr<MarkerData>> detected_markers_;
    mutable std::mutex markers_mutex_;  ///< FIX: guards detected_markers_

    // ── Camera intrinsics ─────────────────────────────────────────────────
    float camera_f_x_, camera_f_y_;
    float camera_u_0_, camera_v_0_;
    bool camera_info_received_;

    // ── Config ────────────────────────────────────────────────────────────
    /// FIX: detections older than this are treated as missing (seconds)
    static constexpr double MARKER_TIMEOUT_SEC = 0.2;

    // ── Callbacks ─────────────────────────────────────────────────────────

    /**
     * @brief Callback for fiducial vertices (marker corner positions)
     */
    void fiducialVerticesCallback(const fiducial_msgs::FiducialArray::ConstPtr& msg);

    /**
     * @brief Callback for fiducial transforms
     *        FIX: now extracts per-marker depth from transform.translation.z
     */
    void fiducialTransformsCallback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg);

    /**
     * @brief Callback for camera information — shuts down subscriber after first receipt
     */
    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

    /**
     * @brief Callback for raw image stream
     */
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);

    // ── Helpers ───────────────────────────────────────────────────────────

    /**
     * @brief FIX: Canonicalise corner winding order to clockwise from top-left.
     *
     * aruco_detect corner ordering can be inconsistent when the marker is near
     * the image edge. Inconsistent ordering corrupts the u11 cross-moment and
     * therefore the orientation error in the IBVS controller.
     *
     * @param marker MarkerData whose corners are sorted in-place
     */
    static void sortCornersClockwise(MarkerData& marker);
};

#endif // MARKER_DETECTOR_HPP
