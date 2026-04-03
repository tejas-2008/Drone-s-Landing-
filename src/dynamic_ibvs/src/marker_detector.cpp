#include "dynamic_ibvs/marker_detector.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

MarkerDetector::MarkerDetector(ros::NodeHandle* nh)
    : it_(*nh),
      camera_f_x_(0), camera_f_y_(0),
      camera_u_0_(0), camera_v_0_(0),
      camera_info_received_(false),
      marker_timeout_(0.2) { // FIX 1: stale marker timeout (seconds)

    fiducial_vertices_sub_ = nh->subscribe("/fiducial_vertices", 10, &MarkerDetector::fiducialVerticesCallback, this);
    fiducial_transforms_sub_ = nh->subscribe("/fiducial_transforms", 10, &MarkerDetector::fiducialTransformsCallback, this);
    camera_info_sub_ = nh->subscribe("/iris_downward_depth_camera/camera/rgb/camera_info", 1, &MarkerDetector::cameraInfoCallback, this);
    image_sub_ = it_.subscribe("/iris_downward_depth_camera/camera/rgb/image_raw", 2, &MarkerDetector::imageCallback, this);
    
    image_pub_ = it_.advertise("platform_ibvs/output_image", 2);

    ROS_INFO("MarkerDetector: Initialized and waiting for detections");
}

std::shared_ptr<MarkerDetector::MarkerData> MarkerDetector::getMarkerById(
    int marker_id) const {
    // FIX 2: Acquire lock before reading detected_markers_ to prevent
    // race condition with fiducialVerticesCallback clearing the vector
    // while imageCallback or external callers are iterating it.
    std::lock_guard<std::mutex> lock(markers_mutex_);

    for (const auto& marker : detected_markers_) {
        if (!marker || marker->id != marker_id) continue;

        // FIX 3: Reject stale detections. Without this, if the marker
        // leaves the frame the last known position is returned forever,
        // causing the controller to chase a ghost.
        double age = (ros::Time::now() - marker->stamp).toSec();
        if (age > marker_timeout_) {
            ROS_WARN_THROTTLE(1.0, "MarkerDetector: Marker %d is stale (%.2fs old), ignoring",
                              marker_id, age);
            return nullptr;
        }
        return marker;
    }
    return nullptr;
}

std::vector<std::shared_ptr<MarkerDetector::MarkerData>> 
MarkerDetector::getAllMarkers() const {
    std::lock_guard<std::mutex> lock(markers_mutex_);
    // Return only fresh markers
    std::vector<std::shared_ptr<MarkerData>> fresh;
    double now = ros::Time::now().toSec();
    for (const auto& marker : detected_markers_) {
        if (marker && (now - marker->stamp.toSec()) <= marker_timeout_) {
            fresh.push_back(marker);
        }
    }
    return fresh;
}

bool MarkerDetector::isMarkerDetected(int marker_id) const {
    return getMarkerById(marker_id) != nullptr;
}

// FIX 4: Extract per-marker depth from the transform array and store it
// alongside each marker. Previously this callback did nothing, so depth
// estimates came from an external source and had no per-marker association.
void MarkerDetector::fiducialTransformsCallback(
    const fiducial_msgs::FiducialTransformArray::ConstPtr& msg) {

    std::lock_guard<std::mutex> lock(markers_mutex_);

    for (const auto& ft : msg->transforms) {
        for (auto& marker : detected_markers_) {
            if (marker && marker->id == ft.fiducial_id) {
                // Z component of translation = depth from camera to marker
                marker->depth = ft.transform.translation.z;
                ROS_DEBUG("MarkerDetector: Marker %d depth updated: %.3fm",
                          marker->id, marker->depth);
            }
        }
    }
}

void MarkerDetector::fiducialVerticesCallback(
    const fiducial_msgs::FiducialArray::ConstPtr& msg) {

    // FIX 5: Lock before modifying detected_markers_ to prevent race
    // with imageCallback and getMarkerById reads.
    std::lock_guard<std::mutex> lock(markers_mutex_);

    detected_markers_.clear();
    
    for (const auto& fiducial : msg->fiducials) {
        auto marker = std::make_shared<MarkerData>();
        marker->id = fiducial.fiducial_id;
        marker->x0 = fiducial.x0;
        marker->y0 = fiducial.y0;
        marker->x1 = fiducial.x1;
        marker->y1 = fiducial.y1;
        marker->x2 = fiducial.x2;
        marker->y2 = fiducial.y2;
        marker->x3 = fiducial.x3;
        marker->y3 = fiducial.y3;
        marker->depth = 0.0f; // will be filled by fiducialTransformsCallback

        // FIX 6: Stamp each detection so stale checks work correctly.
        marker->stamp = msg->header.stamp;

        // FIX 7: Canonicalize corner ordering by angle from centroid
        // (top-left → clockwise). aruco_detect corner order can be
        // inconsistent near image edges; inconsistent ordering corrupts
        // the u11 cross-moment and orientation error.
        sortCornersClockwise(*marker);

        detected_markers_.push_back(marker);
    }

    ROS_DEBUG("MarkerDetector: Detected %zu markers", detected_markers_.size());
}

void MarkerDetector::cameraInfoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& msg) {
    if (!camera_info_received_) {
        camera_f_x_ = msg->K[0];
        camera_f_y_ = msg->K[4];
        camera_u_0_ = msg->K[2];
        camera_v_0_ = msg->K[5];
        
        camera_info_received_ = true;
        
        ROS_INFO("MarkerDetector: Camera info received - fx: %.2f, fy: %.2f, u0: %.2f, v0: %.2f",
                camera_f_x_, camera_f_y_, camera_u_0_, camera_v_0_);

        // FIX 8: Shut down the subscriber after first receipt — no need
        // to keep it alive; camera intrinsics don't change at runtime.
        camera_info_sub_.shutdown();
    }
}

void MarkerDetector::imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
            msg, sensor_msgs::image_encodings::BGR8);
        
        // FIX 9: Take a snapshot of markers under lock, then draw without
        // holding the lock — avoids deadlock if drawing is slow.
        std::vector<std::shared_ptr<MarkerData>> snapshot;
        {
            std::lock_guard<std::mutex> lock(markers_mutex_);
            snapshot = detected_markers_;
        }

        for (const auto& marker : snapshot) {
            if (!marker) continue;
            
            cv::Point p0(static_cast<int>(marker->x0), static_cast<int>(marker->y0));
            cv::Point p1(static_cast<int>(marker->x1), static_cast<int>(marker->y1));
            cv::Point p2(static_cast<int>(marker->x2), static_cast<int>(marker->y2));
            cv::Point p3(static_cast<int>(marker->x3), static_cast<int>(marker->y3));
            
            cv::circle(cv_ptr->image, p0, 5, cv::Scalar(0, 255, 0), -1);
            cv::circle(cv_ptr->image, p1, 5, cv::Scalar(0, 255, 0), -1);
            cv::circle(cv_ptr->image, p2, 5, cv::Scalar(0, 255, 0), -1);
            cv::circle(cv_ptr->image, p3, 5, cv::Scalar(0, 255, 0), -1);
            
            cv::line(cv_ptr->image, p0, p1, cv::Scalar(255, 0, 0), 2);
            cv::line(cv_ptr->image, p1, p2, cv::Scalar(255, 0, 0), 2);
            cv::line(cv_ptr->image, p2, p3, cv::Scalar(255, 0, 0), 2);
            cv::line(cv_ptr->image, p3, p0, cv::Scalar(255, 0, 0), 2);
            
            // Also show depth if available
            std::string label = "ID: " + std::to_string(marker->id);
            if (marker->depth > 0.0f) {
                label += " d:" + std::to_string(static_cast<int>(marker->depth * 100)) + "cm";
            }
            cv::putText(cv_ptr->image, label, p0, cv::FONT_HERSHEY_SIMPLEX,
                        0.5, cv::Scalar(0, 0, 255), 2);
        }
        
        image_pub_.publish(cv_ptr->toImageMsg());
        
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("MarkerDetector: cv_bridge exception: %s", e.what());
    }
}

// FIX 7 (implementation): Sort corners clockwise from top-left.
// Compute centroid, get angle of each corner relative to centroid,
// then sort ascending by angle starting from the top-left quadrant.
void MarkerDetector::sortCornersClockwise(MarkerData& marker) {
    // Collect corners
    std::vector<cv::Point2f> corners = {
        {marker.x0, marker.y0},
        {marker.x1, marker.y1},
        {marker.x2, marker.y2},
        {marker.x3, marker.y3}
    };

    // Centroid
    cv::Point2f centroid(0, 0);
    for (const auto& c : corners) centroid += c;
    centroid *= 0.25f;

    // Sort by angle from centroid (atan2 gives [-π, π], negate y for image coords)
    std::sort(corners.begin(), corners.end(),
              [&centroid](const cv::Point2f& a, const cv::Point2f& b) {
                  float angle_a = std::atan2(a.y - centroid.y, a.x - centroid.x);
                  float angle_b = std::atan2(b.y - centroid.y, b.x - centroid.x);
                  return angle_a < angle_b;
              });

    marker.x0 = corners[0].x; marker.y0 = corners[0].y;
    marker.x1 = corners[1].x; marker.y1 = corners[1].y;
    marker.x2 = corners[2].x; marker.y2 = corners[2].y;
    marker.x3 = corners[3].x; marker.y3 = corners[3].y;
}
