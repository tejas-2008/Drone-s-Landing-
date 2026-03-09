#include "dynamic_ibvs/marker_detector.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

MarkerDetector::MarkerDetector(ros::NodeHandle* nh)
    : it_(*nh),
      camera_f_x_(0), camera_f_y_(0),
      camera_u_0_(0), camera_v_0_(0),
      camera_info_received_(false) {

    // Subscribe to marker detection topics
    fiducial_vertices_sub_ = nh->subscribe("fiducial_vertices", 10, &MarkerDetector::fiducialVerticesCallback, this);
    fiducial_transforms_sub_ = nh->subscribe("fiducial_transforms", 10, &MarkerDetector::fiducialTransformsCallback, this);
    camera_info_sub_ = nh->subscribe("camera_info", 1, &MarkerDetector::cameraInfoCallback, this);
    image_sub_ = it_.subscribe("image_raw", 2, &MarkerDetector::imageCallback, this);
    
    image_pub_ = it_.advertise("platform_ibvs/output_image", 2);

    ROS_INFO("MarkerDetector: Initialized and waiting for detections");
}

std::shared_ptr<MarkerDetector::MarkerData> MarkerDetector::getMarkerById(
    int marker_id) const {
    for (const auto& marker : detected_markers_) {
        if (marker && marker->id == marker_id) {
            return marker;
        }
    }
    return nullptr;
}

std::vector<std::shared_ptr<MarkerDetector::MarkerData>> 
MarkerDetector::getAllMarkers() const {
    return detected_markers_;
}

bool MarkerDetector::isMarkerDetected(int marker_id) const {
    return getMarkerById(marker_id) != nullptr;
}

void MarkerDetector::fiducialVerticesCallback(
    const fiducial_msgs::FiducialArray::ConstPtr& msg) {
    
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
        
        detected_markers_.push_back(marker);
    }

    ROS_DEBUG("MarkerDetector: Detected %zu markers", detected_markers_.size());
}

void MarkerDetector::fiducialTransformsCallback(
    const fiducial_msgs::FiducialTransformArray::ConstPtr& msg) {
    
    ROS_DEBUG("MarkerDetector: Received fiducial transform array with %zu transforms",
             msg->transforms.size());
}

void MarkerDetector::cameraInfoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& msg) {
    
    if (!camera_info_received_) {
        camera_f_x_ = msg->K[0];  // fx
        camera_f_y_ = msg->K[4];  // fy
        camera_u_0_ = msg->K[2];  // cx
        camera_v_0_ = msg->K[5];  // cy
        
        camera_info_received_ = true;
        
        ROS_INFO("MarkerDetector: Camera info received - fx: %.2f, fy: %.2f, u0: %.2f, v0: %.2f",
                camera_f_x_, camera_f_y_, camera_u_0_, camera_v_0_);
    }
}

void MarkerDetector::imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
            msg, sensor_msgs::image_encodings::BGR8);
        
        // Draw detected markers on the image
        for (const auto& marker : detected_markers_) {
            if (!marker) continue;
            
            // Draw marker corners
            cv::Point p0(static_cast<int>(marker->x0), static_cast<int>(marker->y0));
            cv::Point p1(static_cast<int>(marker->x1), static_cast<int>(marker->y1));
            cv::Point p2(static_cast<int>(marker->x2), static_cast<int>(marker->y2));
            cv::Point p3(static_cast<int>(marker->x3), static_cast<int>(marker->y3));
            
            // Draw corners as circles
            cv::circle(cv_ptr->image, p0, 5, cv::Scalar(0, 255, 0), -1);
            cv::circle(cv_ptr->image, p1, 5, cv::Scalar(0, 255, 0), -1);
            cv::circle(cv_ptr->image, p2, 5, cv::Scalar(0, 255, 0), -1);
            cv::circle(cv_ptr->image, p3, 5, cv::Scalar(0, 255, 0), -1);
            
            // Draw edges connecting corners
            cv::line(cv_ptr->image, p0, p1, cv::Scalar(255, 0, 0), 2);
            cv::line(cv_ptr->image, p1, p2, cv::Scalar(255, 0, 0), 2);
            cv::line(cv_ptr->image, p2, p3, cv::Scalar(255, 0, 0), 2);
            cv::line(cv_ptr->image, p3, p0, cv::Scalar(255, 0, 0), 2);
            
            // Add text with marker ID
            cv::putText(cv_ptr->image, "ID: " + std::to_string(marker->id),
                       p0, cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                       cv::Scalar(0, 0, 255), 2);
        }
        
        image_pub_.publish(cv_ptr->toImageMsg());
        
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("MarkerDetector: cv_bridge exception: %s", e.what());
    }
}
