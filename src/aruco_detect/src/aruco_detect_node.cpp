#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TransformStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <fiducial_msgs/Fiducial.h>
#include <fiducial_msgs/FiducialArray.h>
#include <fiducial_msgs/FiducialTransform.h>
#include <fiducial_msgs/FiducialTransformArray.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <map>
#include <string>


#define COLOR_GREEN   "\033[1;32m"
#define COLOR_YELLOW  "\033[1;33m"
#define COLOR_BLUE    "\033[1;34m"
#define COLOR_RESET   "\033[0m"




class ArucoDetector {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    // Subscribers
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_sub_;
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo>* sync_;
    
    // Publishers
    ros::Publisher vertices_pub_;
    ros::Publisher transforms_pub_;
    ros::Publisher camera_info_pub_;
    image_transport::Publisher image_pub_;
    image_transport::ImageTransport it_;
    
    // TF broadcaster
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    // Camera parameters
    sensor_msgs::CameraInfo camera_info_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    bool camera_calibrated_ = false;
    
    // ArUco parameters
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    int marker_size_;
    std::string marker_frame_id_;
    std::string camera_frame_id_;
    std::string aruco_dictionary_;

    
    // Helper function to get ArUco dictionary from string
    int getDictionaryId(const std::string& dict_name) {
        // Create a static map of dictionary names to OpenCV ArUco dictionary enums
        static const std::map<std::string, int> dict_map = {
            {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
            {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
            {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
            {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
            {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
            {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
            {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
            {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
            {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
            {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
            {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
            {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
            {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
            {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
            {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
            {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
            {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL}
        };
        
        // Find the dictionary in the map
        auto it = dict_map.find(dict_name);
        if (it != dict_map.end()) {
            return it->second;
        } else {
            // Default to DICT_4X4_50 if not found
            ROS_WARN("Unknown dictionary '%s', using default DICT_4X4_50", dict_name.c_str());
            return cv::aruco::DICT_4X4_50;
        }
    }
    
public:
    ArucoDetector() : nh_(), pnh_("~"), it_(nh_) {
        // Get parameters from parameter server (loaded from aruco_detect.yaml)
        // Marker size in centimeters
        pnh_.param<int>("marker_size", marker_size_, 5);
        ROS_INFO("Marker size: %d cm", marker_size_);
        
        // Frame IDs
        pnh_.param<std::string>("marker_frame_id", marker_frame_id_, "aruco_marker");
        pnh_.param<std::string>("camera_frame_id", camera_frame_id_, "camera_rgb_optical_frame");
        ROS_INFO("Marker frame ID: %s", marker_frame_id_.c_str());
        ROS_INFO("Camera frame ID: %s", camera_frame_id_.c_str());
        
        // ArUco dictionary selection
        pnh_.param<std::string>("aruco_dictionary", aruco_dictionary_, "DICT_4X4_50");
        ROS_INFO("ArUco dictionary: %s", aruco_dictionary_.c_str());
        
        // Initialize ArUco dictionary (OpenCV 3.4.x compatible)
        int dict_id = getDictionaryId(aruco_dictionary_);
        dictionary_ = cv::aruco::getPredefinedDictionary(dict_id);

        
        // Subscribe to topics
        ROS_INFO(COLOR_BLUE "Subscribing to camera topics..." COLOR_RESET);
        rgb_sub_.subscribe(nh_, "/camera/rgb/image_raw", 10);
        ROS_INFO(COLOR_BLUE "Subscribed to /camera/rgb/image_raw" COLOR_RESET);
        depth_sub_.subscribe(nh_, "/camera/depth/image_raw", 10);
        ROS_INFO(COLOR_BLUE "Subscribed to /camera/depth/image_raw" COLOR_RESET);
        camera_info_sub_.subscribe(nh_, "/camera/rgb/camera_info", 10);
        ROS_INFO(COLOR_BLUE "Subscribed to /camera/rgb/camera_info" COLOR_RESET);

        
        // Time synchronizer
        sync_ = new message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo>(
            rgb_sub_, depth_sub_, camera_info_sub_, 100);
        sync_->registerCallback(boost::bind(&ArucoDetector::imageCallback, this, _1, _2, _3));
        ROS_INFO(COLOR_GREEN "Message synchronizer registered, waiting for messages..." COLOR_RESET);
        
        // Advertise publishers
        vertices_pub_ = nh_.advertise<fiducial_msgs::FiducialArray>("fiducial_vertices", 10);
        transforms_pub_ = nh_.advertise<fiducial_msgs::FiducialTransformArray>("fiducial_transforms", 10);
        camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/camera/rgb/camera_info", 10);
        image_pub_ = it_.advertise("/fiducial_images", 10);
        
        ROS_INFO("ArUco Detector initialized");
    }
    
    ~ArucoDetector() {
        if (sync_) delete sync_;
    }
    
    void imageCallback(const sensor_msgs::ImageConstPtr& rgb_msg,
                       const sensor_msgs::ImageConstPtr& depth_msg,
                       const sensor_msgs::CameraInfoConstPtr& camera_info_msg) {
        try {

            // Camera calibration (only needs to be done once)
            if (!camera_calibrated_) {
                ROS_WARN(COLOR_YELLOW ">>>>> imageCallback TRIGGERED <<<<< (RGB time: %f)" COLOR_RESET, rgb_msg->header.stamp.toSec()); 
                ROS_INFO(COLOR_GREEN "Received camera info" COLOR_RESET);
                camera_info_ = *camera_info_msg;
                // Extract camera matrix
                camera_matrix_ = cv::Mat(3, 3, CV_64F);
                for (int i = 0; i < 9; i++) {
                    camera_matrix_.at<double>(i / 3, i % 3) = camera_info_msg->K[i];
                }
                
                // Extract distortion coefficients
                dist_coeffs_ = cv::Mat(5, 1, CV_64F);
                for (int i = 0; i < 5; i++) {
                    dist_coeffs_.at<double>(i) = camera_info_msg->D[i];
                }
                
                camera_calibrated_ = true;
                ROS_INFO("Camera calibrated with matrix: \n%f %f %f\n%f %f %f\n%f %f %f",
                         camera_matrix_.at<double>(0, 0), camera_matrix_.at<double>(0, 1), camera_matrix_.at<double>(0, 2),
                         camera_matrix_.at<double>(1, 0), camera_matrix_.at<double>(1, 1), camera_matrix_.at<double>(1, 2),
                         camera_matrix_.at<double>(2, 0), camera_matrix_.at<double>(2, 1), camera_matrix_.at<double>(2, 2));
            }
            
            // Convert RGB image
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat image = cv_ptr->image;
            // Convert depth image
            cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth_msg);
            cv::Mat depth_image = depth_ptr->image;
            cv::Mat gray;
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
            
            
            // Detect markers
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            // Create output messages
            fiducial_msgs::FiducialArray vertices_msg;
            fiducial_msgs::FiducialTransformArray transforms_msg;
            
            vertices_msg.header = rgb_msg->header;
            transforms_msg.header = rgb_msg->header;

        
            // Estimate pose logic
            std::vector<cv::Vec3d> rvecs, tvecs;
            
            cv::aruco::detectMarkers(gray, dictionary_, corners, ids);
            // Standard single marker pose estimation
            if (camera_calibrated_ && !corners.empty()) {
                cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);
            }


            // Fill vertices message
            for (size_t i = 0; i < ids.size(); i++) {
                std::vector<cv::Point2f>& marker_corners = corners[i];
                fiducial_msgs::Fiducial fiducial;
                fiducial.fiducial_id = ids[i];
                if (marker_corners.size() >= 4) {
                    fiducial.x0 = marker_corners[0].x;
                    fiducial.y0 = marker_corners[0].y;
                    fiducial.x1 = marker_corners[1].x;
                    fiducial.y1 = marker_corners[1].y;
                    fiducial.x2 = marker_corners[2].x;
                    fiducial.y2 = marker_corners[2].y;
                    fiducial.x3 = marker_corners[3].x;
                    fiducial.y3 = marker_corners[3].y;
                }
                vertices_msg.fiducials.push_back(fiducial);
            }
            vertices_pub_.publish(vertices_msg);
            
            // Fill transforms message
            // Standard individual markers
            for (size_t i = 0; i < ids.size(); i++) {
                if (camera_calibrated_ && i < rvecs.size() && i < tvecs.size()) {
                    fiducial_msgs::FiducialTransform ft;
                    ft.fiducial_id = ids[i];
                    
                    geometry_msgs::TransformStamped transform;
                    transform.header = rgb_msg->header;
                    transform.child_frame_id = marker_frame_id_ + "_" + std::to_string(ids[i]);
                    
                    transform.transform.translation.x = tvecs[i][0] / 100.0;
                    transform.transform.translation.y = tvecs[i][1] / 100.0;
                    transform.transform.translation.z = tvecs[i][2] / 100.0;
                    
                    cv::Mat rotation_matrix;
                    cv::Rodrigues(rvecs[i], rotation_matrix);
                    cv::Mat_<double> R = rotation_matrix;
                    double m00 = R(0, 0), m01 = R(0, 1), m02 = R(0, 2);
                    double m10 = R(1, 0), m11 = R(1, 1), m12 = R(1, 2);
                    double m20 = R(2, 0), m21 = R(2, 1), m22 = R(2, 2);
                    double trace = m00 + m11 + m22;
                    double qx, qy, qz, qw;
                    if (trace > 0) {
                        double s = 0.5 / sqrt(trace + 1.0);
                        qw = 0.25 / s;
                        qx = (m21 - m12) * s;
                        qy = (m02 - m20) * s;
                        qz = (m10 - m01) * s;
                    } else if ((m00 > m11) && (m00 > m22)) {
                        double s = 2.0 * sqrt(1.0 + m00 - m11 - m22);
                        qw = (m21 - m12) / s;
                        qx = 0.25 * s;
                        qy = (m01 + m10) / s;
                        qz = (m02 + m20) / s;
                    } else if (m11 > m22) {
                        double s = 2.0 * sqrt(1.0 + m11 - m00 - m22);
                        qw = (m02 - m20) / s;
                        qx = (m01 + m10) / s;
                        qy = 0.25 * s;
                        qz = (m12 + m21) / s;
                    } else {
                        double s = 2.0 * sqrt(1.0 + m22 - m00 - m11);
                        qw = (m10 - m01) / s;
                        qx = (m02 + m20) / s;
                        qy = (m12 + m21) / s;
                        qz = 0.25 * s;
                    }
                    transform.transform.rotation.x = qx;
                    transform.transform.rotation.y = qy;
                    transform.transform.rotation.z = qz;
                    transform.transform.rotation.w = qw;
                    
                    ft.transform = transform.transform;
                    transforms_msg.transforms.push_back(ft);
                    tf_broadcaster_.sendTransform(transform);
                    
                    cv::aruco::drawAxis(image, camera_matrix_, dist_coeffs_, rvecs[i], tvecs[i], marker_size_ * 0.5);
                }
            }
            
            transforms_pub_.publish(transforms_msg);
            camera_info_pub_.publish(camera_info_);
            
            // Draw detected markers
            cv::aruco::drawDetectedMarkers(image, corners, ids);
            
            cv_ptr->image = image;
            image_pub_.publish(cv_ptr->toImageMsg());
            
            ROS_INFO(COLOR_GREEN "Detected %lu markers" COLOR_RESET);
            
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco_detect_node");
    
    ArucoDetector detector;
    
    ros::spin();
    
    return 0;
}
