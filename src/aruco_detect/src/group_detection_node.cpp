#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <yaml-cpp/yaml.h>
#include <fiducial_msgs/FiducialArray.h>
#include <fiducial_msgs/Fiducial.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <map>
#include <string>
#include <vector>
#include <algorithm>
#include <limits>



#define COLOR_GREEN   "\033[1;32m"
#define COLOR_YELLOW  "\033[1;33m"
#define COLOR_BLUE    "\033[1;34m"
#define COLOR_PURPLE  "\033[1;35m"
#define COLOR_RESET   "\033[0m"




class ArucoGroupDetector {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_sub_;
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo>* sync_;

    ros::Publisher pose_pub_;
    ros::Publisher vertices_pub_;
    ros::Publisher fiducial_transforms_pub_;
    image_transport::Publisher image_pub_;
    image_transport::ImageTransport it_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;

    sensor_msgs::CameraInfo camera_info_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    bool camera_calibrated_ = false;

    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::Board> board_;
    std::string camera_frame_id_;
    std::string platform_frame_id_;
    std::string aruco_dictionary_;
    std::string group_definition_file_;

    int getDictionaryId(const std::string& dict_name) {
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
        
        auto it = dict_map.find(dict_name);
        if (it != dict_map.end()) {
            return it->second;
        } else {
            ROS_WARN("Unknown dictionary '%s', using default DICT_4X4_50", dict_name.c_str());
            return cv::aruco::DICT_4X4_50;
        }
    }

public:
    ArucoGroupDetector() : nh_(), pnh_("~"), it_(nh_) {

        //parames Loading......
        pnh_.param<std::string>("camera_frame_id", camera_frame_id_, "camera_rgb_optical_frame");
        pnh_.param<std::string>("aruco_dictionary", aruco_dictionary_, "DICT_4X4_50");
        pnh_.param<std::string>("group_definition_file", group_definition_file_, "");

        int dict_id = getDictionaryId(aruco_dictionary_);
        dictionary_ = cv::aruco::getPredefinedDictionary(dict_id);

        // Load and parse the group definition file
        if (!group_definition_file_.empty()) {
            try {
                YAML::Node config = YAML::LoadFile(group_definition_file_);
                const YAML::Node& groups = config["groups"];
                if (groups && groups.IsSequence()) {
                    // For now, we only load the first group
                    const YAML::Node& group = groups[0];
                    platform_frame_id_ = group["group_name"].as<std::string>();

                    std::vector<std::vector<cv::Point3f>> board_corners;
                    std::vector<int> board_ids;

                    const YAML::Node& markers = group["markers"];
                    if (markers && markers.IsSequence()) {
                        for (const auto& marker : markers) {
                            int id = marker["id"].as<int>();
                            double size = marker["size"].as<double>();
                            std::vector<double> position = marker["position"].as<std::vector<double>>();
                            
                            if (position.size() < 3) {
                                ROS_ERROR("Marker position must have 3 coordinates (x, y, z)");
                                continue;
                            }
                            
                            float half_size = size / 2.0;
                            std::vector<cv::Point3f> corners;
                            corners.push_back(cv::Point3f(position[0] - half_size, position[1] + half_size, position[2]));
                            corners.push_back(cv::Point3f(position[0] + half_size, position[1] + half_size, position[2]));
                            corners.push_back(cv::Point3f(position[0] + half_size, position[1] - half_size, position[2]));
                            corners.push_back(cv::Point3f(position[0] - half_size, position[1] - half_size, position[2]));
                            
                            board_corners.push_back(corners);
                            board_ids.push_back(id);
                        }
                    }
                    
                    // Validate board data before creating board
                    if (!board_corners.empty() && !board_ids.empty() && board_corners.size() == board_ids.size()) {
                        try {
                            board_ = cv::aruco::Board::create(board_corners, dictionary_, board_ids);
                            ROS_INFO("Successfully loaded %zu markers for group '%s'", board_ids.size(), platform_frame_id_.c_str());
                        } catch (const cv::Exception& e) {
                            ROS_ERROR("Failed to create ArUco board: %s", e.what());
                            board_ = nullptr;
                        }
                    } else {
                        ROS_WARN("No valid markers found in configuration. Expected markers: %zu, Loaded markers: %zu", 
                                 board_corners.size(), board_ids.size());
                        board_ = nullptr;
                    }
                } else {
                    ROS_WARN("No groups found in configuration file");
                }
            } catch (const YAML::Exception& e) {
                ROS_ERROR("Failed to load or parse group definition file '%s': %s", group_definition_file_.c_str(), e.what());
            }
        } else {
            ROS_WARN("No group definition file specified");
        }

        // All subscribers
        rgb_sub_.subscribe(nh_, "/camera/rgb/image_raw", 10);
        depth_sub_.subscribe(nh_, "/camera/depth/image_raw", 10);
        camera_info_sub_.subscribe(nh_, "/camera/rgb/camera_info", 10);

        // Time Synchronizer for RGB, Depth, and Camera Info
        sync_ = new message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo>(rgb_sub_, depth_sub_, camera_info_sub_, 100);
        sync_->registerCallback(boost::bind(&ArucoGroupDetector::imageCallback, this, _1, _2, _3));

        
        // Publishers defined 
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("platform_pose", 10);
        vertices_pub_ = nh_.advertise<fiducial_msgs::FiducialArray>("fiducial_vertices", 10);
        fiducial_transforms_pub_ = nh_.advertise<geometry_msgs::PoseArray>("fiducial_transforms", 10);
        image_pub_ = it_.advertise("fiducial_images", 10);

        ROS_INFO("Aruco Group Detector initialized for group '%s'", platform_frame_id_.c_str());
    }

    ~ArucoGroupDetector() {
        if (sync_) delete sync_;
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& rgb_msg,
                       const sensor_msgs::ImageConstPtr& depth_msg,
                       const sensor_msgs::CameraInfoConstPtr& camera_info_msg) {
        try {
            if (!camera_calibrated_) {
                camera_info_ = *camera_info_msg;
                camera_matrix_ = cv::Mat(3, 3, CV_64F);
                for (int i = 0; i < 9; i++) {
                    camera_matrix_.at<double>(i / 3, i % 3) = camera_info_msg->K[i];
                }
                
                dist_coeffs_ = cv::Mat(5, 1, CV_64F);
                for (int i = 0; i < 5; i++) {
                    dist_coeffs_.at<double>(i) = camera_info_msg->D[i];
                }
                
                camera_calibrated_ = true;
                ROS_INFO("Camera calibrated");
            }

            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat image = cv_ptr->image;

            cv::Mat gray;
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            cv::aruco::detectMarkers(gray, dictionary_, corners, ids);

            if (ids.size() > 0) {
                // Publish vertices of markers belonging to the group
                if (board_) {
                    fiducial_msgs::FiducialArray vertices_msg;
                    vertices_msg.header = rgb_msg->header;
                    vertices_msg.image_seq = rgb_msg->header.seq;

                    // Store fiducial transforms
                    geometry_msgs::PoseArray fiducial_transforms;
                    fiducial_transforms.header = rgb_msg->header;

                    // Compute bounding box of all detected markers for platform vertices
                    float min_x = std::numeric_limits<float>::max();
                    float max_x = std::numeric_limits<float>::lowest();
                    float min_y = std::numeric_limits<float>::max();
                    float max_y = std::numeric_limits<float>::lowest();

                    for (size_t i = 0; i < ids.size(); i++) {
                        // Check if detected id is part of the board
                        bool is_in_board = false;
                        for (int board_id : board_->ids) {
                            if (ids[i] == board_id) {
                                is_in_board = true;
                                ROS_INFO("Detected marker ID %d is part of the board", ids[i]);
                                break;
                            }
                        }

                        if (is_in_board) {
                            fiducial_msgs::Fiducial fiducial;
                            fiducial.fiducial_id = ids[i];
                            if (corners[i].size() >= 4) {
                                fiducial.x0 = corners[i][0].x;
                                fiducial.y0 = corners[i][0].y;
                                fiducial.x1 = corners[i][1].x;
                                fiducial.y1 = corners[i][1].y;
                                fiducial.x2 = corners[i][2].x;
                                fiducial.y2 = corners[i][2].y;
                                fiducial.x3 = corners[i][3].x;
                                fiducial.y3 = corners[i][3].y;
                            }
                            vertices_msg.fiducials.push_back(fiducial);

                            // Update bounding box
                            for (const auto& corner : corners[i]) {
                                min_x = std::min(min_x, corner.x);
                                max_x = std::max(max_x, corner.x);
                                min_y = std::min(min_y, corner.y);
                                max_y = std::max(max_y, corner.y);
                            }

                            // Estimate pose for individual fiducial marker
                            // Use a standard marker size of 0.05m (5cm) for pose estimation
                            float marker_size = 0.05;
                            std::vector<cv::Vec3d> marker_rvecs, marker_tvecs;
                            cv::aruco::estimatePoseSingleMarkers(
                                {corners[i]}, marker_size, camera_matrix_, dist_coeffs_,
                                marker_rvecs, marker_tvecs
                            );
                            
                            if (!marker_rvecs.empty() && !marker_tvecs.empty()) {
                                cv::Vec3d marker_rvec = marker_rvecs[0];
                                cv::Vec3d marker_tvec = marker_tvecs[0];
                                int valid_marker = 1;
                                // Create transform for individual marker
                                geometry_msgs::TransformStamped marker_transform;
                                marker_transform.header.stamp = rgb_msg->header.stamp;
                                marker_transform.header.frame_id = camera_frame_id_;
                                marker_transform.child_frame_id = platform_frame_id_ + "_marker_" + std::to_string(ids[i]);
                                marker_transform.transform.translation.x = marker_tvec[0];
                                marker_transform.transform.translation.y = marker_tvec[1];
                                marker_transform.transform.translation.z = marker_tvec[2];

                                cv::Mat marker_rot_mat;
                                cv::Rodrigues(marker_rvec, marker_rot_mat);
                                tf2::Matrix3x3 marker_tf_rot_mat(
                                    marker_rot_mat.at<double>(0,0), marker_rot_mat.at<double>(0,1), marker_rot_mat.at<double>(0,2),
                                    marker_rot_mat.at<double>(1,0), marker_rot_mat.at<double>(1,1), marker_rot_mat.at<double>(1,2),
                                    marker_rot_mat.at<double>(2,0), marker_rot_mat.at<double>(2,1), marker_rot_mat.at<double>(2,2)
                                );
                                tf2::Quaternion marker_tf_quat;
                                marker_tf_rot_mat.getRotation(marker_tf_quat);

                                marker_transform.transform.rotation.x = marker_tf_quat.x();
                                marker_transform.transform.rotation.y = marker_tf_quat.y();
                                marker_transform.transform.rotation.z = marker_tf_quat.z();
                                marker_transform.transform.rotation.w = marker_tf_quat.w();

                                tf_broadcaster_.sendTransform(marker_transform);

                                // Add to fiducial transforms
                                geometry_msgs::Pose marker_pose;
                                marker_pose.position.x = marker_tvec[0];
                                marker_pose.position.y = marker_tvec[1];
                                marker_pose.position.z = marker_tvec[2];
                                marker_pose.orientation.x = marker_tf_quat.x();
                                marker_pose.orientation.y = marker_tf_quat.y();
                                marker_pose.orientation.z = marker_tf_quat.z();
                                marker_pose.orientation.w = marker_tf_quat.w();
                                fiducial_transforms.poses.push_back(marker_pose);
                            }
                        }
                    }

                    // Publish platform vertices as tag_id 999
                    if (min_x != std::numeric_limits<float>::max()) {
                        fiducial_msgs::Fiducial platform_fiducial;
                        platform_fiducial.fiducial_id = 999;
                        platform_fiducial.x0 = min_x;
                        platform_fiducial.y0 = min_y;
                        platform_fiducial.x1 = max_x;
                        platform_fiducial.y1 = min_y;
                        platform_fiducial.x2 = max_x;
                        platform_fiducial.y2 = max_y;
                        platform_fiducial.x3 = min_x;
                        platform_fiducial.y3 = max_y;
                        vertices_msg.fiducials.push_back(platform_fiducial);
                    }

                    vertices_pub_.publish(vertices_msg);
                    fiducial_transforms_pub_.publish(fiducial_transforms);
                } else {
                    ROS_WARN_ONCE("Board is not initialized. Make sure group_definition_file parameter is set correctly.");
                }

                cv::aruco::drawDetectedMarkers(image, corners, ids);
                cv::Vec3d rvec, tvec;
                int valid = 0;
                if (board_) {
                     valid = cv::aruco::estimatePoseBoard(corners, ids, board_, camera_matrix_, dist_coeffs_, rvec, tvec);
                }

                if (valid > 0) {
                    cv::aruco::drawAxis(image, camera_matrix_, dist_coeffs_, rvec, tvec, 0.1);
                    
                    // Publish platform transform with ID 999
                    geometry_msgs::TransformStamped transformStamped;
                    transformStamped.header.stamp = rgb_msg->header.stamp;
                    transformStamped.header.frame_id = camera_frame_id_;
                    transformStamped.child_frame_id = platform_frame_id_ + "_platform_999";
                    transformStamped.transform.translation.x = tvec[0];
                    transformStamped.transform.translation.y = tvec[1];
                    transformStamped.transform.translation.z = tvec[2];
                    
                    cv::Mat rot_mat;
                    cv::Rodrigues(rvec, rot_mat);
                    tf2::Matrix3x3 tf_rot_mat(rot_mat.at<double>(0,0), rot_mat.at<double>(0,1), rot_mat.at<double>(0,2),
                                             rot_mat.at<double>(1,0), rot_mat.at<double>(1,1), rot_mat.at<double>(1,2),
                                             rot_mat.at<double>(2,0), rot_mat.at<double>(2,1), rot_mat.at<double>(2,2));
                    tf2::Quaternion tf_quat;
                    tf_rot_mat.getRotation(tf_quat);

                    transformStamped.transform.rotation.x = tf_quat.x();
                    transformStamped.transform.rotation.y = tf_quat.y();
                    transformStamped.transform.rotation.z = tf_quat.z();
                    transformStamped.transform.rotation.w = tf_quat.w();
                    
                    tf_broadcaster_.sendTransform(transformStamped);

                    geometry_msgs::PoseStamped pose_msg;
                    pose_msg.header = transformStamped.header;
                    pose_msg.pose.position.x = tvec[0];
                    pose_msg.pose.position.y = tvec[1];
                    pose_msg.pose.position.z = tvec[2];
                    pose_msg.pose.orientation = transformStamped.transform.rotation;
                    pose_pub_.publish(pose_msg);
                }
                else{
                    ROS_WARN("Could not estimate pose for the platform. Is it visible?");
                }
            }
            else{
                ROS_WARN("No markers detected in the current frame.");
            }

            cv_ptr->image = image;
            image_pub_.publish(cv_ptr->toImageMsg());

        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "group_detection_node");
    ArucoGroupDetector detector;
    ros::spin();
    return 0;
}
