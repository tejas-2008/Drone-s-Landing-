#include <algorithm>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <fiducial_msgs/Fiducial.h>
#include <fiducial_msgs/FiducialArray.h>
#include <fiducial_msgs/FiducialTransform.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <limits>
#include <map>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp> // cv::KalmanFilter
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <vector>
#include <yaml-cpp/yaml.h>

#define COLOR_GREEN "\033[1;32m"
#define COLOR_YELLOW "\033[1;33m"
#define COLOR_BLUE "\033[1;34m"
#define COLOR_PURPLE "\033[1;35m"
#define COLOR_RESET "\033[0m"

class ArucoGroupDetector {
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  message_filters::Subscriber<sensor_msgs::Image> rgb_sub_;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image,
                                    sensor_msgs::CameraInfo> *sync_;

  ros::Publisher pose_pub_;
  ros::Publisher vertices_pub_;
  ros::Publisher fiducial_transforms_pub_;
  ros::Publisher tracking_status_pub_;
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

  // ── KF tracking state ────────────────────────────────────────────────────
  // State vector: [tx, ty, tz, vx, vy, vz]  (6D)
  // Measurement:  [tx, ty, tz]               (3D) — position only
  cv::KalmanFilter kf_;
  bool kf_initialized_;
  bool rvec_valid_; // rvec is used for re-projection; only valid after 1st
                    // detection
  cv::Vec3d last_rvec_;
  ros::Time last_kf_update_time_;
  double max_tracking_duration_; // seconds before tracker gives up

  // Known physical extent of the platform in board frame [m].
  // Used by cv::projectPoints to synthesise ID 999 during track-only frames.
  float platform_half_size_;
  std::vector<cv::Point3f> platform_corners_3d_;
  // ─────────────────────────────────────────────────────────────────────────

  int getDictionaryId(const std::string &dict_name) {
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
        {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL}};
    auto it = dict_map.find(dict_name);
    if (it != dict_map.end())
      return it->second;
    ROS_WARN("Unknown dictionary '%s', using DICT_5X5_50", dict_name.c_str());
    return cv::aruco::DICT_5X5_50;
  }

  // ── KF helpers ───────────────────────────────────────────────────────────

  /**
   * Initialise Kalman filter with a constant-velocity model.
   * @param dt  Initial time step in seconds (will be updated each frame).
   */
  void initKalmanFilter(float dt) {
    kf_.init(6, 3, 0, CV_32F);

    // Constant velocity transition: pos += vel*dt
    cv::setIdentity(kf_.transitionMatrix);
    kf_.transitionMatrix.at<float>(0, 3) = dt;
    kf_.transitionMatrix.at<float>(1, 4) = dt;
    kf_.transitionMatrix.at<float>(2, 5) = dt;

    // Measurement matrix: H = [I_3 | 0_3]  (observe position only)
    kf_.measurementMatrix = cv::Mat::zeros(3, 6, CV_32F);
    kf_.measurementMatrix.at<float>(0, 0) = 1.0f;
    kf_.measurementMatrix.at<float>(1, 1) = 1.0f;
    kf_.measurementMatrix.at<float>(2, 2) = 1.0f;

    // Process noise — small for position, moderate for velocity so the filter
    // can track a platform that accelerates smoothly.
    cv::setIdentity(kf_.processNoiseCov, cv::Scalar::all(1e-4));
    kf_.processNoiseCov.at<float>(3, 3) = 5e-3f;
    kf_.processNoiseCov.at<float>(4, 4) = 5e-3f;
    kf_.processNoiseCov.at<float>(5, 5) = 5e-3f;

    // Measurement noise (pose from estimatePoseBoard, ~1-2 cm accuracy)
    cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(1e-2));

    // Initial error covariance — start uncertain
    cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(1.0));
  }

  /**
   * Update the dt-dependent entries in the KF transition matrix.
   * Must be called each frame before predict() so the state propagation
   * uses the actual inter-frame interval rather than the initialisation value.
   */
  void updateTransitionDt(float dt) {
    kf_.transitionMatrix.at<float>(0, 3) = dt;
    kf_.transitionMatrix.at<float>(1, 4) = dt;
    kf_.transitionMatrix.at<float>(2, 5) = dt;
  }

  /**
   * Update the KF with a fresh pose measurement.
   * Also caches rvec (needed for re-projection when tracking only).
   */
  void kfUpdate(const cv::Vec3d &tvec, const cv::Vec3d &rvec,
                const ros::Time &stamp) {
    float dt =
        kf_initialized_
            ? std::max(0.001f, (float)(stamp - last_kf_update_time_).toSec())
            : (1.0f / 30.0f);

    if (!kf_initialized_) {
      initKalmanFilter(dt);
      // Seed the state with the first measurement (zero initial velocity)
      kf_.statePost.at<float>(0) = (float)tvec[0];
      kf_.statePost.at<float>(1) = (float)tvec[1];
      kf_.statePost.at<float>(2) = (float)tvec[2];
      kf_initialized_ = true;
      ROS_INFO(COLOR_GREEN
               "KF tracker initialised at [%.3f, %.3f, %.3f]" COLOR_RESET,
               tvec[0], tvec[1], tvec[2]);
    }

    updateTransitionDt(dt);
    kf_.predict();

    cv::Mat meas = (cv::Mat_<float>(3, 1) << (float)tvec[0], (float)tvec[1],
                    (float)tvec[2]);
    kf_.correct(meas);

    last_rvec_ = rvec;
    rvec_valid_ = true;
    last_kf_update_time_ = stamp;
  }

  /**
   * Publish a tracking estimate using the KF prediction when estimatePoseBoard
   * fails (partial/full occlusion).  Re-projects the known physical platform
   * corners through the predicted tvec + last valid rvec to generate a
   * synthetic ID 999 fiducial in the same format as direct detection.
   *
   * Publishes nothing and sets tracking_status=false when:
   *   - No detection has ever succeeded (kf not initialised)
   *   - Elapsed time since last detection exceeds max_tracking_duration_
   */
  void publishTrackingEstimate(const std_msgs::Header &header) {
    if (!kf_initialized_ || !rvec_valid_) {
      publishTrackingStatus(false);
      return;
    }

    double elapsed = (header.stamp - last_kf_update_time_).toSec();
    if (elapsed > max_tracking_duration_) {
      ROS_WARN_THROTTLE(1.0,
                        COLOR_YELLOW
                        "Tracking timeout (%.1fs). Platform lost." COLOR_RESET,
                        elapsed);
      publishTrackingStatus(false);
      return;
    }

    // Advance the KF without a measurement.
    float dt = std::max(0.001f, (float)elapsed);
    updateTransitionDt(dt);
    cv::Mat predicted = kf_.predict();
    // No correction: accept the a-priori estimate as the new state
    kf_.statePost = kf_.statePre.clone();

    cv::Vec3d predicted_tvec(predicted.at<float>(0), predicted.at<float>(1),
                             predicted.at<float>(2));

    // Project physical platform corners (board frame) → image pixels using
    // predicted translation + last known rotation.
    std::vector<cv::Point2f> projected;
    cv::projectPoints(platform_corners_3d_, last_rvec_, predicted_tvec,
                      camera_matrix_, dist_coeffs_, projected);

    // Build ID 999 bounding box, matching the convention from direct detection:
    //   x0,y0 = min_x, min_y   (top-left in image)
    //   x1,y1 = max_x, min_y   (top-right)
    //   x2,y2 = max_x, max_y   (bottom-right)
    //   x3,y3 = min_x, max_y   (bottom-left)
    float min_x = projected[0].x, max_x = projected[0].x;
    float min_y = projected[0].y, max_y = projected[0].y;
    for (const auto &p : projected) {
      min_x = std::min(min_x, p.x);
      max_x = std::max(max_x, p.x);
      min_y = std::min(min_y, p.y);
      max_y = std::max(max_y, p.y);
    }

    fiducial_msgs::FiducialArray vertices_msg;
    vertices_msg.header = header;

    fiducial_msgs::Fiducial tracked_fid;
    tracked_fid.fiducial_id = 999;
    tracked_fid.x0 = min_x;
    tracked_fid.y0 = min_y;
    tracked_fid.x1 = max_x;
    tracked_fid.y1 = min_y;
    tracked_fid.x2 = max_x;
    tracked_fid.y2 = max_y;
    tracked_fid.x3 = min_x;
    tracked_fid.y3 = max_y;
    vertices_msg.fiducials.push_back(tracked_fid);
    vertices_pub_.publish(vertices_msg);

    publishTrackingStatus(true);
    ROS_WARN_THROTTLE(
        0.5, "Platform tracking active — %.2fs since last detection", elapsed);
  }

  void publishTrackingStatus(bool is_tracking) {
    std_msgs::Bool msg;
    msg.data = is_tracking;
    tracking_status_pub_.publish(msg);
  }

  // ─────────────────────────────────────────────────────────────────────────

public:
  ArucoGroupDetector()
      : nh_(), pnh_("~"), it_(nh_), kf_initialized_(false), rvec_valid_(false) {
    // ── Existing params ──────────────────────────────────────────────────
    pnh_.param<std::string>("camera_frame_id", camera_frame_id_,
                            "camera_rgb_optical_frame");
    pnh_.param<std::string>("aruco_dictionary", aruco_dictionary_,
                            "DICT_5X5_50");
    pnh_.param<std::string>("group_definition_file", group_definition_file_,
                            "");

    // ── NEW tracking params ──────────────────────────────────────────────
    pnh_.param<double>("max_tracking_duration", max_tracking_duration_, 2.0);
    // platform_half_size defines the outer reach of the marker layout [m].
    // From marker_groups.yaml: corner markers at ±0.6 m, half-size 0.125 m
    // → outer edge = 0.6 + 0.125 = 0.725 m from centre.
    pnh_.param<float>("platform_half_size", platform_half_size_, 0.85f);

    // 3D corners of the platform bounding box in the board frame (Z=0 plane).
    // These are projected back to image space when estimatePoseBoard fails.
    platform_corners_3d_ = {{-platform_half_size_, -platform_half_size_, 0.0f},
                            {platform_half_size_, -platform_half_size_, 0.0f},
                            {platform_half_size_, platform_half_size_, 0.0f},
                            {-platform_half_size_, platform_half_size_, 0.0f}};
    // ────────────────────────────────────────────────────────────────────

    int dict_id = getDictionaryId(aruco_dictionary_);
    dictionary_ = cv::aruco::getPredefinedDictionary(dict_id);

    // Load board from group definition yaml (unchanged)
    if (!group_definition_file_.empty()) {
      try {
        YAML::Node config = YAML::LoadFile(group_definition_file_);
        const YAML::Node &groups = config["groups"];
        if (groups && groups.IsSequence()) {
          const YAML::Node &group = groups[0];
          platform_frame_id_ = group["group_name"].as<std::string>();

          std::vector<std::vector<cv::Point3f>> board_corners;
          std::vector<int> board_ids;

          const YAML::Node &markers = group["markers"];
          if (markers && markers.IsSequence()) {
            for (const auto &marker : markers) {
              int id = marker["id"].as<int>();
              double size = marker["size"].as<double>();
              std::vector<double> position =
                  marker["position"].as<std::vector<double>>();
              if (position.size() < 3) {
                ROS_ERROR("Marker position must have 3 coords");
                continue;
              }
              float hs = size / 2.0f;
              board_corners.push_back(
                  {cv::Point3f(position[0] - hs, position[1] + hs, position[2]),
                   cv::Point3f(position[0] + hs, position[1] + hs, position[2]),
                   cv::Point3f(position[0] + hs, position[1] - hs, position[2]),
                   cv::Point3f(position[0] - hs, position[1] - hs,
                               position[2])});
              board_ids.push_back(id);
            }
          }

          if (!board_corners.empty() &&
              board_corners.size() == board_ids.size()) {
            try {
              board_ = cv::aruco::Board::create(board_corners, dictionary_,
                                                board_ids);
              ROS_INFO("Loaded %zu markers for group '%s'", board_ids.size(),
                       platform_frame_id_.c_str());
            } catch (const cv::Exception &e) {
              ROS_ERROR("Failed to create ArUco board: %s", e.what());
              board_ = nullptr;
            }
          } else {
            ROS_WARN("No valid markers found in config");
            board_ = nullptr;
          }
        }
      } catch (const YAML::Exception &e) {
        ROS_ERROR("Failed to parse group definition: %s", e.what());
      }
    } else {
      ROS_WARN("No group definition file specified");
      board_ = nullptr;
    }

    // Subscribers
    rgb_sub_.subscribe(nh_, "/camera/rgb/image_raw", 10);
    depth_sub_.subscribe(nh_, "/camera/depth/image_raw", 10);
    camera_info_sub_.subscribe(nh_, "/camera/rgb/camera_info", 10);
    sync_ = new message_filters::TimeSynchronizer<
        sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo>(
        rgb_sub_, depth_sub_, camera_info_sub_, 100);
    sync_->registerCallback(
        boost::bind(&ArucoGroupDetector::imageCallback, this, _1, _2, _3));

    // Publishers
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("platform_pose", 10);
    vertices_pub_ =
        nh_.advertise<fiducial_msgs::FiducialArray>("fiducial_vertices", 10);
    fiducial_transforms_pub_ =
        nh_.advertise<fiducial_msgs::FiducialTransformArray>(
            "fiducial_transforms", 10);
    image_pub_ = it_.advertise("fiducial_images", 10);
    tracking_status_pub_ =
        nh_.advertise<std_msgs::Bool>("platform_tracking_active", 10);

    ROS_INFO("ArucoGroupDetector initialised for group '%s' (tracking "
             "timeout=%.1fs, half_size=%.3fm)",
             platform_frame_id_.c_str(), max_tracking_duration_,
             platform_half_size_);
  }

  ~ArucoGroupDetector() {
    if (sync_)
      delete sync_;
  }

  void imageCallback(const sensor_msgs::ImageConstPtr &rgb_msg,
                     const sensor_msgs::ImageConstPtr &depth_msg,
                     const sensor_msgs::CameraInfoConstPtr &camera_info_msg) {
    try {
      // ── Camera calibration (once) ────────────────────────────────────
      if (!camera_calibrated_) {
        camera_info_ = *camera_info_msg;
        camera_matrix_ = cv::Mat(3, 3, CV_64F);
        for (int i = 0; i < 9; i++)
          camera_matrix_.at<double>(i / 3, i % 3) = camera_info_msg->K[i];

        dist_coeffs_ = cv::Mat(5, 1, CV_64F, cv::Scalar(0.0));
        if (camera_info_msg->D.size() >= 5) {
          bool all_valid = true;
          for (size_t i = 0; i < 5; i++) {
            if (std::isnan(camera_info_msg->D[i]) ||
                std::isinf(camera_info_msg->D[i])) {
              all_valid = false;
              break;
            }
            dist_coeffs_.at<double>(i) = camera_info_msg->D[i];
          }
          if (!all_valid) {
            ROS_WARN("Invalid distortion coefficients, using zeros");
            dist_coeffs_ = cv::Mat(5, 1, CV_64F, cv::Scalar(0.0));
          }
        }
        camera_calibrated_ = true;
        ROS_INFO(COLOR_BLUE "Camera calibrated" COLOR_RESET);
      }

      // ── Image prep ───────────────────────────────────────────────────
      cv_bridge::CvImagePtr cv_ptr =
          cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
      cv::Mat image = cv_ptr->image;
      cv::Mat gray;
      cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

      // ── Marker detection ─────────────────────────────────────────────
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> corners;
      cv::aruco::detectMarkers(gray, dictionary_, corners, ids);

      if (ids.size() > 0) {
        // ── Per-marker processing (vertices + individual transforms) ─
        fiducial_msgs::FiducialArray vertices_msg;
        vertices_msg.header = rgb_msg->header;
        fiducial_msgs::FiducialTransformArray fiducial_transforms;
        fiducial_transforms.header = rgb_msg->header;

        for (size_t i = 0; i < ids.size(); i++) {
          bool is_in_board = false;
          if (board_) {
            for (int board_id : board_->ids)
              if (ids[i] == board_id) {
                is_in_board = true;
                break;
              }
          }
          if (!is_in_board)
            continue;

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

          // Individual marker transform
          float msize = 0.05f;
          std::vector<cv::Vec3d> mrvecs, mtvecs;
          try {
            cv::aruco::estimatePoseSingleMarkers(corners, msize, camera_matrix_,
                                                 dist_coeffs_, mrvecs, mtvecs);
          } catch (const cv::Exception &e) {
            ROS_ERROR("estimatePoseSingleMarkers for ID %d: %s", ids[i],
                      e.what());
            continue;
          }
          if (!mrvecs.empty() && !mtvecs.empty()) {
            geometry_msgs::TransformStamped mt;
            mt.header.stamp = rgb_msg->header.stamp;
            mt.header.frame_id = camera_frame_id_;
            mt.child_frame_id =
                platform_frame_id_ + "_marker_" + std::to_string(ids[i]);
            mt.transform.translation.x = mtvecs[0][0];
            mt.transform.translation.y = mtvecs[0][1];
            mt.transform.translation.z = mtvecs[0][2];

            cv::Mat mr;
            cv::Rodrigues(mrvecs[0], mr);
            tf2::Matrix3x3 rtf(
                mr.at<double>(0, 0), mr.at<double>(0, 1), mr.at<double>(0, 2),
                mr.at<double>(1, 0), mr.at<double>(1, 1), mr.at<double>(1, 2),
                mr.at<double>(2, 0), mr.at<double>(2, 1), mr.at<double>(2, 2));
            tf2::Quaternion q;
            rtf.getRotation(q);
            mt.transform.rotation.x = q.x();
            mt.transform.rotation.y = q.y();
            mt.transform.rotation.z = q.z();
            mt.transform.rotation.w = q.w();
            tf_broadcaster_.sendTransform(mt);

            fiducial_msgs::FiducialTransform ft;
            ft.fiducial_id = ids[i];
            ft.transform = mt.transform;
            fiducial_transforms.transforms.push_back(ft);
          }
        }

        fiducial_transforms_pub_.publish(fiducial_transforms);

        // ── Board-level pose estimation ───────────────────────────────
        cv::Vec3d rvec, tvec;
        int valid = 0;
        if (board_)
          valid = cv::aruco::estimatePoseBoard(
              corners, ids, board_, camera_matrix_, dist_coeffs_, rvec, tvec);

        if (valid > 0) {
          cv::aruco::drawAxis(image, camera_matrix_, dist_coeffs_, rvec, tvec,
                              0.1);

          // TF and pose publish
          geometry_msgs::TransformStamped ts;
          ts.header.stamp = rgb_msg->header.stamp;
          ts.header.frame_id = camera_frame_id_;
          ts.child_frame_id = platform_frame_id_ + "_platform_999";
          ts.transform.translation.x = tvec[0];
          ts.transform.translation.y = tvec[1];
          ts.transform.translation.z = tvec[2];

          cv::Mat rot_mat;
          cv::Rodrigues(rvec, rot_mat);
          tf2::Matrix3x3 tf_rot(
              rot_mat.at<double>(0, 0), rot_mat.at<double>(0, 1),
              rot_mat.at<double>(0, 2), rot_mat.at<double>(1, 0),
              rot_mat.at<double>(1, 1), rot_mat.at<double>(1, 2),
              rot_mat.at<double>(2, 0), rot_mat.at<double>(2, 1),
              rot_mat.at<double>(2, 2));
          tf2::Quaternion tf_q;
          tf_rot.getRotation(tf_q);
          ts.transform.rotation.x = tf_q.x();
          ts.transform.rotation.y = tf_q.y();
          ts.transform.rotation.z = tf_q.z();
          ts.transform.rotation.w = tf_q.w();
          tf_broadcaster_.sendTransform(ts);

          geometry_msgs::PoseStamped pose_msg;
          pose_msg.header = ts.header;
          pose_msg.pose.position.x = tvec[0];
          pose_msg.pose.position.y = tvec[1];
          pose_msg.pose.position.z = tvec[2];
          pose_msg.pose.orientation = ts.transform.rotation;
          pose_pub_.publish(pose_msg);

          // ── KF update with confirmed pose measurement ─────────────
          kfUpdate(tvec, rvec, rgb_msg->header.stamp);
          publishTrackingStatus(false); // direct detection, not tracking
          // ─────────────────────────────────────────────────────────

          // ── Project true platform corners → ID 999 vertices ────────
          // Use the board pose (rvec, tvec) and the known 3D platform
          // corners to produce perspective-correct pixel coordinates.
          // These may lie outside the image when the platform is near
          // the frame edge — that is intentional.
          std::vector<cv::Point2f> projected_corners;
          cv::projectPoints(platform_corners_3d_, rvec, tvec, camera_matrix_,
                            dist_coeffs_, projected_corners);

          fiducial_msgs::Fiducial platform_fid;
          platform_fid.fiducial_id = 999;
          platform_fid.x0 = projected_corners[0].x;
          platform_fid.y0 = projected_corners[0].y;
          platform_fid.x1 = projected_corners[1].x;
          platform_fid.y1 = projected_corners[1].y;
          platform_fid.x2 = projected_corners[2].x;
          platform_fid.y2 = projected_corners[2].y;
          platform_fid.x3 = projected_corners[3].x;
          platform_fid.y3 = projected_corners[3].y;
          vertices_msg.fiducials.push_back(platform_fid);
          // ─────────────────────────────────────────────────────────

          vertices_pub_.publish(vertices_msg);

        } else {
          // Board pose failed — still publish the individual marker vertices,
          // then let the KF tracker supply the ID 999 estimate.
          vertices_pub_.publish(vertices_msg);

          ROS_WARN_THROTTLE(1.0,
                            "estimatePoseBoard failed (%zu markers visible). "
                            "Activating tracker.",
                            ids.size());
          publishTrackingEstimate(rgb_msg->header);
        }

        cv::aruco::drawDetectedMarkers(image, corners, ids);

      } else {
        // No markers at all — fall through to tracker
        ROS_WARN_THROTTLE(2.0, "No markers detected. Activating tracker.");
        publishTrackingEstimate(rgb_msg->header);
      }

      cv_ptr->image = image;
      image_pub_.publish(cv_ptr->toImageMsg());

    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "group_detection_node");
  ArucoGroupDetector detector;
  ros::spin();
  return 0;
}