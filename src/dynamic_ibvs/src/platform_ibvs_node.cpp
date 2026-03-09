#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include "dynamic_ibvs/ibvs_controller.hpp"
#include "dynamic_ibvs/marker_detector.hpp"
#include <memory>
#include <vector>
#include <cmath>

/**
 * @file platform_ibvs_node.cpp
 * @brief Main ROS node for Platform IBVS control
 * 
 * Integrates marker detection, IBVS control law computation, and
 * platform motion control to achieve marker-based visual servoing
 * for a mobile robot platform.
 */

class PlatformIBVSNode {
private:
    ros::NodeHandle nh_;
    std::unique_ptr<IBVSController> ibvs_controller_;
    std::unique_ptr<MarkerDetector> marker_detector_;

    ros::Timer control_timer_;
    ros::Publisher velocity_pub_;
    
    // Configuration parameters
    int tracking_marker_id_;
    float marker_size_;
    float control_frequency_;
    float error_threshold_;
    float max_linear_velocity_;
    float max_angular_velocity_;
    float desired_depth_;

    // State variables
    bool initialized_;
    bool tracking_;
    int consecutive_loss_count_;
    static constexpr int MAX_CONSECUTIVE_LOSSES = 5;

public:
    PlatformIBVSNode() 
        : nh_("~"),
          tracking_marker_id_(0),
          marker_size_(0.1),
          control_frequency_(50.0),
          error_threshold_(0.05),
          max_linear_velocity_(0.5),
          max_angular_velocity_(1.0),
          desired_depth_(1.0),
          initialized_(false),
          tracking_(false),
          consecutive_loss_count_(0) {
        
        // Load parameters
        nh_.param("tracking_marker_id", tracking_marker_id_, 0);
        nh_.param("marker_size", marker_size_, 0.25f);
        nh_.param("control_frequency", control_frequency_, 50.0f);
        nh_.param("error_threshold", error_threshold_, 0.05f);
        nh_.param("max_linear_velocity", max_linear_velocity_, 0.5f);
        nh_.param("max_angular_velocity", max_angular_velocity_, 1.0f);
        nh_.param("desired_depth", desired_depth_, 1.0f);

        ROS_INFO("PlatformIBVS: Loaded parameters - marker_id: %d, marker_size: %.3f m, freq: %.1f Hz",
                tracking_marker_id_, marker_size_, control_frequency_);

        // Initialize components
        std::unique_ptr<MarkerDetector> marker_detector_{
            new MarkerDetector(&nh_)
        };
        std::unique_ptr<IBVSController> ibvs_controller_{
            new IBVSController()
        };
        // marker_detector_ = std::make_unique<MarkerDetector>(&nh_);
        // ibvs_controller_ = std::make_unique<IBVSController>();

        // Setup velocity publisher
        velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

        // Setup control loop timer
        control_timer_ = nh_.createTimer(
            ros::Duration(1.0 / control_frequency_),
            &PlatformIBVSNode::controlCallback, this);

        ROS_INFO("PlatformIBVS: Node initialized. Waiting for camera info and markers...");
    }

    /**
     * @brief Calculate pixel-space dimensions of marker from corner coordinates
     * @param marker Detected marker with corner coordinates
     * @return Pair of (width, height) in pixels
     */
    std::pair<float, float> calculateMarkerPixelSize(const std::shared_ptr<MarkerDetector::MarkerData>& marker) {
        // Calculate distances between corners
        float width = std::sqrt(std::pow(marker->x1 - marker->x0, 2) + std::pow(marker->y1 - marker->y0, 2));
        float height = std::sqrt(std::pow(marker->x3 - marker->x0, 2) + std::pow(marker->y3 - marker->y0, 2));
        
        return {width, height};
    }

    /**
     * @brief Estimate depth using camera matrix and marker corner geometry
     * Projects corners to normalized camera coordinates and uses known marker size
     * to compute accurate depth from multiple corner pair constraints
     * 
     * @param marker Detected marker
     * @param camera_fx Camera focal length in x
     * @param camera_fy Camera focal length in y
     * @param camera_u0 Camera principal point u
     * @param camera_v0 Camera principal point v
     * @return Estimated depth in meters
     */
    float estimateDepthFromCornersAndMatrix(
            const std::shared_ptr<MarkerDetector::MarkerData>& marker,
            float camera_fx, float camera_fy,
            float camera_u0, float camera_v0) {
        
        // Normalize pixel coordinates using camera matrix (inverse calibration)
        // Normalized coordinates: x_n = (u - u0) / fx, y_n = (v - v0) / fy
        float x0_norm = (marker->x0 - camera_u0) / camera_fx;
        float y0_norm = (marker->y0 - camera_v0) / camera_fy;
        float x1_norm = (marker->x1 - camera_u0) / camera_fx;
        float y1_norm = (marker->y1 - camera_v0) / camera_fy;
        float x2_norm = (marker->x2 - camera_u0) / camera_fx;
        float y2_norm = (marker->y2 - camera_v0) / camera_fy;
        float x3_norm = (marker->x3 - camera_u0) / camera_fx;
        float y3_norm = (marker->y3 - camera_v0) / camera_fy;
        
        // Calculate normalized distances between corners (actual marker has known size)
        // Distance between corner 0 and 1 in normalized image plane
        float d01_norm = std::sqrt(x1_norm*x1_norm + y1_norm*y1_norm - x0_norm*x0_norm - y0_norm*y0_norm 
                                   + 2*(x0_norm*x1_norm + y0_norm*y1_norm));
        // Better calculation: (p1 - p0) . (p1 - p0)
        float dx01_norm = x1_norm - x0_norm;
        float dy01_norm = y1_norm - y0_norm;
        float d01_norm_proper = std::sqrt(dx01_norm*dx01_norm + dy01_norm*dy01_norm);
        
        // Distance between corner 3 and 0 (the other side)
        float dx30_norm = x0_norm - x3_norm;
        float dy30_norm = y0_norm - y3_norm;
        float d30_norm = std::sqrt(dx30_norm*dx30_norm + dy30_norm*dy30_norm);
        
        // Distance between corner 1 and 2 (opposite corner pair)
        float dx12_norm = x2_norm - x1_norm;
        float dy12_norm = y2_norm - y1_norm;
        float d12_norm = std::sqrt(dx12_norm*dx12_norm + dy12_norm*dy12_norm);
        
        // Calculate average normalized distance
        float avg_d_norm = (d01_norm_proper + d30_norm + d12_norm) / 3.0f;
        
        if (avg_d_norm < 1e-6) {
            ROS_WARN("PlatformIBVS: Invalid normalized marker distance. Using desired_depth.");
            return desired_depth_;
        }
        
        // For a square marker at distance Z:
        // The normalized image coordinate difference between corner pairs relates to marker size
        // At distance Z, physical size S appears with normalized angular size: S/Z
        // Since normalized coords are already divided by focal length, the relationship is:
        // depth = (physical_marker_size) / (normalized_angular_distance)
        
        // For best estimate, use the average normalized distance
        float estimated_depth = marker_size_ / avg_d_norm;
        
        // Sanity check - depth should be positive and reasonable (0.1m to 10m)
        if (estimated_depth < 0.1f || estimated_depth > 10.0f) {
            ROS_WARN_THROTTLE(1.0, "PlatformIBVS: Estimated depth %.3f m out of bounds. Clamping.", estimated_depth);
            estimated_depth = std::max(0.1f, std::min(10.0f, estimated_depth));
        }
        
        return estimated_depth;
    }

    /**
     * @brief Estimate depth using camera matrix and marker size
     * Uses the principle: depth = (focal_length * actual_marker_size) / pixel_marker_size
     * 
     * @param marker Detected marker
     * @param camera_fx Camera focal length in x
     * @return Estimated depth in meters
     */
    float estimateDepthFromMarker(const std::shared_ptr<MarkerDetector::MarkerData>& marker, float camera_fx) {
        auto [pixel_width, pixel_height] = calculateMarkerPixelSize(marker);
        
        // Use average of width and height for robustness
        float pixel_size = (pixel_width + pixel_height) / 2.0f;
        
        if (pixel_size < 1e-6) {
            ROS_WARN("PlatformIBVS: Invalid marker size in pixels. Using desired_depth.");
            return desired_depth_;
        }
        
        // depth = (focal_length * actual_marker_size) / pixel_marker_size
        float estimated_depth = (camera_fx * marker_size_) / pixel_size;
        
        // Sanity check - depth should be positive and reasonable (0.1m to 10m)
        if (estimated_depth < 0.1f || estimated_depth > 10.0f) {
            ROS_WARN_THROTTLE(1.0, "PlatformIBVS: Estimated depth %.3f m out of bounds. Clamping.", estimated_depth);
            estimated_depth = std::max(0.1f, std::min(10.0f, estimated_depth));
        }
        
        return estimated_depth;
    }

    /**
     * @brief Control loop callback - main IBVS computation
     */
    void controlCallback(const ros::TimerEvent& event) {
        // Wait for initial setup
        if (!marker_detector_->hasCameraInfo()) {
            ROS_WARN_THROTTLE(1.0, "PlatformIBVS: Waiting for camera info...");
            return;
        }

        if (!initialized_) {
            initializeController();
            return;
        }

        // Check for marker detection
        auto marker = marker_detector_->getMarkerById(tracking_marker_id_);
        
        if (!marker) {
            consecutive_loss_count_++;
            if (consecutive_loss_count_ > MAX_CONSECUTIVE_LOSSES) {
                stopMotion();
                tracking_ = false;
                ROS_WARN("PlatformIBVS: Marker lost. Stopping motion.");
            }
            return;
        }

        consecutive_loss_count_ = 0;

        // Prepare visual features from marker corners
        IBVSController::VisualFeatures current_features;
        current_features.x = {marker->x0, marker->x1, marker->x2, marker->x3};
        current_features.y = {marker->y0, marker->y1, marker->y2, marker->y3};

        // Estimate actual depth from marker size and camera matrix
        float current_depth = estimateDepthFromCornersAndMatrix(
            marker, 
            marker_detector_->getCameraFx(),
            marker_detector_->getCameraFy(),
            marker_detector_->getCameraU0(),
            marker_detector_->getCameraV0());
        
        ROS_DEBUG("PlatformIBVS: Estimated depth: %.3f m", current_depth);

        // Compute IBVS control law using estimated depth
        auto ctrl_output = ibvs_controller_->computeControlLaw(
            current_features, current_depth);

        ROS_DEBUG("PlatformIBVS: Error norm: %.4f, Ctrl: [%.3f, %.3f, %.3f, %.3f]",
                 ctrl_output.error_norm,
                 ctrl_output.v_x, ctrl_output.v_y, ctrl_output.v_z, ctrl_output.omega_yaw);

        // Check convergence
        if (ctrl_output.error_norm < error_threshold_) {
            if (!tracking_) {
                tracking_ = true;
                ROS_INFO("PlatformIBVS: Marker acquired! Tracking initiated.");
            }
        }

        // Saturate velocities
        float v_x = saturateVelocity(ctrl_output.v_x, max_linear_velocity_);
        float v_y = saturateVelocity(ctrl_output.v_y, max_linear_velocity_);
        float v_z = saturateVelocity(ctrl_output.v_z, max_linear_velocity_);
        float omega = saturateVelocity(ctrl_output.omega_yaw, max_angular_velocity_);

        // Send command to platform
        sendVelocityCommand(v_x, v_y, v_z, omega);
    }

    /**
     * @brief Send velocity command to platform
     */
    void sendVelocityCommand(float v_x, float v_y, float v_z, float omega_yaw) {
        geometry_msgs::Twist vel_cmd;
        
        vel_cmd.linear.x = v_x;
        vel_cmd.linear.y = v_y;
        vel_cmd.linear.z = v_z;
        
        vel_cmd.angular.x = 0;
        vel_cmd.angular.y = 0;
        vel_cmd.angular.z = omega_yaw;
        
        velocity_pub_.publish(vel_cmd);
        
        ROS_DEBUG("PlatformIBVS: Velocity command - vx: %.3f, vy: %.3f, vz: %.3f, omega: %.3f",
                 v_x, v_y, v_z, omega_yaw);
    }

private:
    /**
     * @brief Initialize IBVS controller with desired features
     */
    void initializeController() {
        // Set camera intrinsics
        IBVSController::CameraIntrinsics intrinsics;
        intrinsics.f_x = marker_detector_->getCameraFx();
        intrinsics.f_y = marker_detector_->getCameraFy();
        intrinsics.u_0 = marker_detector_->getCameraU0();
        intrinsics.v_0 = marker_detector_->getCameraV0();

        ibvs_controller_->setActiveCamera(intrinsics);

        // Set desired features (centered on image)
        IBVSController::VisualFeatures desired_features;
        float cx = intrinsics.u_0;
        float cy = intrinsics.v_0;
        float w = 80.0f;   // Half-width of desired marker region
        float h = 80.0f;   // Half-height of desired marker region

        desired_features.x = {cx - w, cx - w, cx + w, cx + w};
        desired_features.y = {cy + h, cy - h, cy - h, cy + h};

        ibvs_controller_->setDesiredFeatures(desired_features, desired_depth_);

        initialized_ = true;
        ROS_INFO("PlatformIBVS: Controller initialized with desired features centered at (%.0f, %.0f)",
                cx, cy);
    }

    /**
     * @brief Saturate velocity to maximum limits with direction preservation
     */
    float saturateVelocity(float velocity, float max_value) {
        if (std::abs(velocity) > max_value) {
            return (velocity > 0) ? max_value : -max_value;
        }
        return velocity;
    }

    /**
     * @brief Stop platform motion
     */
    void stopMotion() {
        sendVelocityCommand(0.0f, 0.0f, 0.0f, 0.0f);
    }
};

// Global node pointer for signal handling
// static std::unique_ptr<PlatformIBVSNode> g_node;
static std::unique_ptr<PlatformIBVSNode> g_node;
/**
 * @brief Signal handler for graceful shutdown
 */
void signalHandler(int signum) {
    ROS_INFO("Received interrupt signal %d. Shutting down gracefully...", signum);
    if (g_node) {
        g_node.reset();
    }
    ros::shutdown();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "platform_ibvs_node");

    // Register signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    ROS_INFO("Starting Platform IBVS Node...");

    try {
        g_node = std::unique_ptr<PlatformIBVSNode>(new PlatformIBVSNode());
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL("Fatal error: %s", e.what());
        return 1;
    }

    return 0;
}
