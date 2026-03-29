#include "dynamic_ibvs/ibvs_controller.hpp"
#include "dynamic_ibvs/marker_detector.hpp"
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <memory>
#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_srvs/SetBool.h>
#include <vector>

// color definitions for console output
#define COLOR_GREEN "\033[1;32m"
#define COLOR_YELLOW "\033[1;33m"
#define COLOR_BLUE "\033[1;34m"
#define COLOR_PURPLE "\033[1;35m"
#define COLOR_RESET "\033[0m"

enum class StartupState {
  WAITING_FOR_FCU,     ///< No /mavros/state yet — FCU not connected
  STREAMING_SETPOINTS, ///< Publishing zeros to prime PX4 watchdog
  REQUESTING_OFFBOARD, ///< Calling /mavros/set_mode OFFBOARD
  ARMING,              ///< Calling /mavros/cmd/arming true
  TAKING_OFF, ///< Climbing to takeoff_altitude_ at takeoff_climb_velocity_
  READY       ///< Armed + OFFBOARD at altitude — normal IBVS operation
};

class PlatformIBVSNode {
private:
  ros::NodeHandle nh_;
  std::unique_ptr<IBVSController> ibvs_controller_;
  std::unique_ptr<MarkerDetector> marker_detector_;

  ros::Timer control_timer_;
  ros::Publisher velocity_pub_;
  ros::Publisher mavros_setpoint_pub_,
      normal_error_pub_; // publishes directly to MAVROS for startup watchdog
  ros::Publisher diag_physical_error_pub_; // x_m, y_m, yaw_rad
  ros::Publisher diag_pixel_error_pub_;    // x_px, y_px, 0
  ros::Publisher diag_error_norms_pub_;    // norm_m, norm_px, norm_ibvs
  ros::Subscriber mavros_state_sub_;
  ros::Subscriber mavros_local_pos_sub_; // live altitude feedback
  ros::Subscriber tracking_status_sub_;

  ros::ServiceClient set_mode_client_;
  ros::ServiceClient arming_client_;
  ros::ServiceServer ibvs_service_; // ~/enable_ibvs — start/stop IBVS loop

  // Configuration parameters
  int tracking_marker_id_;
  bool tracking_mode_ = false;
  float marker_size_;
  float control_frequency_;
  float error_threshold_;
  float max_linear_velocity_;
  float max_angular_velocity_;
  float desired_depth_;
  bool auto_switch_to_offboard_; // if true, node will call /mavros/set_mode to
                                 // switch to OFFBOARD automatically
  float takeoff_altitude_;       // target altitude for initial climb [m]
  float takeoff_climb_velocity_; // upward velocity during takeoff [m/s]

  float lambda_min_, lambda_max_, kd_gain_;

  // State variables
  bool initialized_;
  bool tracking_;
  int consecutive_loss_count_;
  static constexpr int MAX_CONSECUTIVE_LOSSES = 5;

  // Drone flight state (from MAVROS)
  mavros_msgs::State drone_state_;
  bool drone_state_received_;
  bool was_ready_;                     // tracks previous iteration's readiness
  ros::Time last_mode_switch_attempt_; // rate-limits set_mode service calls
  ros::Time last_arm_attempt_;         // rate-limits arming service calls

  // Local position (altitude feedback for takeoff)
  float current_altitude_;  // latest Z from /mavros/local_position/pose [m]
  bool local_pos_received_; // true once first local position msg is received

  // Startup sequence
  StartupState startup_state_;      // current phase of the startup machine
  ros::Time setpoint_stream_start_; // when we started streaming setpoints
  static constexpr double SETPOINT_STREAM_DURATION = 4.0; // seconds

  // IBVS service gate
  bool ibvs_enabled_; // true only after ~/enable_ibvs is called with data=true
  static constexpr float TRACKING_VELOCITY_SCALE = 1.00f;

  // ── Touchdown / descent ─────────────────────────────────────────────────
  float descent_error_threshold_; // max error norm to allow descent
  float
      descent_velocity_; // constant descent rate [m/s] (camera-frame +Z = down)
  float landing_altitude_;           // altitude to disarm [m AGL]
  float descent_stabilization_time_; // seconds error must stay below threshold
                                     // before descending
  int central_marker_id_;       // ArUco marker ID to switch to during descent
  float central_marker_size_;   // physical size of the central marker [m]
  float descent_lateral_scale_; // scale factor for lateral/yaw velocities
                                // during descent
  bool descent_active_; // true once stabilisation passes — stays true until
                        // landing
  bool landed_; // true after disarm — prevents startup machine from re-arming
  bool error_below_threshold_timing_; // true while counting stabilisation
                                      // seconds
  ros::Time error_below_threshold_start_;
  // ─────────────────────────────────────────────────────────────────────────

public:
  PlatformIBVSNode()
      : nh_("~"), tracking_marker_id_(0), marker_size_(0.1),
        control_frequency_(50.0), error_threshold_(0.05),
        max_linear_velocity_(0.5), max_angular_velocity_(1.0),
        desired_depth_(1.0), initialized_(false), tracking_(false),
        consecutive_loss_count_(0), drone_state_received_(false),
        was_ready_(false), last_mode_switch_attempt_(ros::Time(0)),
        last_arm_attempt_(ros::Time(0)), current_altitude_(0.0f),
        local_pos_received_(false),
        startup_state_(StartupState::WAITING_FOR_FCU),
        setpoint_stream_start_(ros::Time(0)), ibvs_enabled_(false),
        descent_active_(false), landed_(false),
        error_below_threshold_timing_(false) {

    // Load parameters
    nh_.param("tracking_marker_id", tracking_marker_id_, 999);
    nh_.param("marker_size", marker_size_, 0.25f);
    nh_.param("control_frequency", control_frequency_, 50.0f);
    nh_.param("error_threshold", error_threshold_, 0.05f);
    nh_.param("max_linear_velocity", max_linear_velocity_, 0.5f);
    nh_.param("max_angular_velocity", max_angular_velocity_, 1.0f);
    nh_.param("desired_depth", desired_depth_, 1.0f);
    nh_.param("auto_switch_to_offboard", auto_switch_to_offboard_, true);
    nh_.param("takeoff_altitude", takeoff_altitude_, 2.5f);
    nh_.param("takeoff_climb_velocity", takeoff_climb_velocity_, 0.5f);

    nh_.param("lambda_min", lambda_min_, 0.25f);
    nh_.param("lambda_max", lambda_max_, 1.0f);
    nh_.param("kd_gain", kd_gain_, 0.15f);

    // Touchdown parameters
    nh_.param("descent_error_threshold", descent_error_threshold_, 0.50f);
    nh_.param("descent_velocity", descent_velocity_, 0.3f);
    nh_.param("landing_altitude", landing_altitude_, 0.5f);
    nh_.param("descent_stabilization_time", descent_stabilization_time_, 2.0f);
    nh_.param("central_marker_id", central_marker_id_, 0);
    nh_.param("central_marker_size", central_marker_size_, 0.5f);
    nh_.param("descent_lateral_scale", descent_lateral_scale_, 0.3f);

    ROS_INFO("PlatformIBVS: Loaded parameters - marker_id: %d, marker_size: "
             "%.3f m, freq: %.1f Hz",
             tracking_marker_id_, marker_size_, control_frequency_);
    ROS_INFO("PlatformIBVS: Touchdown params - descent_vel: %.2f m/s, "
             "landing_alt: %.2f m, err_thresh: %.3f, stab_time: %.1f s, "
             "central_id: %d (%.3f m)",
             descent_velocity_, landing_altitude_, descent_error_threshold_,
             descent_stabilization_time_, central_marker_id_,
             central_marker_size_);

    // Initialize components
    marker_detector_ = std::make_unique<MarkerDetector>(&nh_);
    ibvs_controller_ = std::make_unique<IBVSController>();

    ibvs_controller_->setGainLimits(lambda_min_, lambda_max_, kd_gain_);

    // Subscribe to MAVROS state for drone readiness
    ros::NodeHandle global_nh, global_nh2;
    mavros_state_sub_ = global_nh.subscribe(
        "/mavros/state", 1, &PlatformIBVSNode::mavrosStateCallback, this);
    tracking_status_sub_ =
        global_nh.subscribe("/platform_tracking_active", 1,
                            &PlatformIBVSNode::trackingStatusCallback, this);
    mavros_local_pos_sub_ =
        global_nh.subscribe("/mavros/local_position/pose", 10,
                            &PlatformIBVSNode::localPositionCallback, this);
    set_mode_client_ = global_nh.serviceClient<mavros_msgs::SetMode>(
        "/mavros/set_mode", /* persistent= */ true);
    arming_client_ = global_nh.serviceClient<mavros_msgs::CommandBool>(
        "/mavros/cmd/arming", /* persistent= */ true);
    mavros_setpoint_pub_ = global_nh2.advertise<geometry_msgs::Twist>(
        "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    normal_error_pub_ =
        global_nh.advertise<std_msgs::Float32>("feature_error", 10);
    // Fail fast if MAVROS is not running rather than silently dropping calls
    ROS_INFO("PlatformIBVS: Waiting for MAVROS services...");
    set_mode_client_.waitForExistence();
    arming_client_.waitForExistence();
    ROS_INFO("PlatformIBVS: MAVROS services found.");

    // Setup velocity publisher
    velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    // Diagnostics publishers
    diag_physical_error_pub_ =
        nh_.advertise<geometry_msgs::Vector3>("diagnostics/physical_error", 10);
    diag_pixel_error_pub_ =
        nh_.advertise<geometry_msgs::Vector3>("diagnostics/pixel_error", 10);
    diag_error_norms_pub_ =
        nh_.advertise<geometry_msgs::Vector3>("diagnostics/error_norms", 10);

    // Setup control loop timer
    control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_frequency_),
                                     &PlatformIBVSNode::controlCallback, this);
    ibvs_service_ = nh_.advertiseService(
        "enable_ibvs", &PlatformIBVSNode::enableIBVSCallback, this);

    ROS_INFO("PlatformIBVS: Node initialized. Auto-takeoff is enabled.");
  }

  void trackingStatusCallback(const std_msgs::Bool::ConstPtr &msg) {
    if (msg->data != tracking_mode_) {
      tracking_mode_ = msg->data;
      if (tracking_mode_)
        ROS_WARN("PlatformIBVS: Switched to TRACKER data (velocities scaled "
                 "%.0f%%).",
                 TRACKING_VELOCITY_SCALE * 100.0f);
      else
        ROS_INFO("PlatformIBVS: Platform re-detected. Back to direct control.");
    }
  }

  bool enableIBVSCallback(std_srvs::SetBool::Request &req,
                          std_srvs::SetBool::Response &res) {
    if (req.data && !ibvs_enabled_) {
      ibvs_enabled_ = true;
      ROS_INFO("PlatformIBVS: IBVS enabled.");
    } else if (!req.data && ibvs_enabled_) {
      ibvs_enabled_ = false;
      ROS_INFO("PlatformIBVS: IBVS disabled.");
    } else {
      res.success = false;
      return false;
    }
    res.success = true;
    return true;
  }

  /**
   * @brief Estimate depth using camera matrix and marker corner geometry
   * Projects corners to normalized camera coordinates and uses known marker
   * size to compute accurate depth from multiple corner pair constraints
   *
   * @param marker Detected marker
   * @param camera_fx Camera focal length in x
   * @param camera_fy Camera focal length in y
   * @param camera_u0 Camera principal point u
   * @param camera_v0 Camera principal point v
   * @return Estimated depth in meters
   */
  float estimateDepthFromCornersAndMatrix(
      const std::shared_ptr<MarkerDetector::MarkerData> &marker,
      float camera_fx, float camera_fy, float camera_u0, float camera_v0,
      float override_marker_size = -1.0f) {

    float effective_size =
        (override_marker_size > 0.0f) ? override_marker_size : marker_size_;

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

    // Calculate normalized distances between corners (actual marker has known
    // size) Distance between corner 0 and 1 in normalized image plane
    float d01_norm = std::sqrt(x1_norm * x1_norm + y1_norm * y1_norm -
                               x0_norm * x0_norm - y0_norm * y0_norm +
                               2 * (x0_norm * x1_norm + y0_norm * y1_norm));
    // Better calculation: (p1 - p0) . (p1 - p0)
    float dx01_norm = x1_norm - x0_norm;
    float dy01_norm = y1_norm - y0_norm;
    float d01_norm_proper =
        std::sqrt(dx01_norm * dx01_norm + dy01_norm * dy01_norm);

    // Distance between corner 3 and 0 (the other side)
    float dx30_norm = x0_norm - x3_norm;
    float dy30_norm = y0_norm - y3_norm;
    float d30_norm = std::sqrt(dx30_norm * dx30_norm + dy30_norm * dy30_norm);

    // Distance between corner 1 and 2 (opposite corner pair)
    float dx12_norm = x2_norm - x1_norm;
    float dy12_norm = y2_norm - y1_norm;
    float d12_norm = std::sqrt(dx12_norm * dx12_norm + dy12_norm * dy12_norm);

    // Calculate average normalized distance
    float avg_d_norm = (d01_norm_proper + d30_norm + d12_norm) / 3.0f;

    if (avg_d_norm < 1e-6) {
      ROS_WARN("PlatformIBVS: Invalid normalized marker distance. Using "
               "desired_depth.");
      return desired_depth_;
    }

    // For best estimate, use the average normalized distance
    float estimated_depth = effective_size / avg_d_norm;

    // Sanity check - depth should be positive and reasonable (0.1m to 10m)
    if (estimated_depth < 0.1f || estimated_depth > 10.0f) {
      ROS_WARN_THROTTLE(
          1.0, "PlatformIBVS: Estimated depth %.3f m out of bounds. Clamping.",
          estimated_depth);
      estimated_depth = std::max(0.1f, std::min(10.0f, estimated_depth));
    }

    return estimated_depth;
  }

  void controlCallback(const ros::TimerEvent &) {
    if (landed_)
      return; // Mission complete — no further action

    if (!runStartupSequence())
      return;
    if (!checkReadiness())
      return;

    if (!ibvs_enabled_) {
      publishHoldSetpoint();
      ROS_INFO_THROTTLE(
          5.0,
          "PlatformIBVS: Hovering at %.2f m. Call '~enable_ibvs' (data=true) "
          "to start IBVS.",
          current_altitude_);
      return;
    }
    runIBVS();
  }

  /**
   * @brief Publish a zero-velocity setpoint directly to MAVROS.
   *
   * This feeds PX4's OFFBOARD setpoint watchdog during startup so PX4 is
   * already receiving setpoints before we ask it to switch to OFFBOARD mode.
   * It uses mavros_setpoint_pub_ (→
   * /mavros/setpoint_velocity/cmd_vel_unstamped) rather than velocity_pub_ (→
   * ~/cmd_vel) which only reaches downstream controllers and is invisible to
   * the PX4 watchdog.
   */
  void publishHoldSetpoint() {
    geometry_msgs::Twist zero;
    mavros_setpoint_pub_.publish(zero);
  }

  bool runStartupSequence() {
    if (startup_state_ == StartupState::READY) {
      return true;
    }

    switch (startup_state_) {

    case StartupState::WAITING_FOR_FCU:
      // Don't stream setpoints yet — FCU isn't connected so they'd be dropped.
      if (!drone_state_received_ || !drone_state_.connected) {
        ROS_INFO_THROTTLE(3.0,
                          "PlatformIBVS: [1/4] Waiting for FCU connection...");
        break;
      }
      ROS_INFO("PlatformIBVS: [1/4] FCU connected. Starting setpoint stream.");
      setpoint_stream_start_ = ros::Time::now();
      startup_state_ = StartupState::STREAMING_SETPOINTS;
      break;

    case StartupState::STREAMING_SETPOINTS: {
      // Actively stream zero-velocity directly into MAVROS so PX4's OFFBOARD
      // watchdog sees a steady setpoint flow at our control rate (50 Hz).  PX4
      // refuses to enter OFFBOARD if fewer than 2 messages/s have arrived.
      publishHoldSetpoint();

      double elapsed = (ros::Time::now() - setpoint_stream_start_).toSec();
      ROS_INFO_THROTTLE(
          0.5,
          "PlatformIBVS: [2/4] Streaming hold setpoints to MAVROS "
          "(%.1f / %.1f s)...",
          elapsed, SETPOINT_STREAM_DURATION);
      if (elapsed >= SETPOINT_STREAM_DURATION) {
        ROS_INFO("PlatformIBVS: [2/4] Watchdog satisfied. Requesting OFFBOARD "
                 "mode.");
        startup_state_ = StartupState::REQUESTING_OFFBOARD;
      }
      break;
    }

    case StartupState::REQUESTING_OFFBOARD:
      // Keep streaming so the watchdog doesn't expire while we wait for the
      // mode-switch ACK from PX4 (can take up to 3 s with SITL jitter).
      publishHoldSetpoint();
      if (drone_state_.mode == "OFFBOARD") {
        ROS_INFO("PlatformIBVS: [3/4] OFFBOARD confirmed. Arming...");
        startup_state_ = StartupState::ARMING;
        break;
      }
      requestOffboardMode();
      break;

    case StartupState::ARMING:
      publishHoldSetpoint();
      if (drone_state_.armed) {
        ROS_INFO("PlatformIBVS: [4/5] Armed. Starting takeoff to %.1f m.",
                 takeoff_altitude_);
        startup_state_ = StartupState::TAKING_OFF;
        break;
      }
      requestArming();
      break;

    case StartupState::TAKING_OFF: {
      // Climb by publishing upward velocity directly into MAVROS.
      // The PX4 position controller handles smooth deceleration as we near
      // the target; we simply stop commanding climb once altitude is reached.
      if (!local_pos_received_) {
        ROS_WARN_THROTTLE(2.0,
                          "PlatformIBVS: [5/5] Waiting for local position...");
        publishHoldSetpoint();
        break;
      }

      const float altitude_tolerance = 0.15f; // ±15 cm
      if (current_altitude_ >= takeoff_altitude_ - altitude_tolerance) {
        ROS_INFO(
            "PlatformIBVS: [5/5] Reached %.2f m (target %.1f m). IBVS ready.",
            current_altitude_, takeoff_altitude_);
        startup_state_ = StartupState::READY;
        publishHoldSetpoint();
        break;
      }

      // Publish climb setpoint
      geometry_msgs::Twist climb;
      climb.linear.z = takeoff_climb_velocity_;
      mavros_setpoint_pub_.publish(climb);
      ROS_INFO_THROTTLE(
          1.0, "PlatformIBVS: [5/5] Taking off... altitude=%.2f m / %.1f m",
          current_altitude_, takeoff_altitude_);
      break;
    }

    case StartupState::READY:
      break; // handled by the early return above
    }

    return false; // still starting up
  }

  bool checkReadiness() {
    const bool ready_now = isDroneReady();

    if (was_ready_ && !ready_now) {
      ROS_WARN(
          "PlatformIBVS: Lost readiness (armed=%s, mode=%s). Recovering...",
          drone_state_.armed ? "true" : "false", drone_state_.mode.c_str());
      resetController();
      startup_state_ = StartupState::REQUESTING_OFFBOARD;
    } else if (!was_ready_ && ready_now) {
      ROS_INFO("PlatformIBVS: Drone ready (mode=%s). IBVS active.",
               drone_state_.mode.c_str());
    }

    was_ready_ = ready_now;

    if (!ready_now) {
      // Keep the PX4 watchdog alive even while recovering — if setpoints stop,
      // PX4 cannot re-enter OFFBOARD when we request it again.
      publishHoldSetpoint();
    }

    return ready_now;
  }

  /**
   * @brief Execute one tick of the IBVS control loop.
   *
   * Prerequisites (enforced by callers before this is invoked):
   *   - Drone is armed and in OFFBOARD mode.
   *   - Startup sequence is complete.
   */
  void runIBVS() {
    // keep PX4 watchdog alive
    if (!marker_detector_->hasCameraInfo()) {
      ROS_WARN_THROTTLE(1.0, "PlatformIBVS: Waiting for camera info...");
      publishHoldSetpoint();
      return;
    }
    if (!initialized_) {
      initializeController();
      publishHoldSetpoint();
      return;
    }

    // During descent, switch to the central marker for more reliable tracking
    // as corner markers leave the camera FOV.
    int active_marker_id =
        descent_active_ ? central_marker_id_ : tracking_marker_id_;
    float active_marker_size = descent_active_ ? central_marker_size_ : -1.0f;

    auto marker = marker_detector_->getMarkerById(active_marker_id);

    if (!marker) {
      if (++consecutive_loss_count_ > MAX_CONSECUTIVE_LOSSES) {
        tracking_ = false;
        ROS_WARN_THROTTLE(1.0,
                          "PlatformIBVS: Marker %d lost. Holding position.",
                          active_marker_id);
      }
      // Always send a hold setpoint — even a single missed tick can starve
      // PX4's watchdog if the marker has been lost for several frames.
      publishHoldSetpoint();
      return;
    }

    consecutive_loss_count_ = 0;

    IBVSController::VisualFeatures features;
    features.x = {marker->x0, marker->x1, marker->x2, marker->x3};
    features.y = {marker->y0, marker->y1, marker->y2, marker->y3};

    float depth = estimateDepthFromCornersAndMatrix(
        marker, marker_detector_->getCameraFx(),
        marker_detector_->getCameraFy(), marker_detector_->getCameraU0(),
        marker_detector_->getCameraV0(), active_marker_size);

    ROS_INFO("{PlatformIBVS}: depth=%.4f (marker=%d)", depth, active_marker_id);

    auto ctrl = ibvs_controller_->computeControlLaw(features, depth);

    // ROS_INFO_THROTTLE(1.0, "PlatformIBVS: err=%.4f  vx=%.3f vy=%.3f vz=%.3f
    // ω=%.3f",
    //   ctrl.error_norm, ctrl.v_x, ctrl.v_y, ctrl.v_z, ctrl.omega_yaw);
    std_msgs::Float32 error_msg;
    error_msg.data = ctrl.error_norm;
    normal_error_pub_.publish(error_msg);

    // ── Diagnostics: physical, pixel, and norm errors ─────────────────────
    publishDiagnostics(depth);

    if (ctrl.error_norm < error_threshold_ && !tracking_) {
      tracking_ = true;
      ROS_INFO("{PlatformIBVS}: Marker acquired. Tracking initiated.");
    }

    // ─────────────────────── Touchdown logic ─────────────────────────────
    updateDescentState(ctrl.error_norm);

    float cmd_vz = saturateVelocity(ctrl.v_z, max_linear_velocity_);

    if (descent_active_) {
      // Check for landing first
      if (local_pos_received_ && current_altitude_ <= landing_altitude_) {
        executeLanding();
        return;
      }

      if (ctrl.error_norm < descent_error_threshold_) {
        // Error is low — descend toward the platform
        cmd_vz = descent_velocity_;
        ROS_INFO_THROTTLE(1.0,
                          COLOR_GREEN "TOUCHDOWN: Descending at %.2f m/s | "
                                      "alt=%.2f m | err=%.4f" COLOR_RESET,
                          descent_velocity_, current_altitude_,
                          ctrl.error_norm);
      } else {
        // Error too high — pause descent, let IBVS re-centre
        ROS_WARN_THROTTLE(
            1.0, "TOUCHDOWN: Descent paused — err %.4f > %.4f. Re-centering.",
            ctrl.error_norm, descent_error_threshold_);
      }
    }

    // Compute final velocity commands
    float cmd_vx = saturateVelocity(ctrl.v_x, max_linear_velocity_);
    float cmd_vy = saturateVelocity(ctrl.v_y, max_linear_velocity_);
    float cmd_omega = saturateVelocity(ctrl.omega_yaw, max_angular_velocity_);

    // During descent, scale down lateral/yaw corrections to prevent
    // oscillation caused by marker proximity (large pixel shifts → jitter)
    if (descent_active_) {
      cmd_vx *= descent_lateral_scale_;
      cmd_vy *= descent_lateral_scale_;
      cmd_omega *= descent_lateral_scale_;
    }

    sendVelocityCommand(cmd_vx, cmd_vy, cmd_vz, cmd_omega);
  }

  /**
   * @brief Send velocity command to platform and to MAVROS.
   *
   * Publishes to both:
   *   - velocity_pub_        (~/cmd_vel)                           → downstream
   * controllers / visualization
   *   - mavros_setpoint_pub_ (/mavros/setpoint_velocity/cmd_vel_unstamped) →
   * PX4 watchdog
   *
   * Publishing every IBVS command to MAVROS directly ensures PX4's OFFBOARD
   * setpoint watchdog is always satisfied, preventing mode fallback to POSCTL.
   */
  void sendVelocityCommand(float v_x, float v_y, float v_z, float omega_yaw) {
    if (tracking_mode_) {
      v_x *= TRACKING_VELOCITY_SCALE;
      v_y *= TRACKING_VELOCITY_SCALE;
      v_z *= TRACKING_VELOCITY_SCALE;
      omega_yaw *= TRACKING_VELOCITY_SCALE;
    }

    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = -v_y;
    vel_cmd.linear.y = -v_x;
    vel_cmd.linear.z = -v_z;
    vel_cmd.angular.x = 0;
    vel_cmd.angular.y = 0;
    vel_cmd.angular.z = -omega_yaw;

    velocity_pub_.publish(vel_cmd);        // local/downstream topic
    mavros_setpoint_pub_.publish(vel_cmd); // directly into PX4 watchdog

    ROS_INFO_THROTTLE(
        1.0,
        COLOR_PURPLE
        "PlatformIBVS: cmd vx=%.3f vy=%.3f vz=%.3f ω=%.3f" COLOR_RESET,
        -v_y, -v_x, -v_z, -omega_yaw);
  }

private:
  /**
   * @brief Callback to cache the latest MAVROS flight state
   */
  void mavrosStateCallback(const mavros_msgs::State::ConstPtr &msg) {
    drone_state_ = *msg;
    if (!drone_state_received_) {
      drone_state_received_ = true;
      ROS_INFO("PlatformIBVS: MAVROS state received - armed: %s, mode: %s",
               drone_state_.armed ? "true" : "false",
               drone_state_.mode.c_str());
    }
  }

  /**
   * @brief Cache current altitude from local position estimate.
   *
   * The Z axis of /mavros/local_position/pose is up-positive in the
   * world/NED-converted frame, matching what PX4 uses for altitude setpoints.
   */
  void localPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    current_altitude_ = msg->pose.position.z;
    if (!local_pos_received_) {
      local_pos_received_ = true;
      ROS_INFO(
          "PlatformIBVS: Local position received. Current altitude: %.2f m",
          current_altitude_);
    }
  }

  /**
   * @brief Request a switch to OFFBOARD mode via MAVROS.
   *        Rate-limited to once every 3 s to avoid hammering the FCU.
   */
  void requestOffboardMode() {
    if (!auto_switch_to_offboard_) {
      ROS_WARN_THROTTLE(
          2.0, "PlatformIBVS: auto_switch_to_offboard is false. "
               "Switch to OFFBOARD manually or set the param to true.");
      return;
    }

    const ros::Duration SWITCH_COOLDOWN(3.0);
    if ((ros::Time::now() - last_mode_switch_attempt_) < SWITCH_COOLDOWN) {
      ROS_INFO_THROTTLE(
          1.0,
          "PlatformIBVS: [3/4] Waiting for OFFBOARD mode (current: '%s')...",
          drone_state_.mode.c_str());
      return;
    }
    last_mode_switch_attempt_ = ros::Time::now();

    ROS_INFO("PlatformIBVS: [3/4] Requesting OFFBOARD mode (current: '%s')...",
             drone_state_.mode.c_str());

    mavros_msgs::SetMode srv;
    srv.request.custom_mode = "OFFBOARD";

    if (set_mode_client_.call(srv)) {
      if (srv.response.mode_sent) {
        ROS_INFO("PlatformIBVS: OFFBOARD mode request accepted by FCU.");
      } else {
        ROS_WARN("PlatformIBVS: SetMode called but mode_sent=false. "
                 "FCU rejected — setpoint stream may not be ready yet.");
      }
    } else {
      ROS_ERROR("PlatformIBVS: /mavros/set_mode service call failed. Is MAVROS "
                "running?");
    }
  }

  /**
   * @brief Arm the drone via MAVROS.
   *        Rate-limited to once every 3 s to avoid hammering the FCU.
   */
  void requestArming() {
    const ros::Duration ARM_COOLDOWN(3.0);
    if ((ros::Time::now() - last_arm_attempt_) < ARM_COOLDOWN) {
      ROS_INFO_THROTTLE(
          1.0, "PlatformIBVS: [4/4] Waiting for arming confirmation...");
      return;
    }
    last_arm_attempt_ = ros::Time::now();

    ROS_INFO("PlatformIBVS: [4/4] Sending arm command...");

    mavros_msgs::CommandBool srv;
    srv.request.value = true;

    if (arming_client_.call(srv)) {
      if (srv.response.success) {
        ROS_INFO("PlatformIBVS: Arm command accepted by FCU.");
      } else {
        ROS_WARN("PlatformIBVS: Arm command rejected by FCU. "
                 "Check pre-arm checks (GPS, RC, etc.).");
      }
    } else {
      ROS_ERROR("PlatformIBVS: /mavros/cmd/arming service call failed. Is "
                "MAVROS running?");
    }
  }

  /**
   * @brief Diagnose and log the specific reason the drone is not ready.
   *        Call this whenever isDroneReady() returns false for useful output.
   */
  void logNotReadyReason() const {
    if (!drone_state_received_) {
      ROS_WARN(
          "PlatformIBVS: No /mavros/state received. FCU may be disconnected.");
      return;
    }
    if (!drone_state_.armed) {
      ROS_WARN("PlatformIBVS: Drone is disarmed.");
      return;
    }
    ROS_WARN(
        "PlatformIBVS: Flight mode '%s' does not accept velocity commands. "
        "Switch to OFFBOARD mode (PX4).",
        drone_state_.mode.c_str());
  }

  /**
   * @brief Check whether the drone is ready to accept velocity commands.
   *
   * Conditions:
   *   1. A MAVROS state message has been received (FCU is connected).
   *   2. The drone is armed.
   *   3. The flight mode is one that accepts external velocity setpoints
   *      (GUIDED for ArduPilot, OFFBOARD for PX4).
   *
   * @return true if all conditions are met, false otherwise.
   */
  bool isDroneReady() const {
    if (!drone_state_received_) {
      return false; // No state from FCU yet
    }
    if (!drone_state_.armed) {
      return false; // Motors are off
    }
    const std::string &mode = drone_state_.mode;
    // PX4 mode that accepts external velocity setpoints
    bool mode_ok = (mode == "OFFBOARD");
    return mode_ok;
  }

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
    float w = 80.0f; // Half-width of desired marker region
    float h = 80.0f; // Half-height of desired marker region

    desired_features.x = {cx - w, cx - w, cx + w, cx + w};
    desired_features.y = {cy + h, cy - h, cy - h, cy + h};

    ibvs_controller_->setDesiredFeatures(desired_features, desired_depth_);

    initialized_ = true;
    ROS_INFO("PlatformIBVS: Controller initialized with desired features "
             "centered at (%.0f, %.0f)",
             cx, cy);
  }

  /**
   * @brief Publish diagnostic error information for monitoring convergence.
   *
   * Publishes three Vector3 messages:
   *   - physical_error: (x_meters, y_meters, yaw_radians)
   *   - pixel_error:    (x_pixels, y_pixels, 0)
   *   - error_norms:    (norm_meters, norm_pixels, norm_ibvs_raw)
   */
  void publishDiagnostics(float depth) {
    if (!initialized_ || !marker_detector_->hasCameraInfo())
      return;

    // Centroid offset in normalised image coordinates
    float cx_err = ibvs_controller_->getCurrentCentroidX() -
                   ibvs_controller_->getDesiredCentroidX();
    float cy_err = ibvs_controller_->getCurrentCentroidY() -
                   ibvs_controller_->getDesiredCentroidY();

    // Yaw error from the IBVS error vector (element 3)
    arma::vec e = ibvs_controller_->getCurrentError();
    float yaw_err = (e.n_elem >= 4) ? (float)e(3) : 0.0f;

    // 1. Physical error (metres)
    float phys_x = depth * cx_err;
    float phys_y = depth * cy_err;

    geometry_msgs::Vector3 phys_msg;
    phys_msg.x = phys_x;
    phys_msg.y = phys_y;
    phys_msg.z = yaw_err;
    diag_physical_error_pub_.publish(phys_msg);

    // 2. Pixel error
    float px_x = cx_err * marker_detector_->getCameraFx();
    float px_y = cy_err * marker_detector_->getCameraFy();

    geometry_msgs::Vector3 px_msg;
    px_msg.x = px_x;
    px_msg.y = px_y;
    px_msg.z = 0.0;
    diag_pixel_error_pub_.publish(px_msg);

    // 3. Norms
    float norm_m = std::sqrt(phys_x * phys_x + phys_y * phys_y);
    float norm_px = std::sqrt(px_x * px_x + px_y * px_y);
    float norm_ibvs = (float)arma::norm(e, 2);

    geometry_msgs::Vector3 norm_msg;
    norm_msg.x = norm_m;
    norm_msg.y = norm_px;
    norm_msg.z = norm_ibvs;
    diag_error_norms_pub_.publish(norm_msg);
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
  void stopMotion() { sendVelocityCommand(0.0f, 0.0f, 0.0f, 0.0f); }

  /**
   * @brief Reset IBVS controller state.
   *        Called when the drone loses readiness mid-flight so the controller
   *        starts fresh on recovery. Does NOT reset the startup state machine
   *        (the caller is responsible for stepping it back appropriately).
   */
  // ── Touchdown helpers ───────────────────────────────────────────────────

  /**
   * @brief Update descent state based on current IBVS error.
   *
   * When the feature error stays below descent_error_threshold_ for
   * descent_stabilization_time_ seconds, descent_active_ is set to true.
   * Once active, descent remains enabled — pausing/resuming is handled at
   * the velocity-command level by checking the instantaneous error.
   */
  void updateDescentState(float error_norm) {
    if (descent_active_)
      return; // Already committed to descent

    if (error_norm < descent_error_threshold_) {
      if (!error_below_threshold_timing_) {
        error_below_threshold_timing_ = true;
        error_below_threshold_start_ = ros::Time::now();
        ROS_INFO(COLOR_GREEN "TOUCHDOWN: Error below threshold (%.4f < %.4f). "
                             "Stabilising for %.1f s..." COLOR_RESET,
                 error_norm, descent_error_threshold_,
                 descent_stabilization_time_);
      }

      double elapsed =
          (ros::Time::now() - error_below_threshold_start_).toSec();
      if (elapsed >= descent_stabilization_time_) {
        descent_active_ = true;
        ROS_INFO(COLOR_GREEN
                 "TOUCHDOWN: Stabilisation complete. Beginning descent "
                 "at %.2f m/s toward %.2f m." COLOR_RESET,
                 descent_velocity_, landing_altitude_);
      }
    } else {
      if (error_below_threshold_timing_) {
        error_below_threshold_timing_ = false;
        ROS_WARN("TOUCHDOWN: Error rose above threshold (%.4f > %.4f). "
                 "Stabilisation timer reset.",
                 error_norm, descent_error_threshold_);
      }
    }
  }

  /**
   * @brief Execute the final landing: stop motion, disarm, flag as landed.
   */
  void executeLanding() {
    ROS_INFO(COLOR_GREEN
             "============================================" COLOR_RESET);
    ROS_INFO(COLOR_GREEN
             "  TOUCHDOWN: Landing altitude %.2f m <= %.2f m" COLOR_RESET,
             current_altitude_, landing_altitude_);
    ROS_INFO(COLOR_GREEN "  TOUCHDOWN: Disarming drone..." COLOR_RESET);
    ROS_INFO(COLOR_GREEN
             "============================================" COLOR_RESET);

    // Stop all motion
    publishHoldSetpoint();

    // Disarm
    mavros_msgs::CommandBool srv;
    srv.request.value = false;
    if (arming_client_.call(srv) && srv.response.success) {
      ROS_INFO(COLOR_GREEN
               "TOUCHDOWN: Disarm accepted. Mission complete!" COLOR_RESET);
    } else {
      ROS_WARN("TOUCHDOWN: Disarm command failed/rejected. "
               "Drone may still be armed.");
    }

    ibvs_enabled_ = false;
    descent_active_ = false;
    landed_ = true;
  }

  // ────────────────────────────────────────────────────────────────────────

  void resetController() {
    initialized_ = false;
    tracking_ = false;
    consecutive_loss_count_ = 0;
    descent_active_ = false;
    error_below_threshold_timing_ = false;
    ibvs_controller_->reset();
    ROS_INFO("PlatformIBVS: IBVS controller reset.");
  }
};

// Global node pointer for signal handling
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

int main(int argc, char **argv) {
  ros::init(argc, argv, "platform_ibvs_node");

  // Register signal handlers
  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);

  ROS_INFO("Starting Platform IBVS Node...");

  try {
    g_node = std::unique_ptr<PlatformIBVSNode>(new PlatformIBVSNode());

    // Use a multi-threaded spinner so that blocking MAVROS service calls
    // (SetMode, CommandBool) do not starve the subscription callbacks that
    // deliver the service responses.  With a single-threaded ros::spin() the
    // timer callback blocks on set_mode_client_.call(), MAVROS cannot receive
    // the reply on the same thread, and the MAVLink socket times out with
    // "poll timeout 0, 22".  Four threads is more than enough:
    //   • control timer callback
    //   • /mavros/state subscriber
    //   • service response receiver
    //   • spare
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
  } catch (const std::exception &e) {
    ROS_FATAL("Fatal error: %s", e.what());
    return 1;
  }

  return 0;
}
