#include "dynamic_ibvs/ibvs_controller.hpp"
#include "dynamic_ibvs/marker_detector.hpp"
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <memory>
#include <ros/ros.h>
#include <signal.h>
#include <std_srvs/SetBool.h>
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
//  Startup state machine
// ─────────────────────────────────────────────────────────────────────────────
enum class StartupState
{
    WAITING_FOR_FCU,
    STREAMING_SETPOINTS,
    REQUESTING_OFFBOARD,
    ARMING,
    TAKING_OFF,
    READY
};

// ─────────────────────────────────────────────────────────────────────────────
class PlatformIBVSNode
{
private:
    ros::NodeHandle nh_;
    std::unique_ptr<IBVSController>  ibvs_controller_;
    std::unique_ptr<MarkerDetector>  marker_detector_;

    ros::Timer       control_timer_;
    ros::Publisher   velocity_pub_;
    ros::Publisher   mavros_setpoint_pub_;
    ros::Subscriber  mavros_state_sub_;
    ros::Subscriber  mavros_local_pos_sub_;
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient arming_client_;
    ros::ServiceServer ibvs_service_;

    // ── Parameters ────────────────────────────────────────────────────────────
    int   tracking_marker_id_;
    float marker_size_;
    float control_frequency_;        // Hz — default 100
    float error_threshold_;
    float max_linear_velocity_;
    float max_angular_velocity_;
    float desired_depth_;
    bool  auto_switch_to_offboard_;
    float takeoff_altitude_;
    float takeoff_climb_velocity_;

    // Dynamic desired features mode:
    //   true  → desired corners are always the image-centre rectangle (the drone
    //           chases the marker by trying to keep it centred in frame).
    //   false → desired corners are fixed at first initialisation (original behaviour).
    bool  dynamic_desired_;

    // Feedforward gain multiplier (0 = pure feedback, 1 = full feedforward).
    // Set < 1 during tuning; increase as marker speed grows.
    float ff_gain_;

    // ── State ──────────────────────────────────────────────────────────────────
    bool initialized_;
    bool tracking_;

    // Loss handling — immediate reaction (no holdout counter).
    // The drone holds position for up to hold_on_loss_sec_ before stopping IBVS.
    ros::Time   last_valid_detection_;
    float       hold_on_loss_sec_;   // how long to coast before giving up [s]

    mavros_msgs::State drone_state_;
    bool   drone_state_received_;
    bool   was_ready_;
    ros::Time last_mode_switch_attempt_;
    ros::Time last_arm_attempt_;

    float current_altitude_;
    bool  local_pos_received_;

    StartupState startup_state_;
    ros::Time    setpoint_stream_start_;
    static constexpr double SETPOINT_STREAM_DURATION = 4.0;

    bool ibvs_enabled_;

public:
    // ─────────────────────────────────────────────────────────────────────────
    PlatformIBVSNode()
        : nh_("~"),
          tracking_marker_id_(0), marker_size_(0.1),
          control_frequency_(100.0),     // ← 100 Hz default
          error_threshold_(0.05),
          max_linear_velocity_(0.5), max_angular_velocity_(1.0),
          desired_depth_(1.0),
          auto_switch_to_offboard_(true),
          takeoff_altitude_(2.5f), takeoff_climb_velocity_(0.5f),
          dynamic_desired_(true),        // ← track dynamically by default
          ff_gain_(1.0f),                // applied inside controller; tunable
          initialized_(false), tracking_(false),
          hold_on_loss_sec_(0.1f),       // ← 100 ms coast window (was 5 frames)
          drone_state_received_(false), was_ready_(false),
          last_mode_switch_attempt_(ros::Time(0)),
          last_arm_attempt_(ros::Time(0)),
          current_altitude_(0.f), local_pos_received_(false),
          startup_state_(StartupState::WAITING_FOR_FCU),
          setpoint_stream_start_(ros::Time(0)),
          ibvs_enabled_(false)
    {
        // Load parameters
        nh_.param("tracking_marker_id",    tracking_marker_id_,    999);
        nh_.param("marker_size",           marker_size_,           0.25f);
        nh_.param("control_frequency",     control_frequency_,     100.0f);
        nh_.param("error_threshold",       error_threshold_,       0.05f);
        nh_.param("max_linear_velocity",   max_linear_velocity_,   0.5f);
        nh_.param("max_angular_velocity",  max_angular_velocity_,  1.0f);
        nh_.param("desired_depth",         desired_depth_,         1.0f);
        nh_.param("auto_switch_to_offboard", auto_switch_to_offboard_, true);
        nh_.param("takeoff_altitude",      takeoff_altitude_,      2.5f);
        nh_.param("takeoff_climb_velocity",takeoff_climb_velocity_, 0.5f);
        nh_.param("dynamic_desired",       dynamic_desired_,       true);
        nh_.param("ff_gain",               ff_gain_,               1.0f);
        nh_.param("hold_on_loss_sec",      hold_on_loss_sec_,      0.10f);

        ROS_INFO("PlatformIBVS: freq=%.0f Hz  dynamic_desired=%s  ff_gain=%.2f  "
                 "hold_on_loss=%.3f s",
                 control_frequency_,
                 dynamic_desired_ ? "true" : "false",
                 ff_gain_, hold_on_loss_sec_);

        marker_detector_  = std::make_unique<MarkerDetector>(&nh_);
        ibvs_controller_  = std::make_unique<IBVSController>();

        ros::NodeHandle global_nh, global_nh2;
        mavros_state_sub_ = global_nh.subscribe(
            "/mavros/state", 1,
            &PlatformIBVSNode::mavrosStateCallback, this);
        mavros_local_pos_sub_ = global_nh.subscribe(
            "/mavros/local_position/pose", 10,
            &PlatformIBVSNode::localPositionCallback, this);
        set_mode_client_ = global_nh.serviceClient<mavros_msgs::SetMode>(
            "/mavros/set_mode", true);
        arming_client_ = global_nh.serviceClient<mavros_msgs::CommandBool>(
            "/mavros/cmd/arming", true);
        mavros_setpoint_pub_ = global_nh2.advertise<geometry_msgs::Twist>(
            "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

        ROS_INFO("PlatformIBVS: Waiting for MAVROS services...");
        set_mode_client_.waitForExistence();
        arming_client_.waitForExistence();
        ROS_INFO("PlatformIBVS: MAVROS services found.");

        velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

        control_timer_ = nh_.createTimer(
            ros::Duration(1.0 / control_frequency_),
            &PlatformIBVSNode::controlCallback, this);

        ibvs_service_ = nh_.advertiseService(
            "enable_ibvs", &PlatformIBVSNode::enableIBVSCallback, this);

        last_valid_detection_ = ros::Time::now();

        ROS_INFO("PlatformIBVS: Node initialised.");
    }

    // ─────────────────────────────────────────────────────────────────────────
    bool enableIBVSCallback(std_srvs::SetBool::Request  &req,
                             std_srvs::SetBool::Response &res)
    {
        if (req.data == ibvs_enabled_) { res.success = false; return false; }
        ibvs_enabled_ = req.data;
        ROS_INFO("PlatformIBVS: IBVS %s.", ibvs_enabled_ ? "enabled" : "disabled");
        res.success = true;
        return true;
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Main timer callback
    // ─────────────────────────────────────────────────────────────────────────
    void controlCallback(const ros::TimerEvent &)
    {
        if (!runStartupSequence()) return;
        if (!checkReadiness())    return;

        if (!ibvs_enabled_)
        {
            publishHoldSetpoint();
            ROS_INFO_THROTTLE(5.0,
                "PlatformIBVS: Hovering at %.2f m. Call '~enable_ibvs' to start IBVS.",
                current_altitude_);
            return;
        }
        runIBVS();
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  IBVS loop — called at control_frequency_ Hz
    // ─────────────────────────────────────────────────────────────────────────
    void runIBVS()
    {
        if (!marker_detector_->hasCameraInfo())
        {
            ROS_WARN_THROTTLE(1.0, "PlatformIBVS: Waiting for camera info...");
            publishHoldSetpoint();
            return;
        }

        // ── Initialise / refresh desired features ────────────────────────────
        // In dynamic_desired_ mode we call setDesiredFeatures() every tick so
        // the desired position stays at image centre regardless of past state.
        // This is what lets the drone follow a moving target — the error is
        // always "how far is the marker from the image centre right now".
        if (!initialized_ || dynamic_desired_)
        {
            initializeController();
            if (!initialized_) { publishHoldSetpoint(); return; }
        }

        // ── Marker detection ─────────────────────────────────────────────────
        auto marker = marker_detector_->getMarkerById(tracking_marker_id_);

        if (!marker)
        {
            // No detection this tick — check how long we've been without one.
            double loss_duration =
                (ros::Time::now() - last_valid_detection_).toSec();

            if (loss_duration > hold_on_loss_sec_)
            {
                tracking_ = false;
                ROS_WARN_THROTTLE(1.0,
                    "PlatformIBVS: Marker lost for %.2f s. Holding position.",
                    loss_duration);
            }
            // Always hold — never send stale velocity while marker is gone.
            publishHoldSetpoint();
            return;
        }

        // Valid detection — reset loss timer.
        last_valid_detection_ = ros::Time::now();

        // ── Build current features ───────────────────────────────────────────
        IBVSController::VisualFeatures features;
        features.x = {marker->x0, marker->x1, marker->x2, marker->x3};
        features.y = {marker->y0, marker->y1, marker->y2, marker->y3};

        // ── Depth estimate ───────────────────────────────────────────────────
        float depth = estimateDepthFromCornersAndMatrix(
            marker,
            marker_detector_->getCameraFx(), marker_detector_->getCameraFy(),
            marker_detector_->getCameraU0(), marker_detector_->getCameraV0());

        // ── Kalman feedforward ───────────────────────────────────────────────
        // getMarkerVelocity() returns the Kalman-estimated image-plane velocity.
        // The controller adds ff_gain * velocity to the feedback command so the
        // drone anticipates where the marker is heading rather than always
        // reacting one control period late.
        IBVSController::MarkerVelocity ff =
            marker_detector_->getMarkerVelocity(tracking_marker_id_);

        // Scale by user-tunable ff_gain parameter
        ff.vx_centroid *= ff_gain_;
        ff.vy_centroid *= ff_gain_;
        ff.v_area      *= ff_gain_;
        ff.v_alpha     *= ff_gain_;

        // ── Control law ──────────────────────────────────────────────────────
        auto ctrl = ibvs_controller_->computeControlLaw(features, depth, ff);

        ROS_INFO_THROTTLE(1.0,
            "PlatformIBVS: err=%.4f  vx=%.3f vy=%.3f vz=%.3f ω=%.3f  depth=%.3f",
            ctrl.error_norm,
            ctrl.v_x, ctrl.v_y, ctrl.v_z, ctrl.omega_yaw,
            depth);

        if (ctrl.error_norm < error_threshold_ && !tracking_)
        {
            tracking_ = true;
            ROS_INFO("PlatformIBVS: Marker acquired. Tracking.");
        }

        sendVelocityCommand(
            saturateVelocity(ctrl.v_x,       max_linear_velocity_),
            saturateVelocity(ctrl.v_y,       max_linear_velocity_),
            saturateVelocity(ctrl.v_z,       max_linear_velocity_),
            saturateVelocity(ctrl.omega_yaw, max_angular_velocity_));
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Initialise (or refresh) the desired feature set
    // ─────────────────────────────────────────────────────────────────────────
    void initializeController()
    {
        if (!marker_detector_->hasCameraInfo()) return;

        IBVSController::CameraIntrinsics intrinsics;
        intrinsics.f_x = marker_detector_->getCameraFx();
        intrinsics.f_y = marker_detector_->getCameraFy();
        intrinsics.u_0 = marker_detector_->getCameraU0();
        intrinsics.v_0 = marker_detector_->getCameraV0();
        ibvs_controller_->setActiveCamera(intrinsics);

        // Desired = centred square in image coordinates.
        // Called every tick in dynamic mode so the controller always tries to
        // bring the marker to the image centre — the drone translates to follow.
        float cx = intrinsics.u_0, cy = intrinsics.v_0;
        float w  = 80.f, h = 80.f;

        IBVSController::VisualFeatures desired;
        desired.x = {cx-w, cx-w, cx+w, cx+w};
        desired.y = {cy+h, cy-h, cy-h, cy+h};
        ibvs_controller_->setDesiredFeatures(desired, desired_depth_);

        if (!initialized_)
        {
            initialized_ = true;
            ROS_INFO("PlatformIBVS: Controller initialised. dynamic_desired=%s",
                     dynamic_desired_ ? "true" : "false");
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Depth estimation from corner geometry (unchanged from original)
    // ─────────────────────────────────────────────────────────────────────────
    float estimateDepthFromCornersAndMatrix(
        const std::shared_ptr<MarkerDetector::MarkerData> &marker,
        float fx, float fy, float u0, float v0)
    {
        auto norm = [&](float u, float v, float &xn, float &yn) {
            xn = (u - u0) / fx;
            yn = (v - v0) / fy;
        };

        float x0n, y0n, x1n, y1n, x2n, y2n, x3n, y3n;
        norm(marker->x0, marker->y0, x0n, y0n);
        norm(marker->x1, marker->y1, x1n, y1n);
        norm(marker->x2, marker->y2, x2n, y2n);
        norm(marker->x3, marker->y3, x3n, y3n);

        auto dist = [](float ax, float ay, float bx, float by) {
            float dx = bx-ax, dy = by-ay;
            return std::sqrt(dx*dx + dy*dy);
        };

        float d01 = dist(x0n, y0n, x1n, y1n);
        float d12 = dist(x1n, y1n, x2n, y2n);
        float d30 = dist(x3n, y3n, x0n, y0n);
        float avg  = (d01 + d12 + d30) / 3.0f;

        if (avg < 1e-6f)
        {
            ROS_WARN("PlatformIBVS: Invalid marker distance. Using desired_depth.");
            return desired_depth_;
        }

        float depth = marker_size_ / avg;
        depth = std::max(0.1f, std::min(10.0f, depth));
        return depth;
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Startup state machine (unchanged logic from original)
    // ─────────────────────────────────────────────────────────────────────────
    bool runStartupSequence()
    {
        if (startup_state_ == StartupState::READY) return true;

        switch (startup_state_)
        {
        case StartupState::WAITING_FOR_FCU:
            if (!drone_state_received_ || !drone_state_.connected)
            {
                ROS_INFO_THROTTLE(3.0, "PlatformIBVS: [1/5] Waiting for FCU...");
                break;
            }
            ROS_INFO("PlatformIBVS: [1/5] FCU connected.");
            setpoint_stream_start_ = ros::Time::now();
            startup_state_ = StartupState::STREAMING_SETPOINTS;
            break;

        case StartupState::STREAMING_SETPOINTS:
        {
            publishHoldSetpoint();
            double elapsed = (ros::Time::now() - setpoint_stream_start_).toSec();
            ROS_INFO_THROTTLE(0.5,
                "PlatformIBVS: [2/5] Streaming setpoints (%.1f/%.1f s)...",
                elapsed, SETPOINT_STREAM_DURATION);
            if (elapsed >= SETPOINT_STREAM_DURATION)
            {
                startup_state_ = StartupState::REQUESTING_OFFBOARD;
                ROS_INFO("PlatformIBVS: [2/5] Requesting OFFBOARD.");
            }
            break;
        }

        case StartupState::REQUESTING_OFFBOARD:
            publishHoldSetpoint();
            if (drone_state_.mode == "OFFBOARD")
            {
                ROS_INFO("PlatformIBVS: [3/5] OFFBOARD confirmed. Arming...");
                startup_state_ = StartupState::ARMING;
                break;
            }
            requestOffboardMode();
            break;

        case StartupState::ARMING:
            publishHoldSetpoint();
            if (drone_state_.armed)
            {
                ROS_INFO("PlatformIBVS: [4/5] Armed. Taking off to %.1f m.",
                         takeoff_altitude_);
                startup_state_ = StartupState::TAKING_OFF;
                break;
            }
            requestArming();
            break;

        case StartupState::TAKING_OFF:
        {
            if (!local_pos_received_)
            { publishHoldSetpoint(); break; }

            const float tol = 0.15f;
            if (current_altitude_ >= takeoff_altitude_ - tol)
            {
                ROS_INFO("PlatformIBVS: [5/5] At %.2f m. READY.", current_altitude_);
                startup_state_ = StartupState::READY;
                publishHoldSetpoint();
                break;
            }
            geometry_msgs::Twist climb;
            climb.linear.z = takeoff_climb_velocity_;
            mavros_setpoint_pub_.publish(climb);
            ROS_INFO_THROTTLE(1.0,
                "PlatformIBVS: [5/5] Climbing... %.2f m / %.1f m",
                current_altitude_, takeoff_altitude_);
            break;
        }

        case StartupState::READY:
            break;
        }
        return false;
    }

    // ─────────────────────────────────────────────────────────────────────────
    bool checkReadiness()
    {
        bool ready_now = isDroneReady();

        if (was_ready_ && !ready_now)
        {
            ROS_WARN("PlatformIBVS: Lost readiness. Recovering...");
            resetController();
            startup_state_ = StartupState::REQUESTING_OFFBOARD;
        }
        was_ready_ = ready_now;
        if (!ready_now) publishHoldSetpoint();
        return ready_now;
    }

    bool isDroneReady() const
    {
        return drone_state_received_ &&
               drone_state_.armed &&
               drone_state_.mode == "OFFBOARD";
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Velocity output
    // ─────────────────────────────────────────────────────────────────────────
    void sendVelocityCommand(float vx, float vy, float vz, float omega)
    {
        geometry_msgs::Twist cmd;
        // IBVS camera frame → PX4 body frame
        cmd.linear.x  = -vy;
        cmd.linear.y  = -vx;
        cmd.linear.z  = -vz;
        cmd.angular.z = -omega;

        velocity_pub_.publish(cmd);
        mavros_setpoint_pub_.publish(cmd);
    }

    void publishHoldSetpoint()
    {
        geometry_msgs::Twist zero;
        mavros_setpoint_pub_.publish(zero);
    }

    // ─────────────────────────────────────────────────────────────────────────
    float saturateVelocity(float v, float limit)
    {
        return std::abs(v) > limit ? (v > 0.f ? limit : -limit) : v;
    }

    void stopMotion() { sendVelocityCommand(0, 0, 0, 0); }

    void resetController()
    {
        initialized_  = false;
        tracking_     = false;
        ibvs_controller_->reset();
        last_valid_detection_ = ros::Time::now();
        ROS_INFO("PlatformIBVS: Controller reset.");
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  MAVROS helpers
    // ─────────────────────────────────────────────────────────────────────────
    void requestOffboardMode()
    {
        if (!auto_switch_to_offboard_)
        {
            ROS_WARN_THROTTLE(2.0, "PlatformIBVS: Set auto_switch_to_offboard=true "
                              "or switch manually.");
            return;
        }
        const ros::Duration COOLDOWN(3.0);
        if ((ros::Time::now() - last_mode_switch_attempt_) < COOLDOWN)
        {
            ROS_INFO_THROTTLE(1.0, "PlatformIBVS: Waiting for OFFBOARD ack...");
            return;
        }
        last_mode_switch_attempt_ = ros::Time::now();

        mavros_msgs::SetMode srv;
        srv.request.custom_mode = "OFFBOARD";
        if (!set_mode_client_.call(srv))
            ROS_ERROR("PlatformIBVS: /mavros/set_mode call failed.");
        else if (!srv.response.mode_sent)
            ROS_WARN("PlatformIBVS: mode_sent=false (setpoint stream not ready yet?).");
    }

    void requestArming()
    {
        const ros::Duration COOLDOWN(3.0);
        if ((ros::Time::now() - last_arm_attempt_) < COOLDOWN) return;
        last_arm_attempt_ = ros::Time::now();

        mavros_msgs::CommandBool srv;
        srv.request.value = true;
        if (!arming_client_.call(srv))
            ROS_ERROR("PlatformIBVS: /mavros/cmd/arming call failed.");
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  ROS subscriber callbacks
    // ─────────────────────────────────────────────────────────────────────────
    void mavrosStateCallback(const mavros_msgs::State::ConstPtr &msg)
    {
        drone_state_ = *msg;
        if (!drone_state_received_)
        {
            drone_state_received_ = true;
            ROS_INFO("PlatformIBVS: MAVROS state received.");
        }
    }

    void localPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        current_altitude_ = msg->pose.position.z;
        if (!local_pos_received_)
        {
            local_pos_received_ = true;
            ROS_INFO("PlatformIBVS: Local position received. Alt=%.2f m",
                     current_altitude_);
        }
    }
};

// ─────────────────────────────────────────────────────────────────────────────
//  main
// ─────────────────────────────────────────────────────────────────────────────

static std::unique_ptr<PlatformIBVSNode> g_node;

void signalHandler(int sig)
{
    ROS_INFO("Signal %d received — shutting down.", sig);
    g_node.reset();
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "platform_ibvs_node");
    signal(SIGINT,  signalHandler);
    signal(SIGTERM, signalHandler);

    ROS_INFO("Starting Platform IBVS Node (moving-target edition)...");
    try
    {
        g_node = std::make_unique<PlatformIBVSNode>();
        // Four threads: control timer, /mavros/state, service reply, spare.
        ros::AsyncSpinner spinner(4);
        spinner.start();
        ros::waitForShutdown();
    }
    catch (const std::exception &e)
    { ROS_FATAL("Fatal: %s", e.what()); return 1; }

    return 0;
}
