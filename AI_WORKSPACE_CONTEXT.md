# Workspace Context: Drone Image-Based Visual Servoing (IBVS)

> **For AI agents.** This document is the ground-truth reference for the Drone IBVS project.
> All details are derived directly from source code. When in doubt, source code takes precedence.
> Last updated to reflect: ID 999 changed from bounding-box to cv::projectPoints (2026-03-23).

---

## 1. System Architecture Overview

The goal of this project is to build a ROS-based solution that enables a drone to autonomously
track and land on a moving platform using visual feedback. The system operates as a
closed-loop control system.

- **Control rate:** `platform_ibvs_node` runs at **50 Hz**.
- **Detection rate:** camera-driven, typically **30 Hz**.
- **ROS distro:** Noetic on Ubuntu 20.04.
- **Sim:** Gazebo 11 + PX4 SITL via MAVROS.
- **Dev environment:** Docker (`ros:noetic-perception` base, user `tejas`, workspace at `~/dev_ws`).

### Core Data Flow

```
Camera (30 Hz)
    │
    ▼
group_detection_node         [aruco_detect package]
  ├─ detectMarkers → estimatePoseBoard
  ├─ if success:  KF correct() → publish ID 999 (detected)
  └─ if failure:  KF predict() → projectPoints → publish ID 999 (tracked)
    │
    │  /fiducial_vertices  (ID 999 bounding box)
    │  /platform_tracking_active  (Bool: true=tracker, false=direct)
    ▼
platform_ibvs_node           [dynamic_ibvs package]
  ├─ Startup state machine (FCU → OFFBOARD → ARM → TAKEOFF)
  ├─ estimateDepthFromCornersAndMatrix()
  ├─ IBVSController::computeControlLaw()
  └─ if tracking_mode_: scale velocities × 1.25 (TRACKING_VELOCITY_SCALE)
    │
    │  /mavros/setpoint_velocity/cmd_vel_unstamped
    ▼
PX4 Autopilot → Motors
```

---

## 2. Package Map

```
src/
├── aruco_detect/          # Perception: detection + tracking
│   ├── src/
│   │   ├── aruco_detect_node.cpp      # Single-marker detection (Phase I)
│   │   └── group_detection_node.cpp   # Board detection + KF tracker (Phase II)
│   ├── config/
│   │   ├── aruco_detect.yaml          # marker_size, dictionary, frame IDs
│   │   └── marker_groups.yaml         # Physical board layout (5 markers)
│   └── launch/
│       ├── aruco_detect.launch        # Phase I launch
│       └── group_detection.launch     # Phase II launch (includes tracking params)
│
├── dynamic_ibvs/          # Control: IBVS + drone state machine
│   ├── include/dynamic_ibvs/
│   │   ├── ibvs_controller.hpp
│   │   └── marker_detector.hpp
│   ├── src/
│   │   ├── ibvs_controller.cpp        # Core IBVS math
│   │   ├── marker_detector.cpp        # Subscribes to fiducial_vertices
│   │   └── platform_ibvs_node.cpp     # Main node: state machine + control loop
│   ├── config/
│   │   └── platform_ibvs_params.yaml  # All tunable params
│   └── launch/
│       ├── minimal.launch
│       └── platform_ibvs.launch
│
├── ibvs_msgs/             # Custom messages: Signal.msg, SignalArray.msg
├── platform_control/      # Gazebo plugin to drive the moving platform
│   └── src/moving_platform_plugin.cpp
└── Drone_IBVS/            # Phase I IBVS controller (legacy, see sub_ibvs.cpp)
```

---

## 3. Key Packages — Detailed

### 3.1 `aruco_detect` Package (Perception Layer)

#### `aruco_detect_node.cpp` — Phase I, single-marker detection

| Property | Value |
|---|---|
| Node name | `aruco_detect_node` |
| Default dictionary | `DICT_5X5_50` (set in `aruco_detect.yaml`, **not** 4X4) |
| Marker size param | `marker_size: 20` cm (passed directly to `estimatePoseSingleMarkers`) |
| API style | OpenCV 3.4 (`cv::aruco::detectMarkers`, not the newer `ArucoDetector` class) |
| Translation units | Output of `estimatePoseSingleMarkers` is in cm; **divided by 100** before publishing |
| Quaternion conversion | Manual Shepperd method — not via tf2 |
| Distortion coefficients | Hardcoded `cv::Mat(5,1,CV_64F)` — will crash if `camera_info.D.size() < 5` |
| Camera calibration | Extracted once on first callback; never refreshed |

**Subscribed topics:**

| Topic | Type |
|---|---|
| `/camera/rgb/image_raw` | `sensor_msgs/Image` |
| `/camera/depth/image_raw` | `sensor_msgs/Image` |
| `/camera/rgb/camera_info` | `sensor_msgs/CameraInfo` |

**Published topics:**

| Topic | Type | Notes |
|---|---|---|
| `/fiducial_vertices` | `fiducial_msgs/FiducialArray` | Pixel corners of each detected marker |
| `/fiducial_transforms` | `fiducial_msgs/FiducialTransformArray` | 3D pose, translation in **meters** |
| `/fiducial_images` | `sensor_msgs/Image` | Annotated visualization |

---

#### `group_detection_node.cpp` — Phase II, board detection + KF tracker

This is the primary perception node for landing on the moving platform.
It combines two responsibilities: board-level pose estimation and a Kalman filter
that keeps the control system fed even when the platform is partially or fully occluded.

**Board configuration (`marker_groups.yaml`):**

```
Platform: 2 m × 2 m square board, 5 markers total
  ID  0  — centre,   size 0.50 m, position [0.0,  0.0,  0.0]
  ID 17  — corner,   size 0.25 m, position [-0.6, 0.6,  0.0]
  ID 22  — corner,   size 0.25 m, position [ 0.6, 0.6,  0.0]
  ID 42  — corner,   size 0.25 m, position [ 0.6,-0.6,  0.0]
  ID 37  — corner,   size 0.25 m, position [-0.6,-0.6,  0.0]
```

**The Virtual Marker Trick (ID 999):**
The `dynamic_ibvs` controller is unaware of multi-marker boards. The detection node
abstracts the entire platform into a single synthetic fiducial:

- **On successful `estimatePoseBoard`:** The known 3D platform corners
  (`platform_corners_3d_`) are projected to image space via `cv::projectPoints()`
  using the board's `rvec`/`tvec`. The resulting perspective-correct pixel
  coordinates are published as `fiducial_id = 999` on `/fiducial_vertices`.
  Corners may lie outside the image frame — this is intentional.
- **On tracker frames:** The same projection is performed using the KF-predicted
  translation + last valid rotation (see Tracking below).
- **Format (always):** `(x0,y0)` through `(x3,y3)` are the 4 projected
  platform corners in pixel coordinates, preserving perspective.

**KF Tracking system:**

The node maintains a `cv::KalmanFilter` that is always running, every frame.

| KF property | Value |
|---|---|
| State vector | `[tx, ty, tz, vx, vy, vz]` — 6D, position + velocity in camera frame |
| Measurement vector | `[tx, ty, tz]` — position only from `estimatePoseBoard` |
| Motion model | Constant velocity; `dt` updated from ROS timestamps each frame |
| Process noise (position) | `1e-4` (tight — position is well-constrained) |
| Process noise (velocity) | `5e-3` (looser — platform can accelerate) |
| Measurement noise | `1e-2` (~1-2 cm, typical board pose accuracy) |

**Per-frame logic:**

```
Every frame:
  detectMarkers()
  estimatePoseBoard()

  if board pose valid (valid > 0):
      kf.predict()
      kf.correct(tvec)          ← measurement update
      cache rvec                ← rotation held for re-projection
      publish ID 999 (detected)
      publish tracking_active = false

  else (partial/full occlusion):
      if kf not initialised OR elapsed > max_tracking_duration:
          publish tracking_active = false   ← platform lost
      else:
          kf.predict() only                 ← advance without correction
          kf.statePost = kf.statePre        ← commit a-priori estimate
          projectPoints(platform_corners_3d, last_rvec, predicted_tvec)
          publish ID 999 (tracked)
          publish tracking_active = true
```

**Important:** `rvec` is held fixed during tracking frames. This is intentional —
rotation from partial board detections is far noisier than translation, and the
platform is unlikely to yaw significantly within the 2-second tracking window.

**Tunable tracking params (set in `group_detection.launch`):**

| Param | Default | Meaning |
|---|---|---|
| `max_tracking_duration` | `2.0` s | How long to trust KF prediction before declaring platform lost. Increase for high-speed cruise phases. |
| `platform_half_size` | `0.725` m | Half-extent of the platform bounding box used for re-projection. Derived from layout: 0.6 + 0.125 = 0.725 m. Update if the board layout changes. |

**Published topics (in addition to the aruco_detect_node topics above):**

| Topic | Type | Notes |
|---|---|---|
| `platform_pose` | `geometry_msgs/PoseStamped` | Board-level pose from `estimatePoseBoard`; only published on successful detection |
| `/fiducial_vertices` | `fiducial_msgs/FiducialArray` | Always contains ID 999 while platform is detected or tracked |
| `/fiducial_transforms` | `fiducial_msgs/FiducialTransformArray` | Per-marker transforms for individual board members |
| `platform_tracking_active` | `std_msgs/Bool` | `true` = KF tracker is filling in; `false` = direct detection or platform lost |

**KF tuning note:**
If the tracker lags during high-acceleration platform movement, increase velocity
process noise from `5e-3` toward `1e-2`. If predictions oscillate, reduce toward `1e-3`.
Monitor with `rqt_plot /platform_tracking_active`.

---

### 3.2 `dynamic_ibvs` Package (Control Layer)

All three source files compile into the single `platform_ibvs_node` executable.

#### `ibvs_controller.cpp` — Core IBVS math

Takes 4 corner pixel coordinates and a depth estimate; returns 4-DOF velocity commands.

**Pipeline per call to `computeControlLaw()`:**

1. Normalize corners: `x_n = (px - u0) / fx`, `y_n = (py - v0) / fy`
2. Compute `extractCentroid()` → `(cx, cy)` in normalized coords
3. Compute `computeCenteredMoments()` → 4×4 matrix of moments `m_ij = Σ xⁱ yʲ`
4. Extract area: `accu(moments(row 2, cols 0..2))`
5. Compute orientation: `α = 0.5 * atan2(2*u20, u02 - u20)` (second-order moments)
6. `computeError()` — 4-element error vector:
   - `e[0] = a_n * cx - depth * cx_desired`
   - `e[1] = a_n * cy - depth * cy_desired`
   - `e[2] = a_n - depth` (area/scale error)
   - `e[3] = α - α_desired`
7. `computeJacobian()` — 4×4 interaction matrix; initialised as `-I`, with
   coupling terms added: `J(0,3) = cy`, `J(1,3) = -cx` (yaw coupling to translation)
8. `computeAdaptiveGain()`:
   ```
   λ = λ_max - (λ_max - λ_min) * (‖e‖ / ‖e‖_max)
   λ_min = 0.5, λ_max = 1.25
   ```
   **Correct interpretation:** λ is **large (1.25) when error is small** (near target,
   fine correction) and **small (0.5) when error is large** (far away, conservative).
   This is the opposite of the naming in the legacy `sub_ibvs.cpp` (Phase I), where
   `min_lambda` and `max_lambda` were accidentally swapped. The Phase II implementation
   is correct.
9. `control_velocity = -λ * pinv(J) * e` (Armadillo `arma::pinv`)
10. Output: `[v_x, v_y, v_z, ω_yaw]`

**Desired features** are set in `initializeController()` inside `platform_ibvs_node.cpp`:
an 80 px square centred at the principal point `(u0, v0)`.

---

#### `marker_detector.cpp` — Data ingest

Subscribes to `/fiducial_vertices` and stores the latest corner coordinates for each
marker ID in `detected_markers_` (cleared and rebuilt on every callback).
Camera intrinsics are read once from the Gazebo camera topic:
`/iris_downward_depth_camera/camera/rgb/camera_info`.

---

#### `platform_ibvs_node.cpp` — Orchestrator

**Startup state machine** (6 states, runs inside the 50 Hz timer):

```
WAITING_FOR_FCU
    │  /mavros/state received and connected
    ▼
STREAMING_SETPOINTS          ← publishes zero-velocity to MAVROS for 4 s
    │  PX4 watchdog satisfied
    ▼
REQUESTING_OFFBOARD          ← calls /mavros/set_mode, rate-limited to 1/3 s
    │  drone_state_.mode == "OFFBOARD"
    ▼
ARMING                       ← calls /mavros/cmd/arming, rate-limited to 1/3 s
    │  drone_state_.armed == true
    ▼
TAKING_OFF                   ← publishes climb velocity to MAVROS directly
    │  current_altitude_ >= takeoff_altitude_ - 0.15 m
    ▼
READY                        ← normal IBVS operation begins
```

**IBVS control loop** (only runs in READY state with `ibvs_enabled_ == true`):

1. Gate: wait for camera info, wait for controller init.
2. `getMarkerById(999)` — fetch the virtual bounding box marker.
3. If marker missing for > `MAX_CONSECUTIVE_LOSSES` (5) frames: hover.
4. `estimateDepthFromCornersAndMatrix()` — geometric depth from corner spacing:
   ```
   depth = marker_size_ / avg_normalised_corner_distance
   ```
   Clamped to [0.1, 10.0] m. `marker_size_` is set to `0.725` m in params
   (the platform half-size, matching the bounding box scale).
5. `ibvs_controller_->computeControlLaw(features, depth)`
6. Saturate each axis to `max_linear_velocity_` / `max_angular_velocity_`.
7. `sendVelocityCommand()` — maps IBVS camera-frame output to NED body frame:
   ```
   vel.linear.x  = -v_y   (camera left/right → NED forward/back)
   vel.linear.y  = -v_x   (camera fwd/back   → NED left/right)
   vel.linear.z  = -v_z
   vel.angular.z = -ω_yaw
   ```
   If `tracking_mode_ == true` (KF tracker data), all values are additionally
   scaled by **1.25** (`TRACKING_VELOCITY_SCALE`) before publishing. This
   amplifies commands slightly during tracker-only frames to compensate for
   the lag inherent in KF-predicted bounding boxes.
8. Published to both `~/cmd_vel` AND directly to
   `/mavros/setpoint_velocity/cmd_vel_unstamped` (keeps PX4 watchdog alive).

**Tracking awareness:**
The node subscribes to `platform_tracking_active` (std_msgs/Bool). When `true`,
it sets `tracking_mode_` and reduces commanded velocities. When the platform is
re-detected (`false`), it reverts to full-gain commands and logs the transition.

**Threading:** `ros::AsyncSpinner(4)` — required so that blocking MAVROS service
calls (`set_mode`, `cmd/arming`) do not starve the subscription callbacks that
deliver the ACK responses. A single-threaded `ros::spin()` would deadlock here.

**Service gate:** The IBVS loop is blocked until `~/enable_ibvs` (std_srvs/SetBool)
is called with `data=true`. Before that, the drone hovers at `takeoff_altitude_`.

---

### 3.3 `platform_control` Package (Simulation)

A `gazebo::ModelPlugin` (`MovingPlatformPlugin`) that moves the landing platform
in Gazebo by directly integrating velocity each physics step:
```
pose.Pos().X() += vx_ * dt
pose.Rot updated via yaw_rate_ * dt
```
Controlled via ROS service `/platform/set_velocity`
(`platform_control/SetPlatformVelocity.srv`: `{vx, vy, vz, yaw_rate}`).
Uses `ros::AsyncSpinner(1)` internally.

---

### 3.4 `ibvs_msgs` Package (Custom Messages)

```
Signal.msg       { float64 v1; float64 v2; float64 v3; float64 v4; }
SignalArray.msg  { float64 norm_err; Signal err_signal; Signal ctrl_signal; }
```

Published on `/sub_ibvs/analysis` at 20 Hz (Phase I `Drone_IBVS` package only).

---

## 4. Key Parameters (`platform_ibvs_params.yaml`)

| Param | Value | Notes |
|---|---|---|
| `tracking_marker_id` | 999 | Virtual bounding box marker |
| `marker_size` | 0.725 m | Platform outer half-extent (matches `platform_half_size`) |
| `control_frequency` | 50.0 Hz | Main timer rate |
| `desired_depth` | 2.5 m | Camera-to-platform distance at convergence |
| `takeoff_altitude` | 3.5 m | Initial autonomous climb target |
| `takeoff_climb_velocity` | 0.5 m/s | Climb rate |
| `max_linear_velocity` | 1.5 m/s | Per-axis saturation |
| `max_angular_velocity` | 1.0 rad/s | Yaw saturation |
| `lambda_min` | 0.5 | Gain at large error (conservative) |
| `lambda_max` | 1.25 | Gain at small error (aggressive) |
| `auto_switch_to_offboard` | true | Node handles mode switch automatically |

---

## 5. Topic Reference

| Topic | Type | Publisher | Subscriber | Notes |
|---|---|---|---|---|
| `/iris_downward_depth_camera/camera/rgb/image_raw` | `sensor_msgs/Image` | Gazebo | `group_detection_node` | Camera remapped in launch |
| `/fiducial_vertices` | `fiducial_msgs/FiducialArray` | `group_detection_node` | `marker_detector` | Always contains ID 999 while platform visible/tracked |
| `/fiducial_transforms` | `fiducial_msgs/FiducialTransformArray` | `group_detection_node` | — | Per-marker poses |
| `platform_pose` | `geometry_msgs/PoseStamped` | `group_detection_node` | — | Board-level pose, detection frames only |
| `platform_tracking_active` | `std_msgs/Bool` | `group_detection_node` | `platform_ibvs_node` | `true` = KF tracker active |
| `/mavros/state` | `mavros_msgs/State` | MAVROS | `platform_ibvs_node` | Startup state machine |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | MAVROS | `platform_ibvs_node` | Altitude feedback during takeoff |
| `/mavros/setpoint_velocity/cmd_vel_unstamped` | `geometry_msgs/Twist` | `platform_ibvs_node` | PX4 | Primary actuation + watchdog feed |
| `/platform/set_velocity` | service | — | `platform_control` | Drive the Gazebo platform |

---

## 6. Important Concepts for Agents

**Virtual bounding box (ID 999):**
The `dynamic_ibvs` controller is entirely unaware of multi-marker boards. It treats
the 4 corners of the bounding box emitted by `group_detection_node` as a single
large marker. This abstraction means the IBVS math is identical whether 1 or 5 markers
are visible — the only thing that changes is the quality of the bounding box.

**KF tracker vs. direct detection:**
When `platform_tracking_active = true`, the ID 999 corners published to
`/fiducial_vertices` are synthesised from a KF prediction, not from image pixels.
The controller scales its output by 1.25 (`TRACKING_VELOCITY_SCALE`) in this mode. Agents modifying the control
loop should check `tracking_mode_` before assuming corner positions are directly observed.

**Depth estimation is geometric, not from the depth camera:**
Despite subscribing to the depth stream (needed by `TimeSynchronizer`), the system
does not use RGBD depth values. Depth is computed from the apparent size of the marker
in the image. The depth camera stream is reserved for a future filtering enhancement.

**OFFBOARD watchdog:**
PX4 requires setpoints to arrive at >2 Hz before it will accept an OFFBOARD mode switch,
and will fall back to POSCTL if the stream stops. `platform_ibvs_node` publishes
zero-velocity directly to MAVROS during every startup state and during every marker-loss
hover. Never remove or delay these hold-setpoint calls.

**AsyncSpinner is mandatory:**
The startup sequence calls blocking MAVROS services (`/mavros/set_mode`,
`/mavros/cmd/arming`) from inside a timer callback. A single-threaded spinner would
deadlock waiting for the service ACK that can only arrive on the same thread.
`AsyncSpinner(4)` provides the minimum thread count to avoid this.

**Phase I vs Phase II:**
`Drone_IBVS/src/sub_ibvs.cpp` (Phase I) is a separate, older implementation that tracks
individual markers (IDs 0 and 7). It is not used in Phase II. The adaptive gain in Phase I
has inverted `min_lambda`/`max_lambda` naming — this was a known bug that is fixed in the
Phase II `ibvs_controller.cpp`.

**Dictionary mismatch trap:**
`aruco_detect.yaml` sets `aruco_dictionary: DICT_5X5_50`. The README and older
documentation incorrectly state `DICT_4X4_50`. Any marker generated for this system
must use `DICT_5X5_50`.

---

## 7. Build and Run

```bash
# Build
cd ~/dev_ws && catkin build && source devel/setup.bash

# Phase II full system
roslaunch aruco_detect group_detection.launch
roslaunch dynamic_ibvs minimal.launch

# Enable IBVS after drone reaches altitude
rosservice call /platform_ibvs/enable_ibvs "data: true"

# Drive the platform
rosservice call /platform/set_velocity "{vx: 0.3, vy: 0.0, vz: 0.0, yaw_rate: 0.0}"

# Monitor tracking
rqt_plot /platform_tracking_active
rostopic echo /fiducial_vertices
```