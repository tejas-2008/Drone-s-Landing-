# Workspace Context: Drone Image-Based Visual Servoing (IBVS)

This document provides a comprehensive overview of the Drone IBVS project workspace, designed to quickly onboard AI agents to the system architecture, core components, and specific implementations within the ROS packages.

## 1. System Architecture Overview

The goal of this project is to build a ROS-based solution that enables a drone to autonomously track and land on a moving platform using visual feedback. The system operates as a closed-loop control system.

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
  ├─ if success:  KF correct() → projectPoints → publish ID 999 (detected)
  └─ if failure:  KF predict() → projectPoints → publish ID 999 (tracked)
    │
    │  /fiducial_vertices  (ID 999 projected platform corners)
    │  /platform_tracking_active  (Bool: true=tracker, false=direct)
    ▼
platform_ibvs_node           [dynamic_ibvs package]
  ├─ Startup state machine (FCU → OFFBOARD → ARM → TAKEOFF)
  ├─ estimateDepthFromCornersAndMatrix()
  ├─ IBVSController::computeControlLaw() (Full PID)
  ├─ Descent logic: switch ID 999 (approach) → ID 0 (touchdown)
  └─ if tracking_mode_: scale velocities × 1.25 (TRACKING_VELOCITY_SCALE)
    │
    │  /mavros/setpoint_velocity/cmd_vel_unstamped
    ▼
PX4 Autopilot → Motors
```

## 2. Key Packages

### 2.1. `aruco_detect` Package (Perception Layer)

This package bridges standard OpenCV ArUco detection with the custom IBVS control system. It processes RGBD streams and publishes fiducial information.

*   **Standalone Detection (`aruco_detect_node.cpp`)** — Phase I, single-marker
    *   Subscribes to RGB, depth, and camera intrinsics using a `TimeSynchronizer`.
    *   Detects individual ArUco markers (actual dict: `DICT_5X5_50`).
    *   Estimates 3D pose using `cv::aruco::estimatePoseSingleMarkers`.
    *   Outputs marker corners in pixel coordinates to `/fiducial_vertices`.

*   **Grouped Detection (`group_detection_node.cpp`)** — Phase II, board + KF tracker
    *   *Primary perception node for platform landing.*
    *   Loads YAML config defining 5-marker board (`marker_groups.yaml`):
        - ID 0 — centre, 0.50 m, at [0,0,0]
        - IDs 17,22,42,37 — corners, 0.25 m, at ±0.6 m offsets
    *   Uses `cv::aruco::estimatePoseBoard` for robust pose estimation.
    *   **Virtual Marker (ID 999):** Projects known 3D platform corners via `cv::projectPoints()` → perspective-correct pixel corners.
    *   **KF Tracking:** Constant-velocity Kalman filter (6D state). On occlusion: predict-only with held rvec for up to 2 seconds.
    *   Publishes `platform_tracking_active` (Bool) to signal tracker vs. direct mode.

### 2.2. `dynamic_ibvs` Package (Control Layer)

All three source files compile into the single `platform_ibvs_node` executable.

*   **`IBVSController` Class (`ibvs_controller.cpp`)**
    *   **Full PID Law:** `v = -λ(‖e‖) · J⁺ · e - ki · J⁺ · ∫e dt - kd · J⁺ · ė`.
    *   Adaptive gain: λ_max=1.25 (small error), λ_min=0.5 (large error).
    *   **Integral Control:** Eliminates steady-state bias (depth error / moving target lag).
    *   **Anti-Windup:** Integral vector L2-norm clamped to `windup_limit` (default 0.5).
    *   Uses Armadillo library (`arma::pinv`).

*   **`MarkerDetector` Class (`marker_detector.cpp`)**
    *   Subscribes to `/fiducial_vertices`, stores latest corners per marker ID.
    *   Camera intrinsics from `/iris_downward_depth_camera/camera/rgb/camera_info`.

*   **`PlatformIBVSNode` (`platform_ibvs_node.cpp`)**
    *   Orchestrator with 6-state startup machine.
    *   **Touchdown Logic:** Stabilizes for 5s before descending at `0.1 m/s`.
    *   **Marker Switch:** Automically transitions from ID 999 to ID 0 on descent.
    *   **Re-initialization:** `reinitializeForDescentMarker()` computes new desired windows on-the-fly to prevent error spikes.
    *   **Lateral Scaling:** `descent_lateral_scale` damper (default 0.3) reduces gains during descent.
    *   AsyncSpinner(4) mandatory to avoid deadlock.

### 2.3. `platform_control` Package (Simulation)

Gazebo `ModelPlugin` that moves the landing platform via direct pose integration. Controlled via ROS service `/platform/set_velocity`.

## 3. Important Concepts and Edge Cases

*   **Virtual Marker (ID 999):** Perspective-correct corners, not axis-aligned bounding box.
*   **Depth Estimation:** Geometric from marker apparent size, NOT from depth camera.
*   **OFFBOARD Watchdog:** PX4 needs >2 Hz setpoints. Node publishes zero-velocity during idle.
*   **AsyncSpinner(4):** Mandatory — single-threaded spinner deadlocks on blocking calls.
*   **PID Reset:** `ibvs_controller_->reset()` must be called whenever switching markers (e.g., transition to ID 0) to clear integral wind-up.

## 4. Build and Run

```bash
cd ~/dev_ws && catkin build dynamic_ibvs && source devel/setup.bash

# Phase II full system
roslaunch dynamic_ibvs platform_ibvs.launch

# Enable IBVS after takeoff
rosservice call /platform_ibvs_node/enable_ibvs "data: true"
```

## 5. ai-lifecycle Module (2026-10-28)

- **Location:** `src/ai-lifecycle/`
- **Entry point:** `ailc` CLI + `ailc tui` TUI
- **Managers:** `UvDockerDependencyManager`, `YoloDatasetManager`, `YoloV11ModelManager`, `PostTrainingReporter`
- **Config:** Pydantic v2 layered YAML (`configs/base.yaml` + experiment override)
- **Registry:** `@register_model("yolov11")`, `@register_dataset("yolo")` — new variants = 1 file + 1 decorator
- **Docker:** `docker/Dockerfile.base` (CUDA 12.1 + uv), `docker/docker-compose.yml`, `docker/entrypoint.sh`
- **Outputs:** `outputs/<project>/<run_name>_<timestamp>/` with checkpoints, plots, metrics, summary.md

- **Fixing PX4 Mode Switch (conv 73245ff2):** Resolved "poll timeout" errors via AsyncSpinner(4).
- **Visual Servoing Drone Control (conv 132142ba):** Fixed segfaults and adapted logic for PX4 OFFBOARD mode.
- **PD Controller Implementation (conv 573d367b):** Added derivative term and adaptive gains to handle high speed platform tracking.
- **PID Upgrade & Landing Logic (conv 573d367b):** Upgraded to full PID, added descent re-initialization, and lateral damping during touchdown.