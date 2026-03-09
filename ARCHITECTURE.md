# System Architecture

## Overview Diagram

```
┌──────────────────────────────────────────────────────────────┐
│                     DRONE IBVS SYSTEM                        │
└──────────────────────────────────────────────────────────────┘

                         Camera Stream
                              ↓
                    ┌──────────────────┐
                    │ ArUco Detector   │
                    │ (aruco_detect)   │
                    └────────┬─────────┘
                             ↓
            ┌────────────────────────────────────┐
            │ Marker Positions & Transforms      │
            │ /fiducial_vertices                 │
            │ /fiducial_transforms               │
            └────────────────┬───────────────────┘
                             ↓
                    ┌──────────────────┐
                    │  IBVS Control    │
                    │ (sub_ibvs)       │
                    │ - Feature Error  │
                    │ - Jacobian       │
                    │ - Control Law    │
                    └────────┬─────────┘
                             ↓
            ┌────────────────────────────────────┐
            │   Velocity Commands                │
            │   /mavros/setpoint_raw/local       │
            └────────────────┬───────────────────┘
                             ↓
                    ┌──────────────────┐
                    │ MAVRos Bridge    │
                    │ (PX4 Autopilot)  │
                    └────────┬─────────┘
                             ↓
                    ┌──────────────────┐
                    │  Drone Motors    │
                    └──────────────────┘
```

## Core Components

### 1. ArUco Detector (`aruco_detect_node`)

**Responsibility**: Real-time marker detection and pose estimation

**Inputs**:
- RGB camera stream: `/camera/rgb/image_raw`
- Depth stream: `/camera/depth/image_raw`
- Camera calibration: `/camera/rgb/camera_info`

**Outputs**:
- Marker vertices: `/fiducial_vertices` (image coordinates)
- Marker transforms: `/fiducial_transforms` (3D pose)
- Annotated images: `/fiducial_images`

**Key Operations**:
1. Convert image to grayscale
2. Detect ArUco markers using OpenCV
3. Estimate 3D pose using `cv::aruco::estimatePoseSingleMarkers()`
4. Publish marker features and transforms
5. Broadcast transform frames

**Parameters**:
- `marker_size`: Physical marker size in cm
- `aruco_dictionary`: Dictionary selection (DICT_4X4_50, etc.)
- `marker_frame_id`: TF frame name for markers
- `camera_frame_id`: Camera frame name

### 2. IBVS Control (`sub_ibvs`)

**Responsibility**: Visual servoing control computation and drone command generation

**Inputs**:
- Marker positions: `/fiducial_vertices`
- Marker transforms: `/fiducial_transforms`
- Camera info: `/camera/rgb/camera_info`
- Drone state: `/mavros/state`
- Drone pose: `/mavros/local_position/pose`
- Images: `/fiducial_images`

**Outputs**:
- Velocity commands: `/mavros/setpoint_raw/local`
- Position setpoints: `/mavros/setpoint_position/local`
- Analysis data: `/sub_ibvs/analysis`
- Output video: `/sub_ibvs/output_video`

**Key Functions**:

#### `compute_ctrl_law()`
Calculates control errors and command velocities using:
- **Image moments**: Geometric features of marker
- **Jacobian matrix**: Relationship between image features and camera motion
- **Control gain**: Adaptive lambda based on error magnitude
- **Feature error**: Distance to desired positions

#### `get_center_moments()`
Computes centered image moments:
- m_00: Number of points
- m_10, m_01: Center of mass
- m_20, m_02, m_11: Second-order moments for shape analysis

#### `compute_jacobian()`
Builds the interaction matrix relating feature velocities to camera velocities

#### `send_ctrl_signal()`
Formats and publishes velocity commands to drone:
- velocity.x, velocity.y, velocity.z: Linear velocities (m/s)
- yaw_rate: Angular velocity (rad/s)

### 3. Coordinate Transformations

**Camera Frame**:
- Origin: Camera optical center
- X-axis: Right in image
- Y-axis: Down in image
- Z-axis: Out of camera (depth)

**Image Coordinates**:
- (x0, y0): Top-left corner
- (x1, y1): Top-right corner
- (x2, y2): Bottom-right corner
- (x3, y3): Bottom-left corner

**Normalized Coordinates** (used in control):
```
x_normalized = (x_pixel - u0) / f_x
y_normalized = (y_pixel - v0) / f_y
```
Where: u0, v0 = principal point; f_x, f_y = focal lengths

## Data Flow Details

### Marker Detection Pipeline

```
Raw Image
    ↓
Convert to Grayscale
    ↓
Detect Markers (cv::aruco::detectMarkers)
    ↓
├─ Get corner coordinates (pixels)
├─ Estimate pose (3D position & rotation)
└─ Draw visualizations
    ↓
Publish:
  - Fiducial message (vertices)
  - Transform message (3D pose)
  - Annotated image
```

### Control Computation Pipeline

```
Marker Vertices (pixels)
    ↓
Normalize to camera frame
    ↓
Compute moments (center, area, orientation)
    ↓
Build Jacobian matrix
    ↓
Compute feature error
    ↓
Calculate control gain (lambda)
    ↓
Command = -lambda * J_pinv * error
    ↓
Publish velocity command
```

## Message Flow Timing

```
Time T:
  Camera captures frame
      ↓ (0-5ms)
  ArUco detection completes
      ↓ (publish)
  IBVS receives markers
      ↓ (0-2ms)
  Control law computed
      ↓ (0-1ms)
  Command published
      ↓ (5-10ms)
  Drone receives & actuates
      ↓ (20-50ms typical latency)
  Drone responds

Total Loop Time: ~30-70ms @ 20Hz
```

## State Variables

### Initialization Flags
- `k_flag`: Camera calibration received
- `c_flag`: Transform data received
- `m_flag`: Marker data received
- `first_detection`: First marker detection done

### Control State
- `marker1`: Currently tracking marker ID 0 (true) or marker ID 7 (false)
- `takeoff`: Takeoff phase complete
- `takeoff_time_init`: Takeoff initialization done

### Geometric Variables
- `x_g, y_g`: Center of mass of marker
- `xd_g, yd_g`: Desired center of mass
- `a_d, a_n`: Desired and actual area
- `x_n, y_n`: Normalized features

### Matrices
- `J_combined`: Combined interaction matrix
- `J_pinv`: Pseudo-inverse of J
- `p_desired_transformed`: Desired positions in normalized coords
- `centered_m_list`: Image moment matrix
- `centered_m_list_d`: Desired moment matrix

## Control Gains

```cpp
float min_lambda = 1.25;    // Low error (converged)
float max_lambda = 0.5;     // High error (diverged)

lambda = (max_lambda - min_lambda) * (norm_err / norm_err_max) + min_lambda
```

Adaptive gain: Slower response when far, faster when close to minimize overshoot.

## Error Definition

Control error vector (4-element):
```
err[0] = x_n - z_d * xd_g       (X position error)
err[1] = y_n - z_d * yd_g       (Y position error)  
err[2] = a_n - z_d              (Area/scale error)
err[3] = alpha - alpha_d         (Orientation error)
```

Where z_d is desired depth and alpha is marker rotation angle.

## Transition Logic

**Marker 1 → Marker 2 Transition**:
```
if (error < 0.06 threshold AND marker1_tracking) {
  marker1 = false
  Reduce desired window size (100px → 50px)
  Reduce altitude (0.6m → 0.15m)
  Reinitialize control variables
}
```

**Landing Logic**:
```
if (!marker1 AND error < 0.015 AND drone.armed) {
  Send land command (MAV_CMD_COMPONENT_ARM_DISARM)
  Shutdown system
}
```
