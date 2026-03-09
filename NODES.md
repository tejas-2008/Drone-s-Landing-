# ROS Nodes Reference

## ArUco Detect Node

**Node Name**: `aruco_detect_node`  
**Package**: `aruco_detect`  
**Type**: Sensor/Detection

### Description
Detects ArUco markers in camera images and estimates their 3D poses relative to the camera frame.

### Published Topics

#### `/fiducial_vertices`
- **Type**: `fiducial_msgs/FiducialArray`
- **Frequency**: ~30 Hz (camera dependent)
- **Description**: Marker corner positions in image coordinates
- **Fields**:
  - `fiducial_id`: Marker ID number
  - `x0, y0`: Top-left corner (pixels)
  - `x1, y1`: Top-right corner
  - `x2, y2`: Bottom-right corner
  - `x3, y3`: Bottom-left corner

#### `/fiducial_transforms`
- **Type**: `fiducial_msgs/FiducialTransformArray`
- **Frequency**: ~30 Hz
- **Description**: 3D pose of each marker relative to camera
- **Fields**:
  - `fiducial_id`: Marker ID
  - `transform.translation.x/y/z`: Position in **centimeters**
  - `transform.rotation.x/y/z/w`: Quaternion orientation

#### `/fiducial_images`
- **Type**: `sensor_msgs/Image`
- **Frequency**: ~30 Hz
- **Description**: Annotated image with detected markers and axes drawn

#### `/camera/rgb/camera_info`
- **Type**: `sensor_msgs/CameraInfo`
- **Frequency**: ~1 Hz
- **Description**: Re-published camera calibration data

### Subscribed Topics

#### `/camera/rgb/image_raw`
- **Type**: `sensor_msgs/Image`
- **Description**: RGB camera stream (BGR8 format)

#### `/camera/depth/image_raw`
- **Type**: `sensor_msgs/Image`
- **Description**: Depth stream

#### `/camera/rgb/camera_info`
- **Type**: `sensor_msgs/CameraInfo`
- **Description**: Camera calibration matrix and distortion coefficients

### Parameters

```yaml
# Marker size in centimeters
marker_size: 20

# ArUco dictionary to use
aruco_dictionary: DICT_4X4_50

# TF frame names
marker_frame_id: aruco_marker
camera_frame_id: camera_link
```

### Configuration File
Location: `src/aruco_detect/config/aruco_detect.yaml`

### Implementation Details

**Class**: `ArucoDetector`

**Key Methods**:
- `imageCallback()`: Main processing function
  - Synchronizes RGB, depth, and camera info messages
  - Runs marker detection on grayscale image
  - Estimates pose for all markers
  - Publishes results and transforms

**Output Coordinate System**:
- **Units**: Centimeters for XYZ position
- **Origin**: Camera optical center
- **Z-axis**: Pointing forward from camera

**Algorithm**:
1. Convert image to grayscale
2. Call `cv::aruco::detectMarkers()` with selected dictionary
3. For each detected marker:
   - Extract corner coordinates
   - Call `cv::aruco::estimatePoseSingleMarkers()` to get pose
   - Convert rotation vector to quaternion
   - Publish transform

**Transform Broadcast**:
- Publishes TF: camera → marker
- Frame ID: `{marker_frame_id}_{marker_id}`
- Updates at detection frequency

---

## IBVS Control Node

**Node Name**: `sub_ibvs`  
**Package**: `Drone_IBVS`  
**Type**: Control/Planning

### Description
Implements Image-Based Visual Servoing control to track ArUco markers and generate drone velocity commands.

### Published Topics

#### `/mavros/setpoint_position/local`
- **Type**: `geometry_msgs/PoseStamped`
- **Frequency**: 20 Hz
- **Description**: Desired position setpoint (during takeoff)
- **Fields**:
  - `pose.position.x/y/z`: Target position in meters

#### `/mavros/setpoint_raw/local`
- **Type**: `mavros_msgs/PositionTarget`
- **Frequency**: 20 Hz
- **Description**: Velocity commands (during control)
- **Fields**:
  - `coordinate_frame`: 8 (FRAME_BODY_NED)
  - `type_mask`: 1991 (velocity + yaw_rate)
  - `velocity.x/y/z`: Body-frame velocity (m/s)
  - `yaw_rate`: Angular velocity (rad/s)

#### `/sub_ibvs/analysis`
- **Type**: `ibvs_msgs/SignalArray`
- **Frequency**: 20 Hz
- **Description**: Control signal analysis (errors and commands)

#### `/sub_ibvs/output_video`
- **Type**: `sensor_msgs/Image`
- **Frequency**: 30 Hz (image dependent)
- **Description**: Annotated video with desired markers drawn

### Subscribed Topics

#### `/fiducial_vertices`
- **Type**: `fiducial_msgs/FiducialArray`
- **Description**: Current marker corner positions

#### `/fiducial_transforms`
- **Type**: `fiducial_msgs/FiducialTransformArray`
- **Description**: Current marker 3D poses

#### `/camera/rgb/camera_info`
- **Type**: `sensor_msgs/CameraInfo`
- **Description**: Camera calibration for coordinate conversions

#### `/mavros/state`
- **Type**: `mavros_msgs/State`
- **Description**: Drone connection status and mode

#### `/mavros/local_position/pose`
- **Type**: `geometry_msgs/PoseStamped`
- **Description**: Drone position and orientation

#### `/fiducial_images`
- **Type**: `sensor_msgs/Image`
- **Description**: Annotated images for visualization

### Control Loop

**Main Loop (20 Hz)**:
```cpp
while (ros::ok()) {
  if (!takeoff_complete) {
    drone_takeOff(0.2, 0.2, 2.0);  // Hover at 2m
  } else {
    marker_selection();             // Run IBVS control
  }
  ros::spinOnce();
}
```

### Execution Stages

#### Stage 1: Takeoff
- **Duration**: Until altitude reaches 2.0m
- **Activity**: Send position setpoints
- **Command Topic**: `/mavros/setpoint_position/local`

#### Stage 2: Marker 1 Tracking
- **Marker ID**: 0 (large marker)
- **Window Size**: 100×100 pixels
- **Altitude**: 0.6m
- **Convergence Threshold**: error < 0.06
- **Command Topic**: `/mavros/setpoint_raw/local`

#### Stage 3: Marker 2 Tracking & Landing
- **Marker ID**: 7 (small marker)
- **Window Size**: 50×50 pixels
- **Altitude**: 0.15m
- **Landing Threshold**: error < 0.015
- **Command Topic**: `/mavros/setpoint_raw/local`

### Control Law

**Error Vector** (4-DOF):
```
e = [x_n - z_d*x_d, y_n - z_d*y_d, a_n - z_d, α - α_d]ᵀ
```

**Control Command**:
```
v_cmd = -λ(||e||) * J⁺ * e
```

Where:
- `J⁺`: Pseudo-inverse of 4×4 interaction matrix
- `λ`: Adaptive gain (1.25 @ low error → 0.5 @ high error)
- `v_cmd`: [v_x, v_y, v_z, ω_z] body velocities

### Key Classes & Methods

**Class**: `IBVS`

**Initialization**:
- `IBVS(ros::NodeHandle *nh)`: Constructor, sets up subscribers/publishers

**Callbacks**:
- `c_infoCallback()`: Extract camera matrix when received
- `markerCallback()`: Store current marker positions
- `transformCallback()`: Store current marker transforms
- `imageCb()`: Draw desired positions on image
- `drone_state_callback()`: Monitor connection status
- `drone_pose_callback()`: Track current position

**Control Functions**:
- `drone_takeOff()`: Handles takeoff phase with setpoint publishing
- `marker_selection()`: Selects which marker to track based on state
- `compute_ctrl_law()`: Calculates velocity commands from errors
- `compute_jacobian()`: Builds interaction matrix
- `get_center_moments()`: Computes image moments
- `init_desired_variables()`: Sets up desired feature set
- `send_ctrl_signal()`: Formats and publishes commands

### Velocity Mapping

**Body Frame Convention** (NED):
```
              Forward (X)
                   ↑
                   |
     Left (Y) ← Drone → Right (-Y)
                   |
                   ↓
              Backward (-X)
```

**Velocity Command Mapping**:
```cpp
vel.velocity.x = -v_cmd[1]     // Sideways motion
vel.velocity.y = -v_cmd[0]     // Forward/backward
vel.velocity.z = -v_cmd[2]     // Up/down
vel.yaw_rate = -v_cmd[3]       // Rotation
```

### State Flags

| Flag | Type | Purpose |
|------|------|---------|
| `k_flag` | bool | Camera calibration received |
| `c_flag` | bool | Transform data available |
| `m_flag` | bool | Marker positions available |
| `takeoff_time_init` | bool | Takeoff loop initialized |
| `first_detection` | bool | Used for error normalization |
| `marker1` | bool | Tracking stage (0→1 or 1→7) |
| `takeoff` | bool | Altitude setpoint reached |

---

## Alternative Publisher Node

**Node Name**: `offb_node`  
**Package**: `Drone_IBVS`  
**File**: `pub_ibvs.cpp`  
**Type**: Control (alternative)

### Description
Simple alternative that sends fixed velocity commands and angular rates to the drone. Used for testing/debugging.

### Published Topics
- `/mavros/setpoint_position/local`: Fixed position
- `/mavros/setpoint_velocity/cmd_vel`: Fixed velocity

### Key Differences from sub_ibvs
- No visual servoing computation
- Fixed velocity/rotation values
- No marker tracking
- Simpler state machine

---

## Topic Interdependencies

```
aruco_detect_node
  ├─ publishes → /fiducial_vertices
  ├─ publishes → /fiducial_transforms
  └─ publishes → /fiducial_images
                     ↓
                sub_ibvs
                  └─ publishes → /mavros/setpoint_raw/local
                                      ↓
                            MAVRos Bridge
                              ↓
                        Autopilot/Drone
```

## Frequency Recommendations

- **Marker Detection**: 20-30 Hz minimum
- **Control Loop**: 20 Hz minimum (PX4 expects >2 Hz)
- **Visualization**: 10 Hz (not time-critical)
- **Camera Info**: 1 Hz

## Message Synchronization

`aruco_detect_node` uses message filters to synchronize:
- RGB image
- Depth image  
- Camera info

**Synchronizer Queue Size**: 100 messages

`sub_ibvs` waits for all three flags before processing:
```cpp
if (c_flag && k_flag && m_flag) {
  // Process control
}
```
