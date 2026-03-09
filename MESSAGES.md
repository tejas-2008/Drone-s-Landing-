# Message Types Reference

## Custom Messages

### `ibvs_msgs/Signal.msg`

**Purpose**: Single control signal component

**Fields**:
```
float32 v1
float32 v2
float32 v3
float32 v4
```

**Usage**: Represents 4 elements of error or command vector
- Index 0: X-component
- Index 1: Y-component
- Index 2: Area/Z-component
- Index 3: Rotation

---

### `ibvs_msgs/SignalArray.msg`

**Purpose**: Complete IBVS control analysis packet

**Fields**:
```
Signal err_signal
Signal ctrl_signal
float32 norm_err
```

**Description**:
- `err_signal`: Error vector [x_err, y_err, area_err, angle_err]
- `ctrl_signal`: Velocity command [v_x, v_y, v_z, yaw_rate]
- `norm_err`: L2 norm of error (scalar for convergence monitoring)

**Published by**: `sub_ibvs`  
**Topic**: `/sub_ibvs/analysis`  
**Frequency**: 20 Hz

**Example Use**:
```cpp
// In analysis_callback
float error_magnitude = msg->norm_err;
if (error_magnitude < threshold) {
  // Converged
}
```

---

## Standard Messages Used

### `fiducial_msgs/Fiducial.msg`

**Purpose**: Single detected marker with corner positions

**Fields**:
```
int32 fiducial_id
float64 x0
float64 y0
float64 x1
float64 y1
float64 x2
float64 y2
float64 x3
float64 y3
```

**Description**:
- Marker corner coordinates in **image pixels**
- (x0,y0): Top-left
- (x1,y1): Top-right
- (x2,y2): Bottom-right
- (x3,y3): Bottom-left
- **Origin**: Image top-left corner

**Published by**: `aruco_detect_node`  
**Aggregated in**: `FiducialArray`

---

### `fiducial_msgs/FiducialArray.msg`

**Purpose**: Collection of detected markers

**Fields**:
```
std_msgs/Header header
Fiducial[] fiducials
```

**Header Contents**:
- `seq`: Sequence number
- `stamp`: Timestamp (same as image)
- `frame_id`: Camera frame ID

**Published by**: `aruco_detect_node`  
**Topic**: `/fiducial_vertices`

---

### `fiducial_msgs/FiducialTransform.msg`

**Purpose**: Single marker with 3D pose

**Fields**:
```
int32 fiducial_id
geometry_msgs/Transform transform
```

**Transform Breakdown**:
```
translation:
  x: float64 (centimeters)
  y: float64 (centimeters)
  z: float64 (centimeters)
rotation:
  x: float64 (quaternion X)
  y: float64 (quaternion Y)
  z: float64 (quaternion Z)
  w: float64 (quaternion W)
```

**Coordinate System**:
- **Origin**: Camera optical center
- **X-axis**: Right in image plane
- **Y-axis**: Down in image plane
- **Z-axis**: Forward from camera
- **Units**: Centimeters

**Quaternion**: Represents rotation from camera frame to marker frame

---

### `fiducial_msgs/FiducialTransformArray.msg`

**Purpose**: Collection of marker poses

**Fields**:
```
std_msgs/Header header
FiducialTransform[] transforms
```

**Published by**: `aruco_detect_node`  
**Topic**: `/fiducial_transforms`  
**Frequency**: ~30 Hz

---

### `mavros_msgs/PositionTarget.msg`

**Purpose**: Raw position/velocity setpoint (used for IBVS)

**Key Fields**:
```
uint8 coordinate_frame    # 8 = FRAME_BODY_NED
uint16 type_mask          # Specifies which fields to use

geometry_msgs/Vector3 position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration_or_force

float32 yaw
float32 yaw_rate
```

**Type Mask Values**:
```
1   = ignore position
2   = ignore velocity
4   = ignore acceleration
8   = ignore force (use acceleration)
16  = ignore yaw
32  = ignore yaw rate
```

**For IBVS Control** (type_mask = 1991):
- Ignore position (bit 0)
- Use velocity (bit 1 clear)
- Ignore acceleration (bit 2)
- Use yaw_rate (bit 5 clear)
- Other bits ignored

**Published by**: `sub_ibvs`  
**Topic**: `/mavros/setpoint_raw/local`  
**Frequency**: 20 Hz minimum

---

### `sensor_msgs/CameraInfo.msg`

**Purpose**: Camera calibration data

**Key Fields**:
```
std_msgs/Header header
uint32 height          # Image height in pixels
uint32 width           # Image width in pixels

string distortion_model
float64[5] D           # Distortion coefficients

float64[9] K           # Camera intrinsic matrix (3x3)
              # [f_x   0  u_0]
              # [ 0  f_y  v_0]
              # [ 0    0    1]

float64[12] P          # Projection matrix (3x4)
float64[9] R           # Rectification matrix
```

**Usage in Code**:
```cpp
// Extract focal lengths
f_x = K[0];  // K(0,0)
f_y = K[4];  // K(1,1)

// Extract principal point
u_0 = K[2];  // K(0,2)
v_0 = K[5];  // K(1,2)

// Normalize image coordinates
x_norm = (x_pixel - u_0) / f_x;
y_norm = (y_pixel - v_0) / f_y;
```

**Published by**: Camera driver  
**Subscribed by**: Both `aruco_detect_node` and `sub_ibvs`

---

## Message Timing & Synchronization

### Synchronization Strategy

**aruco_detect_node** uses `message_filters::TimeSynchronizer`:

```
Image stream
    ↓
  Queue (100 messages)
    ↓
+─→ Synchronize RGB
+─→ Synchronize Depth
+─→ Synchronize CameraInfo (with time window)
    ↓
imageCallback() triggered
```

**Tolerance**: ~100ms window (queue size = 100 at 30Hz)

### Publication Order

**Within imageCallback() of aruco_detect_node**:
1. Detect markers
2. Publish `/fiducial_vertices`
3. Publish `/fiducial_transforms`
4. Broadcast TF transforms
5. Publish `/fiducial_images`
6. Publish `/camera/rgb/camera_info`

**All typically <1ms apart**

### Downstream Processing

**sub_ibvs receives**:
1. `/fiducial_vertices` → sets `m_flag = true`
2. `/fiducial_transforms` → sets `c_flag = true`
3. `/camera/rgb/camera_info` → sets `k_flag = true` (once)

**Control executes when**:
```cpp
if (c_flag && k_flag && m_flag) {
  // Guaranteed all data is synchronized
  marker_selection();
  m_flag = false;  // Require new marker data for next iteration
  c_flag = false;
}
```

---

## Coordinate System Conversions

### Camera to Normalized Coordinates

**Image Pixel** → **Camera Frame Normalized**
```
x_norm = (x_pixel - u_0) / f_x
y_norm = (y_pixel - v_0) / f_y
```

Where:
- u_0, v_0: Principal point (image center offset)
- f_x, f_y: Focal lengths in pixels

### 3D Pose from Transform

**Extract Position**:
```cpp
float x_camera = transform.translation.x / 100.0;  // cm to m
float y_camera = transform.translation.y / 100.0;
float z_camera = transform.translation.z / 100.0;
```

**Extract Rotation**:
```cpp
// Quaternion components
float qx = transform.rotation.x;
float qy = transform.rotation.y;
float qz = transform.rotation.z;
float qw = transform.rotation.w;

// Convert to rotation matrix for further use
// (not done in current code, but available)
```

### Body Frame Velocity Mapping

**Camera Frame** → **Body Frame NED**:
```cpp
// Camera uses standard image convention
// Body uses NED (forward, right, down)
vel.velocity.x = -cmd[1];  // Left/Right → Forward/Back
vel.velocity.y = -cmd[0];  // Forward/Back → Left/Right
vel.velocity.z = -cmd[2];  // Up/Down (negated for NED)
vel.yaw_rate = -cmd[3];    // Yaw rotation
```

---

## Important Notes on Units

| Component | Unit | Notes |
|-----------|------|-------|
| Marker positions (vertices) | Pixels | Image coordinates |
| Marker transforms XYZ | Centimeters | From `estimatePoseSingleMarkers()` |
| Drone position | Meters | MAVRos convention |
| Velocity commands | m/s | Body frame |
| Quaternion | - | Unit quaternion (normalized) |
| Angles | Radians | Throughout all calculations |

**⚠️ UNIT MISMATCH WARNING**: 
- ArUco outputs centimeters but MAVRos expects meters
- **Fix required**: Divide transform.translation by 100 if using for drone positioning
- Control velocities are scale-invariant (pure direction matters)

---

## Message Latency Budget

```
Camera capture: T=0
    ↓ (10ms)
ROS message timestamp: T=10ms
    ↓ (5ms)
aruco_detect publishes: T=15ms
    ↓ (1ms)
sub_ibvs receives (after spinOnce): T=16ms
    ↓ (2ms)
Control computation: T=18ms
    ↓ (1ms)
Publish velocity command: T=19ms
    ↓ (10-20ms network/serial)
Autopilot receives: T=30-40ms
    ↓ (20-50ms)
Drone actuates
```

**Total Loop Latency**: 30-70ms  
**Effective Control Rate**: ~15-20 Hz (including processing delays)
