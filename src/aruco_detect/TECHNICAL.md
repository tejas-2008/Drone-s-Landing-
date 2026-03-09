# ArUco Detect Package - Technical Documentation

## System Overview

```
┌─────────────────────────────────────────────────┐
│            Camera Hardware (RGBD)               │
│  - RGB Image Stream                             │
│  - Depth Image Stream                           │
│  - Camera Calibration Info                      │
└──────────────────┬──────────────────────────────┘
                   │
                   ▼
        ┌──────────────────────┐
        │   ArUco Detector     │
        │   (this package)     │
        └──────────┬───────────┘
                   │
         ┌─────────┼─────────┐
         │         │         │
         ▼         ▼         ▼
    Vertices  Transforms  Image
    (FiducialArray)
         │         │         │
         └─────────┴─────────┘
                   │
                   ▼
        ┌──────────────────────┐
        │   sub_ibvs.cpp       │
        │   (IBVS Controller)  │
        └──────────────────────┘
                   │
                   ▼
        ┌──────────────────────┐
        │   Drone (PX4/APM)    │
        │   Motor Commands     │
        └──────────────────────┘
```

## Topic Mapping

### Input Topics (Subscribed by aruco_detect_node)

| Topic | Type | Rate | Purpose |
|-------|------|------|---------|
| `/camera/rgb/image_raw` | sensor_msgs/Image | 30Hz | Color image for marker detection |
| `/camera/depth/image_raw` | sensor_msgs/Image | 30Hz | Depth image (prepared for future) |
| `/camera/rgb/camera_info` | sensor_msgs/CameraInfo | 30Hz | Camera calibration (K, D) |

### Output Topics (Published by aruco_detect_node)

| Topic | Type | Rate | Purpose |
|-------|------|------|---------|
| `fiducial_vertices` | fiducial_msgs/FiducialArray | 30Hz | 2D marker corners in pixels |
| `fiducial_transforms` | fiducial_msgs/FiducialTransformArray | 30Hz | 3D marker pose (relative to camera) |
| `/fiducial_images` | sensor_msgs/Image | 30Hz | Visualization with detected markers |
| `/camera/rgb/camera_info` | sensor_msgs/CameraInfo | 30Hz | Rebroadcast of camera calibration |

### TF Transforms (Broadcast by aruco_detect_node)

| Parent Frame | Child Frame | Description |
|--------------|-------------|-------------|
| `camera_rgb_optical_frame` | `aruco_marker_0` | Transform to marker 0 |
| `camera_rgb_optical_frame` | `aruco_marker_1` | Transform to marker 1 |
| ... | ... | One frame per detected marker |

## Message Format Details

### FiducialArray (fiducial_vertices)

This message contains the 2D pixel coordinates of detected markers.

```cpp
struct Fiducial {
    uint32 fiducial_id        // Marker ID
    float64 x0, y0           // Corner 0 (top-left)
    float64 x1, y1           // Corner 1 (top-right)
    float64 x2, y2           // Corner 2 (bottom-right)
    float64 x3, y3           // Corner 3 (bottom-left)
}

struct FiducialArray {
    std_msgs/Header header
    Fiducial[] fiducials
}
```

**Used in sub_ibvs.cpp:**
```cpp
void markerCallback(const fiducial_msgs::FiducialArray fva) {
    this->current_markers = fva.fiducials.size();
    for (size_t i=0; i<fva.fiducials.size(); i++) {
        this->fid[i].fiducial_id = fva.fiducials[i].fiducial_id;
        this->fid[i].x0 = fva.fiducials[i].x0;
        this->fid[i].y0 = fva.fiducials[i].y0;
        // ... other corners
    }
    m_flag = true;
}
```

### FiducialTransformArray (fiducial_transforms)

This message contains the 3D pose of detected markers relative to camera frame.

```cpp
struct FiducialTransform {
    uint32 fiducial_id
    geometry_msgs/Transform transform  // [x, y, z, qx, qy, qz, qw]
}

struct FiducialTransformArray {
    std_msgs/Header header
    FiducialTransform[] transforms
}
```

**Used in sub_ibvs.cpp:**
```cpp
void transformCallback(const fiducial_msgs::FiducialTransformArray fta) {
    for (size_t i=0; i<fta.transforms.size(); i++) {
        this->ft->fiducial_id = fta.transforms[i].fiducial_id;
        this->ft->transform = fta.transforms[i].transform;
    }
    c_flag = true;
}
```

## Detection Pipeline

```
Input RGB Image (640x480 typically)
      │
      ▼
Convert to Grayscale
      │
      ▼
ArUco Dictionary Lookup (DICT_4X4_50)
      │
      ▼
Marker Detection
      ├─ Detect contours
      ├─ Filter by aspect ratio
      ├─ Decode marker bits
      └─ Get corner locations (sub-pixel)
      │
      ▼
For Each Detected Marker:
      ├─ Create Fiducial message (2D corners)
      ├─ Estimate 3D pose using solvePnP
      │  (requires camera matrix K and distortion D)
      ├─ Create FiducialTransform message
      ├─ Broadcast TF transform
      └─ Draw on visualization image
      │
      ▼
Publish All Topics
```

## Code Architecture

### Main Components

1. **ArucoDetector Class** (`aruco_detect_node.cpp`)
   - Initializes ROS node and topic subscribers/publishers
   - Manages camera calibration
   - Implements detection callback

2. **Image Callback** (`imageCallback`)
   - Receives synchronized RGB/Depth/CameraInfo
   - Performs marker detection
   - Estimates 3D poses
   - Publishes output messages

3. **Message Synchronization** (message_filters)
   - Synchronizes RGB, depth, and camera info
   - Ensures consistent data across callbacks
   - Drops messages if synchronization fails

### Key Processing Functions

```cpp
// Detect markers in grayscale image
cv::aruco::ArucoDetector detector(dictionary_, parameters_);
detector.detectMarkers(gray, corners, ids);

// Estimate 3D pose for each marker
cv::aruco::estimatePoseSingleMarkers(
    marker_corners,      // 2D corner points
    marker_size_,        // Physical marker size
    camera_matrix_,      // Camera intrinsics K
    dist_coeffs_,        // Distortion coefficients
    rvecs,              // Rotation vectors (output)
    tvecs               // Translation vectors (output)
);

// Convert rotation vector to quaternion
cv::Rodrigues(rvecs[i], rotation_matrix);
// ... matrix to quaternion conversion
```

## Coordinate Frames

### Camera Frame
- **Origin**: Camera optical center
- **X-axis**: Right in image
- **Y-axis**: Down in image
- **Z-axis**: Forward (away from camera)

### Marker Frame
- **Origin**: Marker center
- **X-axis**: Right edge of marker
- **Y-axis**: Up edge of marker
- **Z-axis**: Out of marker plane

### Transform Direction
All transforms published from `camera_rgb_optical_frame` to `aruco_marker_*`

## Camera Calibration

The package requires camera calibration matrix and distortion coefficients.

### Camera Matrix (K)
```
[fx  0  cx]
[ 0 fy  cy]
[ 0  0   1]
```
Where:
- `fx`, `fy`: Focal length in pixels
- `cx`, `cy`: Principal point (image center) in pixels

### Distortion Coefficients (D)
```
[k1, k2, p1, p2, k3]
```
Where:
- `k1`, `k2`, `k3`: Radial distortion
- `p1`, `p2`: Tangential distortion

These are obtained from camera calibration tools (e.g., ROS calibration_camera_checkerboard).

## Performance Characteristics

### Computation
- Marker detection: ~10-20ms
- Pose estimation: ~5-10ms
- Total per frame: ~20-30ms

### Accuracy
- 2D corner localization: ±1 pixel
- 3D pose estimation: ±1-2cm (depends on marker size and distance)
- Orientation: ±1-2 degrees

### Constraints
- Minimum distance: ~10cm from camera
- Maximum distance: Depends on marker size and resolution
- Lighting: Well-lit environment required
- Marker dictionary: Currently DICT_4X4_50

## Integration Points with sub_ibvs

```
sub_ibvs.cpp expects:
├─ markerCallback receives fiducial_vertices
│  └─ Extracts 2D corner positions → Uses in IBVS error computation
├─ transformCallback receives fiducial_transforms
│  └─ Extracts 3D pose → Uses for depth in control law
└─ c_infoCallback receives camera_info
   └─ Extracts K matrix → Used for pixel-to-world conversion
```

## Future Enhancements

1. **Depth-based Filtering**
   - Use depth image to filter detections
   - Improve robustness in cluttered scenes

2. **Adaptive Parameters**
   - Adjust detection thresholds based on image quality
   - Auto-tune marker size for accuracy

3. **Multi-dictionary Support**
   - Support different ArUco dictionaries
   - Dynamic dictionary selection

4. **Performance Optimization**
   - GPU acceleration for detection
   - Parallel processing for multi-marker scenes

5. **Extended Output**
   - Publish confidence/quality metrics
   - Include corner uncertainty estimation

## Troubleshooting Matrix

| Symptom | Cause | Solution |
|---------|-------|----------|
| No markers detected | Poor lighting | Increase lighting or adjust camera exposure |
| | Marker not in frame | Check camera view |
| | Wrong dictionary | Verify DICT_4X4_50 is used |
| Unstable detections | Camera shake | Improve camera mounting |
| | Out of focus | Adjust camera focus |
| | Reflective surface | Use matte markers |
| Incorrect poses | Uncalibrated camera | Re-run camera calibration |
| | Wrong marker size | Update `marker_size` parameter |
| | Distorted markers | Print markers on flat surface |
| No output topics | Node not running | Check `roslaunch` output |
| | Topic remapping | Check launch file topic names |
| | Message filter timeout | Increase sync queue size |

## References

- OpenCV ArUco: https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html
- ROS Message Filters: http://wiki.ros.org/message_filters
- Camera Calibration: http://wiki.ros.org/camera_calibration
