# ArUco Detect Package

A ROS package for detecting ArUco markers in RGBD image streams and publishing fiducial information in the format expected by the IBVS controller.

## Overview

This package processes raw RGBD (RGB + Depth) data from a camera, detects ArUco markers, and publishes the detected marker information in a format compatible with the Drone IBVS (Image-Based Visual Servoing) system.

## Features

- **ArUco Marker Detection**: Uses OpenCV's ArUco module to detect predefined markers (DICT_4X4_50)
- **Pose Estimation**: Estimates 3D pose of detected markers using camera calibration
- **Multi-marker Support**: Handles multiple markers in a single frame
- **Time Synchronization**: Synchronizes RGB, depth, and camera info streams
- **TF Broadcasting**: Publishes marker poses as TF transforms
- **Visualization**: Publishes annotated images with detected markers and axes

## Package Structure

```
aruco_detect/
├── src/
│   └── aruco_detect_node.cpp    # Main detection node
├── launch/
│   ├── aruco_detect.launch      # Basic launch file
│   └── complete.launch          # Complete system launch
├── config/
│   └── aruco_detect.yaml        # Configuration parameters
├── CMakeLists.txt               # CMake build configuration
└── package.xml                  # ROS package metadata
```

## Published Topics

### `fiducial_vertices` (fiducial_msgs/FiducialArray)
Detected marker corner positions in image coordinates.

**Message Fields:**
- `header`: ROS header with timestamp and frame ID
- `fiducials[]`: Array of detected markers
  - `fiducial_id`: Marker ID
  - `x0, y0`: Top-left corner
  - `x1, y1`: Top-right corner
  - `x2, y2`: Bottom-right corner
  - `x3, y3`: Bottom-left corner

### `fiducial_transforms` (fiducial_msgs/FiducialTransformArray)
3D pose estimation of detected markers.

**Message Fields:**
- `header`: ROS header with timestamp
- `transforms[]`: Array of marker transforms
  - `fiducial_id`: Marker ID
  - `transform`: 3D transform relative to camera frame
    - `translation`: x, y, z position (meters)
    - `rotation`: Quaternion orientation

### `/fiducial_images` (sensor_msgs/Image)
Visualization image with detected markers drawn and coordinate axes displayed.

### `/camera/rgb/camera_info` (sensor_msgs/CameraInfo)
Rebroadcast of camera calibration information for reference.

## Subscribed Topics

### `/camera/rgb/image_raw` (sensor_msgs/Image)
Input RGB image stream (color image).

### `/camera/depth/image_raw` (sensor_msgs/Image)
Input depth image stream.

### `/camera/rgb/camera_info` (sensor_msgs/CameraInfo)
Camera calibration parameters (K matrix, distortion coefficients).

## Parameters

All parameters can be set in `config/aruco_detect.yaml` or passed via command line.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `marker_size` | int | 5 | Size of ArUco markers in centimeters |
| `marker_frame_id` | string | `aruco_marker` | Base frame ID for marker transforms |
| `camera_frame_id` | string | `camera_rgb_optical_frame` | Camera frame ID in TF tree |

## Building the Package

```bash
cd ~/dev_ws
catkin build aruco_detect
# or
catkin_make --only-pkg-with-deps aruco_detect
```

## Running the Package

### Basic launch
```bash
roslaunch aruco_detect aruco_detect.launch
```

### With custom parameters
```bash
roslaunch aruco_detect aruco_detect.launch marker_size:=10
```

## Dependencies

**ROS packages:**
- `roscpp`, `rospy`
- `cv_bridge`, `image_transport`
- `sensor_msgs`, `geometry_msgs`, `std_msgs`
- `fiducial_msgs`, `ibvs_msgs`
- `tf2`, `tf2_ros`, `tf2_geometry_msgs`
- `opencv3` (with ArUco support)
- `armadillo` (for numeric computations)

## Integration with IBVS Controller

The output topics from this package (`fiducial_vertices` and `fiducial_transforms`) are directly consumed by the [sub_ibvs.cpp](../Drone_IBVS/src/sub_ibvs.cpp) node through:

```cpp
vertices_sub = nh->subscribe("fiducial_vertices", 3, &IBVS::markerCallback, this);
transforms_sub = nh->subscribe("fiducial_transforms", 3, &IBVS::transformCallback, this);
```

## Camera Requirements

The package expects:
- **Calibrated RGB-D camera** (e.g., RealSense, Kinect)
- **Published camera calibration** via `/camera/rgb/camera_info`
- **Synchronized RGB and depth streams**

## Message Synchronization

The node uses ROS message_filters to synchronize:
1. RGB image (`/camera/rgb/image_raw`)
2. Depth image (`/camera/depth/image_raw`)
3. Camera info (`/camera/rgb/camera_info`)

This ensures all three messages are processed together for consistent pose estimation.

## Known Issues & Limitations

1. Marker detection is done on the **RGB image only** (depth is currently prepared for future use)
2. Requires well-lit environment for reliable marker detection
3. Marker size parameter must match the physical marker size for accurate pose estimation
4. Camera must be calibrated for accurate 3D pose estimation

## Performance Notes

- Typical detection latency: 20-50ms per frame
- Recommended camera resolution: 640x480 or higher
- Published at the rate of input image stream (typically 30Hz)

## Troubleshooting

### Markers not detected
- Ensure proper lighting and marker visibility
- Verify marker dictionary matches (currently set to `DICT_4X4_50`)
- Check that markers are in focus

### Inconsistent poses
- Recalibrate camera
- Ensure camera calibration is published correctly
- Verify marker size parameter is accurate

### Topic not found errors
- Check that camera is publishing to `/camera/rgb/image_raw` and `/camera/depth/image_raw`
- Verify `/camera/rgb/camera_info` is being published
- Use `rostopic list` to verify topics exist

## References

- [OpenCV ArUco Module](https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html)
- [fiducial_msgs Documentation](http://wiki.ros.org/fiducial_msgs)
- [IBVS Controller Package](../Drone_IBVS)

## Future Enhancements

- [ ] Depth-based filtering for more robust detection
- [ ] Multi-dictionary support
- [ ] Configurable ArUco detection parameters
- [ ] Performance optimization for real-time systems
- [ ] Support for marker filtering by distance/size
