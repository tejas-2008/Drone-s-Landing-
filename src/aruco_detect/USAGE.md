# ArUco Detector - Usage Guide

## Quick Start

### 1. Build the package
```bash
cd ~/dev_ws
catkin build aruco_detect
source devel/setup.bash
```

### 2. Launch the detector
```bash
roslaunch aruco_detect aruco_detect.launch
```

### 3. Verify output topics
```bash
rostopic list | grep fiducial
# Should show:
# /fiducial_images
# /fiducial_transforms
# /fiducial_vertices
# /camera/rgb/camera_info
```

## Checking Detection Output

### View detected marker corners
```bash
rostopic echo /fiducial_vertices
```

**Example output:**
```
header: 
  seq: 123
  stamp: 
    secs: 1234567890
    nsecs: 123456789
  frame_id: "camera_rgb_optical_frame"
fiducials: 
  - 
    fiducial_id: 0
    x0: 100.5
    y0: 150.2
    x1: 200.3
    y1: 150.1
    x2: 200.4
    y2: 250.5
    x3: 100.6
    y3: 250.6
```

### View marker poses
```bash
rostopic echo /fiducial_transforms
```

**Example output:**
```
header: 
  seq: 124
  stamp: 
    secs: 1234567890
    nsecs: 125000000
  frame_id: "camera_rgb_optical_frame"
transforms: 
  - 
    fiducial_id: 0
    transform: 
      translation: 
        x: 0.15
        y: -0.02
        z: 0.5
      rotation: 
        x: 0.707
        y: 0.0
        z: 0.0
        w: 0.707
```

### View visualization
```bash
# Using rqt_image_view
rosrun rqt_image_view rqt_image_view /fiducial_images

# Or using command line
rostopic hz /fiducial_images  # Check publication rate
```

## TF Transforms

View published TF tree:
```bash
rosrun tf view_frames
# Then view generated PDF
evince frames.pdf
```

Individual marker transforms can be queried:
```bash
rosrun tf tf_echo camera_rgb_optical_frame aruco_marker_0
```

## Integration with sub_ibvs

The IBVS controller automatically subscribes to the output topics:

```bash
# Run both nodes
roslaunch aruco_detect aruco_detect.launch &
roslaunch drone_ibvs sub_ibvs.launch
```

Monitor IBVS control signals:
```bash
rostopic echo /sub_ibvs/analysis
```

## Debugging

### 1. Check node status
```bash
rosnode list
rosnode info aruco_detect_node
```

### 2. Enable debug logging
```bash
# In launch file, change output="screen" to output="log"
# Or set log level:
export ROSCONSOLE_FORMAT='[${severity}] [${node}] [${time}]: ${message}'
export ROSCONSOLE_DEFAULT_CONFIG_FILE=~/.ros/rosconsole.config
```

### 3. Monitor frame rate
```bash
rostopic hz /fiducial_vertices
```

### 4. Check camera calibration
```bash
rostopic echo /camera/rgb/camera_info
# Look for K matrix (camera matrix) and D coefficients
```

### 5. Diagnose topic latency
```bash
rosrun tf show_tf_latency /fiducial_vertices
```

## Parameter Tuning

### Marker Size
If marker poses seem incorrect, verify marker size matches physical markers:
```xml
<param name="marker_size" value="5"/>  <!-- Change to your marker size in cm -->
```

Standard sizes:
- 5cm: Small markers for close-range detection
- 10cm: Medium markers for typical range
- 20cm: Large markers for long-range detection

### Detection Tuning (in code)
If detection is unreliable, you can modify parameters in `aruco_detect_node.cpp`:
```cpp
parameters_ = cv::aruco::DetectorParameters();
// Adjust thresholds as needed
```

## Common Issues & Solutions

### Issue: "No markers detected"
```bash
# Check image quality
rostopic hz /camera/rgb/image_raw
# Should be 30Hz or higher

# View raw image
rosrun rqt_image_view rqt_image_view /camera/rgb/image_raw
# Ensure markers are visible

# Check camera info
rostopic echo /camera/rgb/camera_info | head -20
```

### Issue: "Messages arriving out of order"
```bash
# Increase time sync queue size in launch file
# Change <param> value in aruco_detect.launch
```

### Issue: "Unstable pose estimates"
```bash
# 1. Re-calibrate camera
# 2. Ensure steady camera mount
# 3. Increase lighting
# 4. Verify marker_size parameter
```

## Performance Benchmarking

```bash
# Check detection rate
rostopic hz /fiducial_vertices

# Check latency
rostopic info /fiducial_vertices

# CPU usage
top -p $(pgrep aruco_detect_node)

# Memory usage
ps aux | grep aruco_detect_node
```

Expected performance on modern hardware:
- 30+ Hz detection rate
- <50ms latency
- <5% CPU per core
- <100MB RAM

## Connecting to Gazebo Simulation

For simulation with Gazebo:
```xml
<!-- In your Gazebo world, add a camera plugin -->
<!-- This will publish to /camera/rgb/image_raw, etc. -->
```

Then launch the detector:
```bash
roslaunch aruco_detect aruco_detect.launch
```

## Recording Data

Record detection for analysis:
```bash
rosbag record /fiducial_vertices /fiducial_transforms /camera/rgb/image_raw -o detection.bag

# Playback later
rosbag play detection.bag
```

## Next Steps

1. **Integrate with IBVS**: Run both aruco_detect and sub_ibvs nodes
2. **Tune control parameters**: See sub_ibvs.cpp for IBVS controller tuning
3. **Optimize performance**: Profile for your hardware
4. **Handle multiple markers**: Configure marker selection strategy in IBVS
