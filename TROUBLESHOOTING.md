# Troubleshooting & Common Issues

## Marker Detection Issues

### Problem: "Detected 0 markers"

**Possible Causes**:

1. **Marker Not Visible**
   - Check camera view includes marker
   - Ensure adequate lighting
   - Verify marker is within focal distance

2. **Marker Size Mismatch**
   - Configured size doesn't match actual marker
   - Check `aruco_detect.yaml`:
     ```yaml
     marker_size: 20  # Must match physical marker in cm
     ```
   - If physical marker is 5cm, set to 5, not 20

3. **Wrong Dictionary**
   - Generated marker uses different dictionary
   - Check marker specification
   - Update `aruco_dictionary` in config:
     ```yaml
     aruco_dictionary: DICT_4X4_50
     ```

4. **Poor Camera Calibration**
   - Distortion coefficients not set
   - Camera matrix invalid
   - Re-run camera calibration

**Solution Steps**:
```bash
# 1. Check marker visibility
rosrun image_view image_view image:=/camera/rgb/image_raw

# 2. Verify ArUco node is publishing
rostopic echo /fiducial_vertices

# 3. Check parameter server
rosparam get /aruco_detect_node/marker_size

# 4. Review camera calibration
rostopic echo /camera/rgb/camera_info
```

---

### Problem: "Transform coordinates are invalid"

**Cause**: Detected marker corners, but pose estimation failed

**Common Reasons**:
1. Marker at extreme angle to camera
2. Partial marker occlusion
3. Marker too close or far from camera

**Fix**:
- Add bounds checking before using transforms
- Verify depth stream is working

---

## Unit Conversion Issues

### Problem: "Drone moving in wrong directions"

**Root Cause**: Units mismatch in transform usage

**Example Error**:
```cpp
// WRONG - mixing units
float x_m = ft->transform.translation.x;     // This is in CM!
drone_pos_pub.publish(pose with x_m);        // Publishing as meters
```

**Solution**:
```cpp
// CORRECT - convert cm to meters
float x_m = ft->transform.translation.x / 100.0;
float y_m = ft->transform.translation.y / 100.0;
float z_m = ft->transform.translation.z / 100.0;
```

### Problem: "YAML parameters not loading"

**Cause**: Incorrect parameter namespace in launch file

**Broken Launch File**:
```xml
<rosparam command="load" file="$(find aruco_detect)/config/aruco_detect.yaml" param="aruco_detect"/>
```
- This loads parameters under `/aruco_detect/` namespace
- Code looks in `/aruco_detect_node/` (private namespace)

**Fixed Launch File**:
```xml
<rosparam command="load" file="$(find aruco_detect)/config/aruco_detect.yaml"/>
```
- Removes `param="aruco_detect"` attribute
- Parameters load directly in node's private namespace

**Verification**:
```bash
# Check parameter was loaded
rosparam get /aruco_detect_node/marker_size
# Output: 20

# Should NOT be under this:
rosparam get /aruco_detect/marker_size
# Should return nothing
```

---

## Control Issues

### Problem: "Drone not responding to commands"

**Checklist**:
1. **Drone Connection**
   ```bash
   rostopic echo /mavros/state
   # Should show connected: true, armed: false/true
   ```

2. **Autopilot Mode**
   ```bash
   # Must be in OFFBOARD mode
   rostopic echo /mavros/state
   # mode: "OFFBOARD"
   ```

3. **Publisher Active**
   ```bash
   rostopic list | grep setpoint_raw
   # Should show /mavros/setpoint_raw/local
   ```

4. **Command Rate**
   - Must publish >2 Hz
   - Current code publishes at 20 Hz ✓

### Problem: "Drone oscillates or spins in place"

**Likely Causes**:

1. **Camera Not Calibrated**
   - Focal lengths (f_x, f_y) wrong
   - Principal point (u_0, v_0) wrong
   - Run camera calibration with `camera_calibration` package

2. **Marker Size Wrong**
   ```yaml
   # If set too small:
   # - Computed depth becomes unrealistic
   # - Control gains become extreme
   
   # If set too large:
   # - Estimated position inaccurate
   # - Control over-compensates
   ```

3. **Adaptive Gain Issues**
   ```cpp
   float min_lambda = 1.25;  // Conservative when converged
   float max_lambda = 0.5;   // Aggressive when far
   
   // Try adjusting these values:
   min_lambda = 0.5;         // More aggressive
   max_lambda = 0.2;         // Less aggressive when far
   ```

4. **Coordinate Frame Mismatch**
   ```cpp
   // Check velocity mapping is correct
   vel.velocity.x = -v_c(1);  // Should negate properly
   vel.velocity.y = -v_c(0);
   vel.velocity.z = -v_c(2);
   vel.yaw_rate = -v_c(3);
   ```

**Debugging**:
```bash
# Monitor control signals
rostopic echo /sub_ibvs/analysis

# Check error convergence
watch -n 0.1 "rostopic echo /sub_ibvs/analysis | grep norm_err"

# Plot errors over time
rqt_plot /sub_ibvs/analysis/norm_err
```

### Problem: "Control convergence too slow"

**Solution**: Increase gains
```cpp
float min_lambda = 0.8;   // Was 1.25 (more conservative)
float max_lambda = 0.3;   // Was 0.5 (more aggressive)

// Or increase base gain in control law:
this->v = -2.0 * lambda * this->J_pinv * this->err;  // Multiply by 2.0
```

### Problem: "Marker tracking loses lock"

**Causes**:
1. **Marker out of view** - Increase window size (w, h)
2. **Wrong marker ID** - Check fiducial_id in condition
3. **Error threshold too tight** - Increase threshold value
4. **Camera latency** - Reduce loop frequency temporarily

**Debug**:
```cpp
ROS_INFO("Tracking marker %d, error: %f, threshold: %f", 
         fid[i].fiducial_id, er, threshold);
```

---

## Hardware/System Issues

### Problem: "No camera feed"

**Check Camera**:
```bash
# List cameras
ls /dev/video*

# Test with v4l2
v4l2-ctl --device=/dev/video0 --list-formats-ext

# View raw stream
rosrun image_view image_view image:=/camera/rgb/image_raw
```

### Problem: "High latency / jerky movement"

**Measure Loop Time**:
```cpp
auto t_start = ros::Time::now();
// ... control computation ...
auto elapsed = ros::Time::now() - t_start;
ROS_INFO("Loop time: %f ms", elapsed.toSec() * 1000);
```

**Optimization**:
- Reduce marker detection resolution
- Decrease queue sizes in filters
- Increase update frequency of non-critical topics

### Problem: "Out of memory"

**Check Memory Usage**:
```bash
ps aux | grep aruco_detect
ps aux | grep sub_ibvs

# Monitor memory over time
top -p <pid>
```

**Memory Leaks**:
- Check for unreleased cv::Mat objects
- Verify message queue sizes not excessive
- Monitor ROS message buffer usage

---

## Software/Build Issues

### Problem: "Cannot find package"

**Solution**:
```bash
# Rebuild workspace
cd ~/dev_ws
catkin clean
catkin build

# Source setup
source devel/setup.bash

# Verify package found
rospack find aruco_detect
rospack find Drone_IBVS
```

### Problem: "Compilation errors"

**Common Issues**:
1. **Missing dependency**
   ```bash
   # Install missing packages
   sudo apt install ros-noetic-opencv # or melodic
   sudo apt install libarmadillo-dev
   ```

2. **OpenCV version mismatch**
   - Code uses OpenCV 4.2
   - Verify installation: `pkg-config --modversion opencv4`

3. **Boost linking**
   ```bash
   # Verify boost installed
   dpkg -l | grep boost
   ```

### Problem: "Node crashes on startup"

**Enable Debug**:
```bash
# Run with debug symbols
gdb rosrun aruco_detect aruco_detect_node

# Or run with verbose logging
export ROSCPP_LOG_LEVEL=DEBUG
rosrun aruco_detect aruco_detect_node
```

---

## Testing & Validation

### Validation Checklist

```bash
# 1. Test marker detection alone
roslaunch aruco_detect aruco_detect.launch

# 2. Verify message publication
rostopic list
rostopic hz /fiducial_vertices
rostopic hz /fiducial_transforms

# 3. Check message contents
rostopic echo /fiducial_vertices (first 5)
rostopic echo /fiducial_transforms (first 5)

# 4. Test camera calibration
rostopic echo /camera/rgb/camera_info | head -30

# 5. Test IBVS control separately
# (simulated drone)
roslaunch Drone_IBVS sub_ibvs.launch

# 6. Monitor control performance
rqt_plot /sub_ibvs/analysis/norm_err

# 7. Full system test with real drone
# (ensure in safe area)
roslaunch aruco_detect complete.launch
roslaunch Drone_IBVS sub_ibvs.launch
```

### Performance Metrics

**Good Performance**:
- Marker detection: >20 Hz
- Control loop: 20 Hz
- Error convergence: <1 second to threshold
- Marker tracking: Continuous (no dropouts)

**Acceptable Performance**:
- Marker detection: 15-20 Hz
- Control loop: 15-20 Hz  
- Error convergence: 1-2 seconds
- Occasional tracking loss (<10%)

**Poor Performance**:
- Marker detection: <15 Hz
- Control loop: <15 Hz
- Error convergence: >2 seconds
- Frequent tracking loss (>10%)

---

## Diagnostic Scripts

### Monitor All Topics
```bash
#!/bin/bash
echo "=== ArUco Detection ==="
rostopic hz /fiducial_vertices
rostopic hz /fiducial_transforms

echo "=== IBVS Control ==="
rostopic hz /mavros/setpoint_raw/local
rostopic hz /sub_ibvs/analysis

echo "=== Drone State ==="
rostopic echo /mavros/state
rostopic echo /mavros/local_position/pose
```

### Plot Control Performance
```bash
#!/bin/bash
rqt_plot \
  /sub_ibvs/analysis/norm_err \
  /sub_ibvs/analysis/err_signal/v1 \
  /sub_ibvs/analysis/err_signal/v2 \
  /sub_ibvs/analysis/ctrl_signal/v1 \
  /sub_ibvs/analysis/ctrl_signal/v2
```

### Check Parameter Loading
```bash
#!/bin/bash
echo "Checking parameter namespace:"
rosparam list | grep marker
rosparam get /aruco_detect_node/marker_size
rosparam get /aruco_detect_node/aruco_dictionary
```

---

## Getting Help

### Enable Verbose Logging
```bash
export ROSCPP_LOG_LEVEL=DEBUG
export ROSCPP_LOG_DEST=file:///tmp/ros.log
roslaunch aruco_detect aruco_detect.launch
# Check /tmp/ros.log
```

### Review Launch File
```bash
cat src/aruco_detect/launch/aruco_detect.launch
cat src/aruco_detect/launch/complete.launch
```

### Check ROS Network
```bash
rosnode list          # All active nodes
rostopic list         # All active topics
rosservice list       # Available services
rosmsg list           # Available message types
```
