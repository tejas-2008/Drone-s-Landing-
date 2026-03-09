# Drone IBVS (Image-Based Visual Servoing)

A ROS-based autonomous drone control system using Image-Based Visual Servoing (IBVS) with ArUco marker detection for precise position control.

## PHASE I: Landing on Static Platform 

### Overview

This system enables a drone to autonomously track and position itself relative to ArUco markers using visual feedback. The system uses a two-stage approach:

1. **Stage 1**: Approach and center on a large marker (ID 0)
2. **Stage 2**: Transition to a smaller marker (ID 7) and hover at a designated altitude


### Key Features

- **ArUco Marker Detection**: Real-time detection of ArUco markers with pose estimation
- **Image-Based Visual Servoing**: Feature-based control using marker corners and geometric moments
- **MAVRos Integration**: Full drone control via PX4 autopilot
- **Visual Feedback**: Real-time visualization with desired and actual marker positions
- **Automatic Mode Switching**: Seamless transition between tracking stages

### System Architecture

```
Camera Input
    ↓
[ArUco Detection Node] → Marker positions & transforms
    ↓
[IBVS Control Node] → Control commands
    ↓
[MAVRos Bridge] → Drone Autopilot
```

### Packages

- **aruco_detect**: ArUco marker detection and pose estimation
- **Drone_IBVS**: IBVS control logic and drone command generation
- **ibvs_msgs**: Custom ROS message definitions

### Dependencies

- ROS (Melodic/Noetic)
- OpenCV 4.2+
- Armadillo (linear algebra library)
- MAVRos
- cv_bridge
- image_transport

### Quick Start

1. **Build the workspace**:
   ```bash
   cd ~/dev_ws
   catkin build
   ```

2. **Source setup**:
   ```bash
   source devel/setup.bash
   ```

3. **Run the complete system**:
   ```bash
   roslaunch aruco_detect complete.launch
   roslaunch Drone_IBVS sub_ibvs.launch
   ```

### Configuration

Main configuration file: `src/aruco_detect/config/aruco_detect.yaml`

```yaml
marker_size: 20          # Marker size in cm
aruco_dictionary: DICT_4X4_50  # Dictionary type
marker_frame_id: aruco_marker
camera_frame_id: camera_link
```

### Topics

#### Published
- `/fiducial_vertices`: Detected marker corner positions
- `/fiducial_transforms`: Marker pose in camera frame
- `/fiducial_images`: Annotated image with detected markers
- `/sub_ibvs/output_video`: IBVS output with desired positions
- `/sub_ibvs/analysis`: Control signal analysis data

#### Subscribed
- `/camera/rgb/image_raw`: Camera RGB stream
- `/camera/depth/image_raw`: Depth stream
- `/camera/rgb/camera_info`: Camera calibration
- `mavros/state`: Drone state
- `mavros/local_position/pose`: Drone position

### Control Flow

1. Camera captures images of markers
2. ArUco detector identifies marker positions and poses
3. IBVS computes control errors based on visual features
4. Control law generates velocity commands
5. Commands sent to drone via MAVRos
6. Drone responds and cycle repeats

### Marker Tracking Stages

**Stage 1 (Marker ID 0)**:
- Larger search window
- Higher altitude (0.6m)
- Threshold: error < 0.06

**Stage 2 (Marker ID 7)**:
- Smaller window (precise positioning)
- Lower altitude (0.15m)
- Threshold: error < 0.015
- Lands when conditions met

### File Structure

```
dev_ws/
├── src/
│   ├── aruco_detect/          # Marker detection
│   │   ├── src/aruco_detect_node.cpp
│   │   ├── config/aruco_detect.yaml
│   │   └── launch/
│   ├── Drone_IBVS/            # IBVS control
│   │   ├── src/
│   │   │   ├── sub_ibvs.cpp   # Main control node
│   │   │   └── pub_ibvs.cpp   # Alternative publisher
│   │   └── launch/
│   └── ibvs_msgs/             # Custom messages
└── docs/                      # Documentation
```

### Troubleshooting

#### Marker not detected
- Check camera calibration
- Verify marker size matches config
- Ensure adequate lighting

#### Drone not arming
- Check MAVRos connection
- Verify autopilot is in correct mode
- Check battery levels

#### Control unstable
- Adjust `lambda` gain parameters in `compute_ctrl_law()`
- Verify camera calibration matrix
- Check marker_size configuration

## PHASE II: Landing on Moving Platform
### Overview

This phase mostly focuses on improving drone's ability to land on moving plaform. 

This will be done is following steps: 

1. **Stage 1 [PERCEPTION]**: Platform detection and Tracking. 
2. **Stage 2 [CRUSING]**: Velocity Esimation and Crusing. This stage will reduce the distance between drone and platform below a certain thrshold.
3. **Stage 3 [PRE-TOCHDOWN]**: In this stage, the drone will try to bring the error as low as possible, while achiving stability.
4. **Stage 4 [TOUCHDOWN]**: Final stage where the drone will try to land on the moving platform, followed by disarming.


### System Architecture

```
Camera Input
    ↓
[PERCEPTION] → Platform positions & transforms
    ↓
[IBVS Control Node] → Control State Machine 
    ↓
[MAVRos Bridge] → Drone Autopilot
```

### Detection/Tracking Pipeline
**Changes** :

1. ***Platform Designing***: The Platform will be a squared shaped white board. It has 5 ArUco markers [One at centre and Four at corners]. Centre marker should be larger enabling distant detection.
2. ***Introducing Tracking***: The strategy should also include tracking after successful identification storing last known pose of platform. This will make pose consistent.

### Control Pipeline
**Changes** :
1. ***State Machine***: The control pipeline will include a state machine switching between different stages.
2. ***STATE I [IDLE]***: This is the state when detection data is not being received.
3. ***STATE II [CRUSING]***: This state will be active when the distance between drone and platform is above a certain threshold. The drone will receive a pose from PERCEPTION. The MPC based controller will be executed to achieve the dynamic pose.
4. ***STATE III [PRE-TOUCHDOWN]***: This state will be active when the distance between drone and platform is below a certain threshold, reducing the altitude to a minimum value. The drone will try to align with the platform centre till stability is achieved.
5. ***STATE IV [TOUCHDOWN]***: This state will be active when the drone is stable above the platform. The drone will try to land on the platform and disarm.
6. ***STATE V [ERROR]***: This state will be triggered whenever we observe unexpected behaviour. It should show a debugging message.


### File Structure

```
dev_ws/
├── src/
│   ├── aruco_detect/          # Marker detection/tracking
│   │   ├── src/aruco_detect_node.cpp
│   │   ├── config/aruco_detect.yaml
│   │   └── launch/
│   ├── dynamic_ibvs/            # MPC inspired IBVS control 
|   |   ├── params
|   |   |   └── dynamic_controller.yaml
|   |   ├── include
|   |   │   └── dynamic_ibvs/
|   |   │       └── controller.hpp   # Header files
│   │   ├── src/
│   │   │   ├── controller.cpp   
│   │   │   └── main.cpp   #  Main control node
│   │   └── launch/
|   |   │   └── controller.launch      # launch file to launch all control nodes
│   └── ibvs_msgs/             # Custom messages
└── docs/                      # Documentation
```


### References

- ArUco: [OpenCV ArUco Module](https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html)
- IBVS: [Visual Servoing](https://en.wikipedia.org/wiki/Visual_servoing)
- MAVROS: [MAVRos Documentation](http://wiki.ros.org/mavros)
