# ArUco Detect - Visual Reference Guide

## System Architecture Diagram

```
┌────────────────────────────────────────────────────────────────────┐
│                         CAMERA SYSTEM                              │
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐ │
│  │   RGB Camera     │  │  Depth Camera    │  │  Calibration     │ │
│  │                  │  │                  │  │  Parameters      │ │
│  └────────┬─────────┘  └────────┬─────────┘  └────────┬─────────┘ │
└───────────┼──────────────────────┼──────────────────────┼──────────┘
            │                      │                      │
            │ /camera/rgb/         │ /camera/depth/       │ /camera/rgb/
            │ image_raw            │ image_raw            │ camera_info
            │ (30Hz)               │ (30Hz)               │ (30Hz)
            │                      │                      │
            └──────────────────────┼──────────────────────┘
                                   │
                    ┌──────────────▼──────────────┐
                    │  Message Synchronizer       │
                    │  (message_filters)          │
                    │  - Aligns timestamps        │
                    │  - Buffers messages         │
                    │  - Calls callback when ready│
                    └──────────────┬──────────────┘
                                   │
                    ┌──────────────▼──────────────┐
                    │   ArUco Detector Node       │
                    │                             │
                    │  ┌──────────────────────┐   │
                    │  │ Image Processing:    │   │
                    │  │ - Convert to Gray    │   │
                    │  │ - Detect Markers     │   │
                    │  │ - Extract Corners    │   │
                    │  └──────────────────────┘   │
                    │                             │
                    │  ┌──────────────────────┐   │
                    │  │ Pose Estimation:     │   │
                    │  │ - Load Camera Matrix │   │
                    │  │ - Run solvePnP       │   │
                    │  │ - Get Rvec/Tvec      │   │
                    │  └──────────────────────┘   │
                    │                             │
                    │  ┌──────────────────────┐   │
                    │  │ Output Generation:   │   │
                    │  │ - Create FiducialMsg │   │
                    │  │ - Create TransformMsg│   │
                    │  │ - Broadcast TF       │   │
                    │  │ - Draw Visualization │   │
                    │  └──────────────────────┘   │
                    └──────────────┬──────────────┘
                                   │
                ┌──────────────────┼──────────────────┐
                │                  │                  │
        ┌───────▼────────┐  ┌──────▼──────────┐  ┌───▼────────────┐
        │ fiducial_      │  │ fiducial_       │  │ /fiducial_     │
        │ vertices       │  │ transforms      │  │ images         │
        │ (FiducialArray)│  │ (FiducialTrans) │  │ (Image)        │
        │                │  │                 │  │                │
        │ 2D Corners     │  │ 3D Poses        │  │ Visualization  │
        │ in Pixels      │  │ in Camera Frame │  │ with Markers   │
        └────────┬───────┘  └────────┬────────┘  └───────────────┘
                 │                   │
                 └───────────┬───────┘
                             │
                    ┌────────▼────────┐
                    │  sub_ibvs.cpp   │
                    │                 │
                    │ ┌─────────────┐ │
                    │ │ Callbacks:  │ │
                    │ │ - Marker CB │ │
                    │ │ - Transform │ │
                    │ │ - Camera CB │ │
                    │ └─────────────┘ │
                    │                 │
                    │ ┌─────────────┐ │
                    │ │ IBVS Ctrl:  │ │
                    │ │ - Compute   │ │
                    │ │   Features  │ │
                    │ │ - Compute   │ │
                    │ │   Error     │ │
                    │ │ - Compute   │ │
                    │ │   Velocity  │ │
                    │ └─────────────┘ │
                    │                 │
                    └────────┬────────┘
                             │
                    Velocity Commands
                             │
                    ┌────────▼────────┐
                    │  PX4 Autopilot  │
                    │  (mavros)       │
                    └────────┬────────┘
                             │
                    ┌────────▼────────┐
                    │  Drone Motors   │
                    │  (Flight)       │
                    └─────────────────┘
```

## Message Flow Diagram

### For Each Image Frame:

```
FRAME ARRIVAL
    │
    ├─→ [RGB Image] + [Depth Image] + [Camera Info]
    │        + Timestamp alignment check
    │
    ├─→ SYNCHRONIZATION SUCCESSFUL
    │        │
    │        ├─→ Convert RGB to Grayscale
    │        │
    │        ├─→ ArUco Detection
    │        │        │
    │        │        ├─→ If markers found:
    │        │        │        │
    │        │        │        ├─→ FiducialArray message
    │        │        │        │    (2D corners: x0,y0,x1,y1,x2,y2,x3,y3)
    │        │        │        │
    │        │        │        ├─→ 3D Pose Estimation (solvePnP)
    │        │        │        │    - Input: 2D corners, camera matrix K, dist D
    │        │        │        │    - Output: Rotation vector, Translation vector
    │        │        │        │
    │        │        │        ├─→ Rodrigues(Rvec) → Rotation Matrix
    │        │        │        │
    │        │        │        ├─→ Matrix → Quaternion conversion
    │        │        │        │
    │        │        │        ├─→ FiducialTransformArray message
    │        │        │        │    (Transform: xyz translation + xyzw quaternion)
    │        │        │        │
    │        │        │        ├─→ TF Transform broadcast
    │        │        │        │    (camera_rgb_optical_frame → aruco_marker_N)
    │        │        │        │
    │        │        │        ├─→ Draw markers on image
    │        │        │        │    (cv::circle for corners, cv::drawAxis)
    │        │        │        │
    │        │        │        └─→ Annotated image message
    │        │        │
    │        │        └─→ If no markers: send empty messages
    │        │
    │        └─→ Publish all messages
    │
    └─→ LOOP FOR NEXT FRAME
```

## Data Structure Diagrams

### FiducialArray (fiducial_vertices)
```
┌─ FiducialArray ──────────────────────┐
│                                       │
│ header:                               │
│   ├─ seq: uint32                      │
│   ├─ stamp: time (ROS timestamp)      │
│   └─ frame_id: "camera_rgb_..."       │
│                                       │
│ fiducials[]: Fiducial                 │
│   ├─ [0] Fiducial {                   │
│   │     fiducial_id: 0                │
│   │     x0: 100.5, y0: 150.2          │
│   │     x1: 200.3, y1: 150.1          │
│   │     x2: 200.4, y2: 250.5          │
│   │     x3: 100.6, y3: 250.6          │
│   │   }                               │
│   ├─ [1] Fiducial {                   │
│   │     fiducial_id: 7                │
│   │     x0: 350.1, y0: 200.0          │
│   │     ... (other corners)           │
│   │   }                               │
│   └─ ...                              │
│                                       │
└───────────────────────────────────────┘
```

### FiducialTransformArray (fiducial_transforms)
```
┌─ FiducialTransformArray ──────────────┐
│                                       │
│ header:                               │
│   ├─ seq: uint32                      │
│   ├─ stamp: time                      │
│   └─ frame_id: "camera_rgb_..."       │
│                                       │
│ transforms[]: FiducialTransform       │
│   ├─ [0] FiducialTransform {          │
│   │     fiducial_id: 0                │
│   │     transform: {                  │
│   │       translation: {              │
│   │         x: 0.150 (meters)         │
│   │         y: -0.020                 │
│   │         z: 0.500                  │
│   │       }                           │
│   │       rotation: {                 │
│   │         x: 0.707 (quaternion)     │
│   │         y: 0.000                  │
│   │         z: 0.000                  │
│   │         w: 0.707                  │
│   │       }                           │
│   │     }                             │
│   │   }                               │
│   ├─ [1] FiducialTransform {          │
│   │     fiducial_id: 7                │
│   │     ...                           │
│   │   }                               │
│   └─ ...                              │
│                                       │
└───────────────────────────────────────┘
```

## Coordinate Frame Relationships

```
Camera Optical Frame
        │
        │ (looking down Z-axis, away from camera)
        │
        ├─→ X-axis (right in image)
        │
        ├→→ Y-axis (down in image)
        │
        └→→→ Z-axis (forward, out of camera)


    Y
    ↓
    ├─→ X (right)
    │
    └→→ Z (forward, away from camera)


Marker Frame (on marker)
    
    Y (up edge)
    ↑
    ├─→ X (right edge)
    │
    └→→ Z (out of marker plane)


Transform Path:
camera_rgb_optical_frame
            │
            │ [Rvec, Tvec from PnP]
            │ (3D position + rotation)
            ▼
    aruco_marker_0
    aruco_marker_1
    aruco_marker_N
```

## Processing Pipeline (Detailed)

```
                          INPUT IMAGE
                              │
                    ┌─────────▼─────────┐
                    │  Format Check     │
                    │  - Verify size    │
                    │  - Check encoding │
                    └─────────┬─────────┘
                              │
                    ┌─────────▼─────────┐
                    │  Color Conversion │
                    │  BGR → Grayscale  │
                    │  (cv::cvtColor)   │
                    └─────────┬─────────┘
                              │
                    ┌─────────▼─────────────────┐
                    │  ArUco Detection          │
                    │  cv::aruco::ArucoDetector │
                    │                           │
                    │  1. Find candidates       │
                    │  2. Validate corners      │
                    │  3. Decode marker bits    │
                    │  4. Match with dictionary │
                    │  5. Sub-pixel refining    │
                    └─────────┬─────────────────┘
                              │
                    ┌─────────▼──────────────┐
                    │  Check if calibrated   │
                    │  (camera matrix ready?)│
                    │       /      \         │
                    │      /        \        │
              NO ──/        YES      \       
                   │                  │
          Publish    │        ┌────────▼─────────┐
          FiducialArray        │ 3D Pose Est.    │
                   │        │ cv::solvePnP     │
                   │        │                   │
                   │        │ Inputs:           │
                   │        │ - 2D pts (pixels) │
                   │        │ - 3D pts (marker) │
                   │        │ - K matrix        │
                   │        │ - Dist coeffs     │
                   │        │                   │
                   │        │ Outputs:          │
                   │        │ - Rvec (3x1)     │
                   │        │ - Tvec (3x1)     │
                   │        └────────┬──────────┘
                   │                │
                   │        ┌────────▼──────────┐
                   │        │ Rodrigues Conv.   │
                   │        │ Rvec → RotMatrix  │
                   │        │ (3x3 matrix)      │
                   │        └────────┬──────────┘
                   │                │
                   │        ┌────────▼──────────┐
                   │        │ Quaternion Conv.  │
                   │        │ RotMatrix → Quat  │
                   │        │ (x,y,z,w)         │
                   │        └────────┬──────────┘
                   │                │
                   └────────┬────────┘
                            │
                    ┌───────▼─────────┐
                    │ Create Messages │
                    │                 │
                    ├─ FiducialArray  │
                    ├─ FiducialTrans  │
                    └────────┬────────┘
                             │
                    ┌────────▼────────┐
                    │ Visualization   │
                    │ (Draw on Image) │
                    │                 │
                    │ - cv::circle()  │
                    │   (corners)     │
                    │ - drawMarkers() │
                    │ - drawAxis()    │
                    └────────┬────────┘
                             │
                    ┌────────▼────────┐
                    │ Publish Topics  │
                    │                 │
                    ├─ vertices_pub   │
                    ├─ transforms_pub │
                    ├─ image_pub      │
                    ├─ camera_info_pub│
                    └────────┬────────┘
                             │
                    ┌────────▼────────┐
                    │ TF Broadcasting │
                    │ tf_broadcaster_ │
                    │ .sendTransform()│
                    └────────┬────────┘
                             │
                    ┌────────▼────────┐
                    │ Loop for next   │
                    │ frame           │
                    └─────────────────┘
```

## Topic Timeline (Single Frame Processing)

```
Time
 │
 ├─ t₀: RGB frame arrives
 ├─ t₀: Depth frame arrives  
 ├─ t₀: CameraInfo arrives
 │       │
 │       └─→ Synchronizer waits for all three
 │
 ├─ t₀+ε: Callback triggered (all messages synchronized)
 │       │
 │       ├─→ Convert to grayscale (1-2 ms)
 │       │
 │       ├─→ Detect markers (5-10 ms)
 │       │
 │       ├─→ Estimate pose (5-10 ms)
 │       │
 │       ├─→ Create messages (1 ms)
 │       │
 │       └─→ Publish (1 ms)
 │
 └─ t₀+20-30ms: Topics available for sub_ibvs
       │
       └─→ sub_ibvs receives messages
           (subject to its own subscription queue)
```

## File Organization

```
aruco_detect/
├── Documentation
│   ├── README.md           ← Start here for overview
│   ├── SUMMARY.md          ← Quick reference
│   ├── TECHNICAL.md        ← Architecture & details
│   ├── USAGE.md            ← Examples & debugging
│   ├── BUILD.md            ← Build & dependencies
│   └── [This file]         ← Visual diagrams
│
├── Configuration
│   ├── CMakeLists.txt      ← Build configuration
│   ├── package.xml         ← Package metadata
│   └── config/
│       └── aruco_detect.yaml
│
├── Source Code
│   └── src/
│       └── aruco_detect_node.cpp
│           ├── ArucoDetector class
│           ├── Synchronization setup
│           ├── Detection callback
│           ├── Pose estimation
│           └── Message publishing
│
└── Launch Files
    └── launch/
        ├── aruco_detect.launch
        └── complete.launch
```

## Dependencies Graph

```
aruco_detect_node
    │
    ├─ Core ROS
    │   ├─ roscpp
    │   ├─ rospy
    │   └─ message_filters
    │
    ├─ Messages
    │   ├─ sensor_msgs (Image, CameraInfo)
    │   ├─ geometry_msgs (TransformStamped)
    │   ├─ std_msgs (Header)
    │   ├─ fiducial_msgs (custom)
    │   └─ ibvs_msgs (custom)
    │
    ├─ Computer Vision
    │   ├─ OpenCV
    │   │   ├─ cv::aruco (detection)
    │   │   ├─ cv::Rodrigues (rotation)
    │   │   └─ cv::cvtColor (conversion)
    │   └─ cv_bridge
    │
    ├─ Image Transport
    │   └─ image_transport
    │
    ├─ Transform & Frames
    │   ├─ tf2
    │   ├─ tf2_ros
    │   └─ tf2_geometry_msgs
    │
    ├─ Linear Algebra
    │   └─ Armadillo
    │
    └─ System Libraries
        ├─ C++ Standard Library
        ├─ OpenGL/GLEW (if visualization)
        └─ System calls
```

---

**Use these diagrams to understand:**
- System architecture and data flow
- Message structures and timing
- Processing pipeline steps
- Integration points with sub_ibvs
- File organization and dependencies
