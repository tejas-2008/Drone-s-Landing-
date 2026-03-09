# Build and Dependencies Guide

## System Dependencies

Before building the package, ensure the following dependencies are installed:

### Required System Packages (Ubuntu 20.04/Focal)

```bash
# OpenCV with ArUco support
sudo apt-get install libopencv-dev

# Armadillo (linear algebra library)
sudo apt-get install libarmadillo-dev

# ROS development tools
sudo apt-get install ros-focal-cv-bridge ros-focal-image-transport
sudo apt-get install ros-focal-message-filters
sudo apt-get install ros-focal-tf2-ros ros-focal-tf2-geometry-msgs
```

### Required ROS Packages

The following packages should be in your catkin workspace:

```bash
# Message packages
rosinstall_generator fiducial_msgs --rosdistro focal --deps
rosinstall_generator ibvs_msgs --rosdistro focal --deps

# Or install pre-built versions
sudo apt-get install ros-focal-fiducial-msgs ros-focal-ibvs-msgs
```

## Building the Package

### Option 1: Using catkin build (recommended)

```bash
cd ~/dev_ws
catkin build aruco_detect

# Or with verbose output
catkin build aruco_detect --verbose

# Clean build
catkin clean aruco_detect
catkin build aruco_detect
```

### Option 2: Using catkin_make

```bash
cd ~/dev_ws
catkin_make --only-pkg-with-deps aruco_detect

# Or
catkin_make --pkg aruco_detect
```

## Verifying Installation

After successful build, verify the package:

```bash
# Check package is found
rospack find aruco_detect
# Should output: /home/tejas/dev_ws/src/aruco_detect

# Check package dependencies
rospack depends aruco_detect

# Check if executable was built
find ~/dev_ws/devel -name "aruco_detect_node" -type f

# Verify executable is executable
ls -la ~/dev_ws/devel/lib/aruco_detect/aruco_detect_node
# Should show -rwxr-xr-x (executable)
```

## Common Build Issues

### Issue 1: Package not found
```
CMake Error: Could not find a package configuration file provided by "fiducial_msgs"
```

**Solution:**
```bash
# Make sure message packages are in workspace
cd ~/dev_ws/src
git clone <repository-with-fiducial_msgs>
git clone <repository-with-ibvs_msgs>
catkin build fiducial_msgs ibvs_msgs
source devel/setup.bash
catkin build aruco_detect
```

### Issue 2: OpenCV not found
```
CMake Error: Could not find OpenCV
```

**Solution:**
```bash
# Install OpenCV development files
sudo apt-get install libopencv-dev opencv-data

# If still failing, specify OpenCV path
catkin build aruco_detect -DCMAKE_PREFIX_PATH=/usr/local/opencv
```

### Issue 3: Armadillo not found
```
CMake Error: Could not find Armadillo
```

**Solution:**
```bash
# Install Armadillo
sudo apt-get install libarmadillo-dev

# Verify installation
pkg-config --cflags --libs armadillo
```

### Issue 4: Message filter compilation errors
```
error: 'message_filters/...' file not found
```

**Solution:**
```bash
# Ensure message_filters is installed
sudo apt-get install ros-focal-message-filters

# Update CMake cache
rm -rf ~/dev_ws/build ~/dev_ws/devel
catkin build aruco_detect
```

## Dependency Tree

```
aruco_detect
├── roscpp
├── rospy
├── std_msgs
├── geometry_msgs
├── sensor_msgs
├── cv_bridge
├── image_transport
├── tf2
├── tf2_ros
├── tf2_geometry_msgs
├── fiducial_msgs (custom)
├── ibvs_msgs (custom)
├── OpenCV 3+
│   └── (includes ArUco module)
└── Armadillo

sub_ibvs
├── [same as above]
├── mavros_msgs
└── geometry_msgs
```

## Docker Build

If using Docker with the provided Dockerfile:

```bash
# Build Docker image
docker build -t ibvs-system .

# Run container with ROS environment
docker run -it --rm ibvs-system

# Inside container, build package
catkin build aruco_detect
source devel/setup.bash
roslaunch aruco_detect aruco_detect.launch
```

## Environment Setup

Before running any ROS nodes, set up your environment:

```bash
# Add to ~/.bashrc
source ~/dev_ws/devel/setup.bash
export ROS_PACKAGE_PATH=/home/tejas/dev_ws/src:$ROS_PACKAGE_PATH

# Apply changes
source ~/.bashrc

# Verify setup
echo $ROS_PACKAGE_PATH
```

## Testing the Build

### 1. Run unit tests (if any exist)
```bash
catkin build aruco_detect --catkin-make-args run_tests
```

### 2. Start roscore
```bash
roscore &
```

### 3. Check node can start
```bash
rosrun aruco_detect aruco_detect_node &
# Should show initialization messages
```

### 4. Check published topics (will be empty without camera input)
```bash
rostopic list | grep fiducial
```

### 5. Shutdown
```bash
killall aruco_detect_node
killall rosmaster
```

## Performance Build

For optimized performance:

```bash
# Build with optimizations
catkin build aruco_detect \
  -DCMAKE_BUILD_TYPE=Release \
  --cmake-args -O3 -march=native

# View build options
cmake --help-variable CMAKE_BUILD_TYPE
```

## Debug Build

For debugging:

```bash
# Build with debug symbols
catkin build aruco_detect \
  -DCMAKE_BUILD_TYPE=Debug \
  --verbose

# Use gdb for debugging
gdb ~/dev_ws/devel/lib/aruco_detect/aruco_detect_node
```

## Installation

To install the built package system-wide:

```bash
# After successful build
catkin install --only-pkg-with-deps aruco_detect

# Files will be installed to devel/install directory
# Then you can source:
source ~/dev_ws/install/setup.bash
```

## Version Compatibility

| ROS Distribution | OpenCV | Ubuntu | Status |
|-----------------|--------|--------|--------|
| Focal (20.04) | 4.2+ | 20.04 | ✓ Tested |
| Melodic (18.04) | 3.4+ | 18.04 | ✓ Should work |
| Noetic (20.04) | 4.2+ | 20.04 | ✓ Tested |
| Kinetic (16.04) | 3.2+ | 16.04 | ⚠ Old, may need fixes |

## Cleanup

If you need to clean the build:

```bash
# Clean build artifacts
catkin clean aruco_detect

# Full workspace clean
catkin clean --all

# Remove build/devel directories
rm -rf ~/dev_ws/build ~/dev_ws/devel ~/dev_ws/install

# Fresh build
catkin build
```

## Build Logs

Build output is saved in:
```bash
# View recent build log
cat ~/dev_ws/build/aruco_detect/cmake.log

# View all logs
ls -la ~/.catkin_tools/logs/aruco_detect/
```

## Getting Help

If build fails:

1. Check the CMakeLists.txt for syntax errors
2. Verify all dependencies are installed
3. Check ROS environment is properly sourced
4. Review build log for specific error messages
5. Check package.xml for dependency declarations

```bash
# Diagnose the build
rosdep check --all-packages --rosdistro focal
rosdep resolve fiducial_msgs
rosdep resolve ibvs_msgs
```
