cmake_minimum_required(VERSION 2.8.3)
project(aruco_detect)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  image_transport
  tf2
  tf2_ros
  tf2_geometry_msgs
  fiducial_msgs
  ibvs_msgs
  message_generation
)

find_package(OpenCV REQUIRED)
find_package(Armadillo REQUIRED)
find_package(yaml-cpp REQUIRED)

set(Armadillo_INCLUDE_DIRS ${ARMADILLO_INCLUDE_DIRS})
set(Armadillo_LIBRARIES ${ARMADILLO_LIBRARIES})


add_service_files(
  FILES
    GenerateFractalMarker.srv
)


generate_messages(
  DEPENDENCIES
    std_msgs
)


catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs cv_bridge image_transport tf2 tf2_ros tf2_geometry_msgs fiducial_msgs ibvs_msgs message_runtime
  DEPENDS OpenCV Armadillo yaml-cpp
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${ARMADILLO_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)


# ArUco detection node
add_executable(aruco_detect_node src/aruco_detect_node.cpp)
target_link_libraries(aruco_detect_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${ARMADILLO_LIBRARIES}
)

add_dependencies(aruco_detect_node ${catkin_EXPORTED_TARGETS})

# ArUco group detection node
add_executable(group_detection_node src/group_detection_node.cpp)
target_link_libraries(group_detection_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)
add_dependencies(group_detection_node ${catkin_EXPORTED_TARGETS})


# add_executable(fractal_marker_server src/generate_fractal.cpp)
# target_link_libraries(fractal_marker_server
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   ${YAML_CPP_LIBRARIES}
# )
# add_dependencies(fractal_marker_server ${catkin_EXPORTED_TARGETS})



# Install executables
install(TARGETS aruco_detect_node group_detection_node #fractal_marker_server
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
