cmake_minimum_required(VERSION 2.8.3)
project(dynamic_ibvs)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  fiducial_msgs
  cv_bridge
  image_transport
  mavros_msgs
)

find_package(OpenCV REQUIRED)
find_package(Armadillo REQUIRED)

set(Armadillo_INCLUDE_DIRS ${ARMADILLO_INCLUDE_DIRS})
set(Armadillo_LIBRARIES ${ARMADILLO_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs fiducial_msgs cv_bridge image_transport mavros_msgs
  DEPENDS OpenCV Armadillo
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${ARMADILLO_INCLUDE_DIRS}
)

# Dynamic IBVS Controller Node
add_executable(platform_ibvs_node 
  src/platform_ibvs_node.cpp
  src/marker_detector.cpp
  src/ibvs_controller.cpp
)

target_link_libraries(platform_ibvs_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${ARMADILLO_LIBRARIES}
)

add_dependencies(platform_ibvs_node ${catkin_EXPORTED_TARGETS})

# Install executables
install(TARGETS platform_ibvs_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
