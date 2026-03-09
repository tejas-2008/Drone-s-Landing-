cmake_minimum_required(VERSION 3.0.2)
project(platform_control)

## Compile as C++14 (safe for Gazebo + Noetic)
add_compile_options(-std=c++14)

# --------------------------------------------------
# Find catkin packages
# --------------------------------------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

# --------------------------------------------------
# Find Gazebo
# --------------------------------------------------
find_package(gazebo REQUIRED)

# --------------------------------------------------
# Declare ROS service files
# --------------------------------------------------
add_service_files(
  FILES
    SetPlatformVelocity.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

# --------------------------------------------------
# catkin package
# --------------------------------------------------
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES moving_platform_plugin
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

# --------------------------------------------------
# Include directories
# --------------------------------------------------
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${gazebo_LIBRARY_DIRS}
)

# --------------------------------------------------
# Build Gazebo plugin (shared library)
# --------------------------------------------------
add_library(moving_platform_plugin SHARED
  src/moving_platform_plugin.cpp
)

add_dependencies(moving_platform_plugin
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(moving_platform_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

# --------------------------------------------------
# Install plugin (important for Gazebo discovery)
# --------------------------------------------------
install(TARGETS moving_platform_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY srv
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
