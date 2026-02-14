cmake_minimum_required(VERSION 3.8)
project(turret_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(turret_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Sensor fusion node
add_executable(fusion_node
  src/fusion_node.cpp
  src/kalman_filter.cpp
)
target_include_directories(fusion_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(fusion_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  turret_msgs
)

# Target marker node (visualizes tracked targets in RViz)
add_executable(target_marker_node
  src/target_marker_node.cpp
)
target_include_directories(target_marker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(target_marker_node
  rclcpp
  visualization_msgs
  geometry_msgs
  turret_msgs
)

# Install executables
install(TARGETS
  fusion_node
  target_marker_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
