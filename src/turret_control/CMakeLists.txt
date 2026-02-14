cmake_minimum_required(VERSION 3.8)
project(turret_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(turret_msgs REQUIRED)

# Turret controller node
add_executable(turret_controller_node
  src/turret_controller_node.cpp
  src/pid_controller.cpp
)
target_include_directories(turret_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(turret_controller_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  turret_msgs
)

# Joint simulator node (simulates joints without Gazebo)
add_executable(joint_simulator_node
  src/joint_simulator_node.cpp
)
target_include_directories(joint_simulator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(joint_simulator_node
  rclcpp
  sensor_msgs
  std_msgs
)

# Install executables
install(TARGETS
  turret_controller_node
  joint_simulator_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
