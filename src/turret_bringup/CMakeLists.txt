cmake_minimum_required(VERSION 3.8)
project(turret_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# Target simulator node
add_executable(target_sim_node
  src/target_sim_node.cpp
)
ament_target_dependencies(target_sim_node
  rclcpp
  tf2_ros
  geometry_msgs
)

# Install executables
install(TARGETS
  target_sim_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
