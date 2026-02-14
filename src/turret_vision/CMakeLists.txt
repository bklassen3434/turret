cmake_minimum_required(VERSION 3.8)
project(turret_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turret_msgs REQUIRED)

# Vision detector node
add_executable(vision_detector_node
  src/vision_detector_node.cpp
)
target_include_directories(vision_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(vision_detector_node
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(vision_detector_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  image_transport
  turret_msgs
)

# Synthetic camera node (for when Gazebo camera doesn't work)
add_executable(synthetic_camera_node
  src/synthetic_camera_node.cpp
)
target_include_directories(synthetic_camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(synthetic_camera_node
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(synthetic_camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  tf2_ros
)

# Install executables
install(TARGETS
  vision_detector_node
  synthetic_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
