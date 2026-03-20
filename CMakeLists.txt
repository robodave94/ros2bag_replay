cmake_minimum_required(VERSION 3.20)
project(ros2bag_replay CXX)

# ── Standards ──────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── ROS 2 dependencies ─────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# ── camera_info_fixer_node ─────────────────────────────────────────────────
# Subscribes to a bag's CameraInfo (latched) and a depth Image stream, then
# republishes CameraInfo with corrected stamp and frame_id so that RViz2
# DepthCloud can time-sync the two topics correctly.
add_executable(camera_info_fixer_node
  src/camera_info_fixer_node.cpp
)

ament_target_dependencies(camera_info_fixer_node
  rclcpp
  sensor_msgs
)

# ── Install ────────────────────────────────────────────────────────────────
install(
  TARGETS camera_info_fixer_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
