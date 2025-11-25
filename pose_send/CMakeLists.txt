cmake_minimum_required(VERSION 3.8)
project(pose_send)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(pose_send_node src/pose_send_node.cpp)
ament_target_dependencies(pose_send_node
  rclcpp
  geometry_msgs
  nav_msgs
)

install(TARGETS
  pose_send_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
