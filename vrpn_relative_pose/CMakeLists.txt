cmake_minimum_required(VERSION 3.8)
project(vrpn_relative_pose)

# 依赖查找
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 节点构建
add_executable(vrpn_relative_pose_node src/vrpn_relative_pose_node.cpp)
ament_target_dependencies(vrpn_relative_pose_node
  rclcpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# 安装目标
install(TARGETS vrpn_relative_pose_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 导出依赖
ament_export_dependencies(
  rclcpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

ament_package()
