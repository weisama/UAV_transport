cmake_minimum_required(VERSION 3.5)
project(offboard_test)

# 默认使用C++17标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 添加可执行文件
add_executable(offboard_test_node 
  src/offboard_test.cpp
)

# 链接依赖
ament_target_dependencies(offboard_test_node
  rclcpp
  geometry_msgs
  mavros_msgs
  tf2
  tf2_geometry_msgs
)

# 添加包含目录
target_include_directories(offboard_test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 安装可执行文件
install(TARGETS offboard_test_node
  DESTINATION lib/${PROJECT_NAME}
)

# 导出依赖
ament_target_dependencies(offboard_test_node
  rclcpp
  geometry_msgs
  mavros_msgs
  tf2
  tf2_geometry_msgs
)

# 导出包配置
ament_package()
