cmake_minimum_required(VERSION 3.8)
project(aubo_auto_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

# 添加pose_publisher可执行文件
add_executable(pose_publisher src/pose_publisher.cpp)
ament_target_dependencies(pose_publisher
  rclcpp
  geometry_msgs
)

# 添加auto_move_controller可执行文件  
add_executable(auto_move_controller src/auto_move_controller.cpp)
ament_target_dependencies(auto_move_controller
  rclcpp
  rclcpp_action
  geometry_msgs
  moveit_msgs
  shape_msgs
)

# 安装可执行文件
install(TARGETS
  pose_publisher
  auto_move_controller
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
