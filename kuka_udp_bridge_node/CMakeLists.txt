cmake_minimum_required(VERSION 3.8)
project(kuka_udp_bridge_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

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

add_executable(udp_bridge_node src/udp_bridge_node.cpp)
ament_target_dependencies(udp_bridge_node rclcpp geometry_msgs nav_msgs tf2 tf2_ros sensor_msgs std_msgs)

add_executable(kmp_rectangle_node src/kmp_rectangle_node.cpp)
ament_target_dependencies(kmp_rectangle_node
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  nav_msgs
  std_msgs
)

add_executable(kmp_rectangle_sequential src/kmp_rectangle_sequential.cpp)
ament_target_dependencies(kmp_rectangle_sequential
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
)

add_executable(Arm_homing src/Arm_homing.cpp)
ament_target_dependencies(Arm_homing
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
)

include_directories(include)

add_executable(badminton_demo_node
  src/kuka_robot_interface.cpp
  src/badminton_demo_node.cpp
)

ament_target_dependencies(badminton_demo_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
)

add_executable(Set_speed src/Set_speed.cpp)
ament_target_dependencies(Set_speed
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
)

install(TARGETS
  udp_bridge_node
  kmp_rectangle_node
  kmp_rectangle_sequential
  badminton_demo_node
  Arm_homing
  Set_speed 
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
