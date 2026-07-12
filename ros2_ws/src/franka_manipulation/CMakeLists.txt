cmake_minimum_required(VERSION 3.8)
project(franka_manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(franka_interfaces REQUIRED)

# ------------------------------------------------------------------
# Executable
# ------------------------------------------------------------------

add_executable(
  executor_node

  src/main.cpp
  src/executor.cpp
  src/arm.cpp
  src/gripper.cpp
  src/transforms.cpp
)

target_include_directories(
  executor_node
  PUBLIC
    include
)

ament_target_dependencies(
  executor_node

  rclcpp
  rclcpp_action

  geometry_msgs
  control_msgs

  tf2
  tf2_ros
  tf2_geometry_msgs

  moveit_ros_planning_interface
  moveit_msgs
  franka_interfaces
)

#------------------------------------------------------------------
add_executable(moveit_smoke_test
    src/moveit_smoke_test.cpp
)

ament_target_dependencies(moveit_smoke_test
    rclcpp
    geometry_msgs
    moveit_ros_planning_interface
)

target_include_directories(moveit_smoke_test
    PUBLIC
    include
)

install(TARGETS
    moveit_smoke_test
    DESTINATION lib/${PROJECT_NAME}
)

#------------------------------------------------------------------

# ------------------------------------------------------------------
# Install
# ------------------------------------------------------------------

install(
  TARGETS executor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()