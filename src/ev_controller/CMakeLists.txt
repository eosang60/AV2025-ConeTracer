cmake_minimum_required(VERSION 3.0.2)
project(ev_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)   # ← C++11 추가 (강추)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp              # ← 반드시 필요!
  rospy
  std_msgs
  fs_msgs
  lidar_cone_detector
)

catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES ev_controller
  CATKIN_DEPENDS roscpp rospy std_msgs fs_msgs
  #  DEPENDS system_lib
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

add_executable(motor_ctr_node src/motor_ctr_node.cpp)

add_dependencies(motor_ctr_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) # 빌드 의존성 (선택이지만 권장)

target_link_libraries(motor_ctr_node
  ${catkin_LIBRARIES}
)
