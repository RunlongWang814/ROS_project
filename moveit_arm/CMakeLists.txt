cmake_minimum_required(VERSION 3.0.2)
project(moveit_arm)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
  roscpp
  rospy
  link_service
  geometry_msgs
  std_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system filesystem date_time thread)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime roscpp moveit_core moveit_ros_planning moveit_ros_planning_interface geometry_msgs std_msgs link_service
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare C++ executable targets
add_executable(moveit_arm src/moveit_arm.cpp)
add_executable(moveit_service src/moveit_service.cpp)

## Ensure that your custom messages/services are generated before these executables are built
add_dependencies(moveit_arm ${catkin_EXPORTED_TARGETS})
add_dependencies(moveit_service ${catkin_EXPORTED_TARGETS} ${link_service_EXPORTED_TARGETS})

## Specify libraries to link the executables to
target_link_libraries(moveit_arm
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(moveit_service
  ${catkin_LIBRARIES}
)

## Install targets
install(TARGETS moveit_arm moveit_service
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
