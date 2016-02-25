cmake_minimum_required(VERSION 2.8.3)
project(odometry_manipulation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs nav_msgs sensor_msgs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(odometry_manipulation_node src/odometry_manipulation_node.cpp)

add_dependencies(odometry_manipulation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(odometry_manipulation_node ${catkin_LIBRARIES})
