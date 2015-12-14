cmake_minimum_required(VERSION 2.8.3)
project(header_manipulation)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
  tf
  topic_tools
)


generate_dynamic_reconfigure_options(
  cfg/HeaderManipulation.cfg
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(scan_manipulation_node src/PointcloudManipulation.cpp)
add_executable(header_manipulation_node src/HeaderManipulation.cpp)
add_executable(time_manipulation_node src/HeaderManipulation.cpp)

add_dependencies(scan_manipulation_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(header_manipulation_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(time_manipulation_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(scan_manipulation_node ${catkin_LIBRARIES})
target_link_libraries(header_manipulation_node ${catkin_LIBRARIES})
target_link_libraries(time_manipulation_node ${catkin_LIBRARIES})
