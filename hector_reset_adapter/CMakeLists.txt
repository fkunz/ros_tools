cmake_minimum_required(VERSION 2.8.3)
project(hector_reset_adapter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hector_reset_adapter
#  CATKIN_DEPENDS std_msgs std_srvs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(hector_reset_adapter src/reset_adapter.cpp)

add_dependencies(hector_reset_adapter ${PROJECT_NAME}_gencfg)

target_link_libraries(hector_reset_adapter ${catkin_LIBRARIES})
