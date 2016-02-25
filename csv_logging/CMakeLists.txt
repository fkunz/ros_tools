cmake_minimum_required(VERSION 2.8.3)
project(csv_logging)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES periodic_logger
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(periodic_logger src/periodic_logger.cpp)

add_dependencies(periodic_logger ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(periodic_logger ${catkin_LIBRARIES})
