cmake_minimum_required(VERSION 2.8...3.16)
project(sensys)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  serial
  std_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(sensys_node src/driver.cpp)

target_link_libraries(sensys_node ${catkin_LIBRARIES})


