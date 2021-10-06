cmake_minimum_required(VERSION 2.8...3.16)
project(arduino_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(arduino_driver src/driver.cpp)

target_link_libraries(arduino_driver ${catkin_LIBRARIES})


