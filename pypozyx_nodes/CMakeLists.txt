cmake_minimum_required(VERSION 2.8.3)
project(pypozyx_nodes)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14") # if want to use auto in Lamda function, should be more recent than c++14
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    message_generation
    message_runtime
)

add_message_files(
  FILES
    uwb.msg
    uwb_array.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

