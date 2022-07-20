cmake_minimum_required(VERSION 2.8.3)
project(ekf_landing)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14") # if want to use auto in Lamda function, should be more recent than c++14
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    cv_bridge
    pcl_conversions
    pcl_ros
    pypozyx_nodes
    message_generation
    message_runtime
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

add_message_files(
  FILES
    bbox.msg
    bboxes.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package()

add_executable(ekf_landing_node src/main.cpp)
target_link_libraries(ekf_landing_node ${catkin_LIBRARIES} ${EIGEN3_LIBS} ${OpenCV_LIBS})

add_executable(autolanding_node src/autoland.cpp)
target_link_libraries(autolanding_node ${catkin_LIBRARIES} ${EIGEN3_LIBS} ${OpenCV_LIBS})