cmake_minimum_required(VERSION 3.0.2)
project(realsense_bridge)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
SET(CMAKE_BUILD_TYPE "Release")
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS} -O3 -Wall")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        pcl_conversions
        pcl_ros
        roscpp
        sensor_msgs
        std_msgs
        )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
# set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4")
find_package(OpenCV)

find_package(realsense2)
catkin_package(
        CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${realsense_INCLUDE_DIR}

)


add_executable(bag_replay bag_replay.cpp)
target_link_libraries(bag_replay
        ${catkin_LIBRARIES} ${OpenCV_LIBS}
        ${realsense2_LIBRARY}
        )

add_executable(live live.cpp)
target_link_libraries(live
        ${catkin_LIBRARIES} ${OpenCV_LIBS}
        ${realsense2_LIBRARY}
        )