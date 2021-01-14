cmake_minimum_required(VERSION 3.10)
project(ros_mapping)

add_compile_options(-D WITH_ROS_SIMULATE)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fPIC")





find_package(Eigen3 3.3.3 EXACT REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    image_transport
    rosbag)
include_directories(${catkin_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR} ./camera_models/include)


add_executable(ros_mapping ./src/ros_mapping.cpp)
target_link_libraries(ros_mapping slam_lib ${catkin_LIBRARIES}) 