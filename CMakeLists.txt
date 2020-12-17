cmake_minimum_required(VERSION 2.8.3)
project(vins)

SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(PCL 1.3 REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_subdirectory(camera_models/src)
add_subdirectory(vins_estimator/src)
add_subdirectory(loop_fusion/src)

add_compile_options(-D WITH_ROS_SIMULATE)

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
#include_directories(../../camera_models/include ../../ros_simulate  ${CERES_INCLUDE_DIRS})


add_executable(rosbagtest_lib vins_estimator/src/rosbagtest_lib.cpp vins_estimator/src/vslam_interface.cpp)
target_link_libraries(rosbagtest_lib vins_lib loop_fusion_lib camera_models ${catkin_LIBRARIES} ${PCL_LIBRARIES}) 

#add python wrapper libary
set(PYBIND11_PYTHON_VERSION  2.7)
add_subdirectory(pybind11)
pybind11_add_module(vslam  python_wrapper/main.cpp  vins_estimator/src/vslam_interface.cpp)
target_link_libraries(vslam  PRIVATE vins_lib loop_fusion_lib camera_models  ${PCL_LIBRARIES} ${OpenCV_LIBS})
