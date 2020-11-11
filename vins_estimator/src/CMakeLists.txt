cmake_minimum_required(VERSION 2.8.3)
project(vins)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


find_package(OpenCV 3.4.6 EXACT REQUIRED)

message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

find_package(Ceres 1.14.0 EXACT REQUIRED)

include_directories(../../camera_models/include ../../ros_simulate  ${CERES_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3 3.3.3 EXACT REQUIRED)
include_directories(
#  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

message("-- Found Eigen3 version ${Eigen3_VERSION}: ${EIGEN3_INCLUDE_DIR}")
message("-- Found Ceres version ${Ceres_VERSION}: ${CERES_INCLUDE_DIR}")
#catkin_package()

add_compile_options(-D WITH_ROS_SIMULATE)

add_library(vins_lib
    estimator/parameters.cpp
    estimator/estimator.cpp
    estimator/feature_manager.cpp
    factor/pose_local_parameterization.cpp
    factor/projectionTwoFrameOneCamFactor.cpp
    factor/projectionTwoFrameTwoCamFactor.cpp
    factor/projectionOneFrameTwoCamFactor.cpp
    factor/marginalization_factor.cpp
    utility/utility.cpp
    ../../ros_simulate/utility/visualization.cpp
#    utility/CameraPoseVisualization.cpp
    initial/solve_5pts.cpp
    initial/initial_aligment.cpp
    initial/initial_sfm.cpp
    initial/initial_ex_rotation.cpp
    featureTracker/feature_tracker.cpp)
target_link_libraries(vins_lib  ${OpenCV_LIBS} ${CERES_LIBRARIES})
