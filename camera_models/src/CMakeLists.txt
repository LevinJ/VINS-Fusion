cmake_minimum_required(VERSION 2.8.3)
project(camera_models)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fPIC")

#find_package(catkin REQUIRED COMPONENTS
#    roscpp
#    std_msgs
#    )

find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
include_directories(${Boost_INCLUDE_DIRS})

find_package(OpenCV 3.4.6 EXACT REQUIRED)

# set(EIGEN_INCLUDE_DIR "/usr/local/include/eigen3")
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})


#catkin_package(
#    INCLUDE_DIRS include
#    LIBRARIES camera_models
#    CATKIN_DEPENDS roscpp std_msgs
##    DEPENDS system_lib
#    )

#include_directories(
#    ${catkin_INCLUDE_DIRS}
#    )

include_directories("../include")

add_executable(Calibrations 
    intrinsic_calib.cc
    chessboard/Chessboard.cc
    calib/CameraCalibration.cc
    camera_models/Camera.cc
    camera_models/CameraFactory.cc
    camera_models/CostFunctionFactory.cc
    camera_models/PinholeCamera.cc
    camera_models/PinholeFullCamera.cc
    camera_models/CataCamera.cc
    camera_models/EquidistantCamera.cc
    camera_models/ScaramuzzaCamera.cc
    sparse_graph/Transform.cc
    gpl/gpl.cc
    gpl/EigenQuaternionParameterization.cc)

add_library(camera_models STATIC
    chessboard/Chessboard.cc
    calib/CameraCalibration.cc
    camera_models/Camera.cc
    camera_models/CameraFactory.cc
    camera_models/CostFunctionFactory.cc
    camera_models/PinholeCamera.cc
    camera_models/PinholeFullCamera.cc
    camera_models/CataCamera.cc
    camera_models/EquidistantCamera.cc
    camera_models/ScaramuzzaCamera.cc
    sparse_graph/Transform.cc
    gpl/gpl.cc
    gpl/EigenQuaternionParameterization.cc)

target_link_libraries(Calibrations ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
target_link_libraries(camera_models ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
