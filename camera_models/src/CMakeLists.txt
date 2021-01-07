cmake_minimum_required(VERSION 2.8.3)
project(camera_models)

if(NOT BUILD_QNX)
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11 -fPIC")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 3.4.6 EXACT REQUIRED)
endif()


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
include_directories(${EIGEN_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

#message(STATUS "CMAKE_CXX_FLAGS = ${CMAKE_CXX_FLAGS}")
#message(STATUS "EIGEN_INCLUDE_DIR = ${EIGEN_INCLUDE_DIR}")
#message(STATUS "CERES_INCLUDE_DIRS = ${CERES_INCLUDE_DIRS}")
#message(STATUS "CERES_LIBRARIES = ${CERES_LIBRARIES}")
#message(STATUS "Boost_INCLUDE_DIRS = ${Boost_INCLUDE_DIRS}")
#message(STATUS "Boost_LIBRARIES = ${Boost_LIBRARIES}")
#message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
#message(STATUS "OpenCV_LIBRARIES = ${OpenCV_LIBRARIES}")



include_directories("../include")

#get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
#foreach(dir ${dirs})
#  message(STATUS "dir='${dir}'")
#endforeach()


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

add_library(camera_models SHARED 
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
