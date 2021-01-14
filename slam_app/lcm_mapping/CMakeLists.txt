cmake_minimum_required(VERSION 3.10)
project(main)

if(NOT BUILD_QNX)
#set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11 -fPIC")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


find_package(OpenCV 3.4.6 EXACT REQUIRED)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
include_directories(${EIGEN_INCLUDE_DIR}  ${OpenCV_INCLUDE_DIRS})

include_directories(../../../modules/message/lcm/localization)
include_directories(../../../)

if (BUILD_QNX)
	link_directories(../../../bin_QNX)
else()
	link_directories(../../../bin_x86)
endif()

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

add_executable(lcm_mapping  src/lcm_mapping.cpp)
target_link_libraries(lcm_mapping  lcm  slam_lib Common) 
