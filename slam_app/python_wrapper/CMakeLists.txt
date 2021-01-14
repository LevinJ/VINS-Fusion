cmake_minimum_required(VERSION 3.10)
project(python_wrapper)


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

include_directories( ${EIGEN3_INCLUDE_DIR})



set(PYBIND11_PYTHON_VERSION  2.7)
add_subdirectory(pybind11)
pybind11_add_module(vslam  ./src/main.cpp)
target_link_libraries(vslam  PRIVATE slam_lib)
