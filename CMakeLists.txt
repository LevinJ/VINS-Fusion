cmake_minimum_required(VERSION 2.8.3)
project(vins)



add_subdirectory(camera_models/src)
add_subdirectory(vins_estimator/src)
add_subdirectory(loop_fusion/src)

add_compile_options(-D WITH_ROS_SIMULATE)

#include_directories(../modules/message/lcm ./ros_simulate ./camera_models/include)
#include_directories(${LCM_INCLUDE_DIR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
#include_directories(${EIGEN_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_executable(sim_estimator_lcm  vins_estimator/src/sim_estimator_lcm.cpp   
			vins_estimator/src/vslam_interface.cpp  vins_estimator/src/interface/handler.cpp)
target_link_libraries(sim_estimator_lcm 
    vins_lib 
    loop_fusion_lib
    camera_models
    Common
    ${CERES_LIB}
    ${OpenCV_LIBS}
    #protobuf-lite
   # protobuf 
    lcm 

)

target_include_directories(sim_estimator_lcm PUBLIC  ${EIGEN_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}  )
target_include_directories(sim_estimator_lcm  PUBLIC ../modules/message/lcm  ./ros_simulate ./camera_models/include )
target_include_directories(sim_estimator_lcm PUBLIC ${LCM_INCLUDE_DIR})
			
			
			

if(NOT BUILD_QNX)
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
#include_directories(../../camera_models/include ../../ros_simulate  ${CERES_INCLUDE_DIRS})


add_executable(rosbagtest_lib vins_estimator/src/rosbagtest_lib.cpp vins_estimator/src/vslam_interface.cpp)
target_link_libraries(rosbagtest_lib vins_lib loop_fusion_lib camera_models ${catkin_LIBRARIES}) 

#add python wrapper libary
set(PYBIND11_PYTHON_VERSION  2.7)
add_subdirectory(pybind11)
pybind11_add_module(vslam  python_wrapper/main.cpp  vins_estimator/src/vslam_interface.cpp)
target_link_libraries(vslam  PRIVATE vins_lib loop_fusion_lib camera_models  ${OpenCV_LIBS})
endif()