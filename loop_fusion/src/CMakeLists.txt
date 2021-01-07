cmake_minimum_required(VERSION 2.8.3)
project(loop_fusion)

if(NOT BUILD_QNX)
set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11 -fPIC")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(OpenCV 3.4.6 EXACT REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
include_directories(${EIGEN_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


add_compile_options(-D WITH_ROS_SIMULATE)

include_directories(../../camera_models/include  ../../ros_simulate  ${EIGEN3_INCLUDE_DIR})


add_library(loop_fusion_lib
#    pose_graph_node.cpp
    pose_graph.cpp
    keyframe.cpp
#    utility/CameraPoseVisualization.cpp
    utility/LoopInfoLogging.cpp
    ThirdParty/DBoW/BowVector.cpp
    ThirdParty/DBoW/FBrief.cpp
    ThirdParty/DBoW/FeatureVector.cpp
    ThirdParty/DBoW/QueryResults.cpp
    ThirdParty/DBoW/ScoringObject.cpp
    ThirdParty/DUtils/Random.cpp
    ThirdParty/DUtils/Timestamp.cpp
    ThirdParty/DVision/BRIEF.cpp
    ThirdParty/VocabularyBinary.cpp
    CloudPointMap.cpp
    LoopFusion.cpp
    LPStateSubscriber.cpp
#    Loop_fusion_node.cpp
    )

