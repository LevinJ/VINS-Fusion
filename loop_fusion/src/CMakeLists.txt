cmake_minimum_required(VERSION 2.8.3)
project(loop_fusion)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(OpenCV 3.4.6 EXACT REQUIRED)
find_package(Ceres REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)

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

