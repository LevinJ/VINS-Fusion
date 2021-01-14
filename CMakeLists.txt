cmake_minimum_required(VERSION 2.8.3)
project(vins)



add_subdirectory(camera_models/src)
add_subdirectory(vins_estimator/src)
add_subdirectory(loop_fusion/src)

add_subdirectory(slam_app)
