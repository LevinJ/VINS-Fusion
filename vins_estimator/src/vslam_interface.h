#pragma once

#include <string>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

void init_estimator(std::string config_file);
void init_loop_fusion(std::string loop_fution_path);
void inputIMU(double t, const Eigen::Vector3d &linearAcceleration, const Eigen::Vector3d &angularVelocity);
void inputImage(double t, cv::Mat &  img1);
void create_map();
