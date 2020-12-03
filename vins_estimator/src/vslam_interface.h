#pragma once

#include <string>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <functional>
class OdomExtrinsicInfo;
class KeyframeInfo;
class ImageInfo;

void init_estimator(std::string config_file);
void init_loop_fusion(std::string loop_fution_path);
void inputIMU(double t, const Eigen::Vector3d &linearAcceleration, const Eigen::Vector3d &angularVelocity);
void inputImage(double t, cv::Mat &  img1);
void create_map();
void register_vo_callbacks(std::function<void(OdomExtrinsicInfo &)> odom_extric_f,
		std::function<void(KeyframeInfo &)> key_frame_info_f_, std::function<void(ImageInfo &)> img_info_f);
