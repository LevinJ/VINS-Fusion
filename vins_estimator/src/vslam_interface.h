#pragma once

#include <string>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <functional>
#include <map>
class OdomExtrinsicInfo;
class KeyframeInfo;
class ImageInfo;
class LPInfo;
class VOInfo;

void init_estimator(std::string config_file);
void init_loop_fusion(std::string loop_fution_path);
void inputIMU(double t, const Eigen::Vector3d &linearAcceleration, const Eigen::Vector3d &angularVelocity);
void inputImage(double t, const cv::Mat &  img1, const cv::Mat &img2 = cv::Mat());
void create_map();
void register_vo_callbacks(std::function<void(OdomExtrinsicInfo &)> odom_extric_f,
		std::function<void(KeyframeInfo &)> key_frame_info_f_, std::function<void(ImageInfo &)> img_info_f);

void vo_callback(std::function<void(VOInfo)> cb);
//interfaces for localization

void init_reloc(std::string config_file, std::string loop_fution_path);
void reloc_image(double _time_stamp, cv::Mat &_image);
void reloc_callback(std::function<void(LPInfo)> cb);

void set_multiple_thread(int flag);

void inputFeature(double t, const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> &featureFrame);

