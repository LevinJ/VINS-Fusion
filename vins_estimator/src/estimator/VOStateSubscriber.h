/*
 * VOStateSubscriber.h
 *
 *  Created on: Nov 16, 2020
 *      Author: levin
 */

#ifndef VINS_FUSION_VINS_ESTIMATOR_SRC_ESTIMATOR_VOSTATESUBSCRIBER_H_
#define VINS_FUSION_VINS_ESTIMATOR_SRC_ESTIMATOR_VOSTATESUBSCRIBER_H_

#include <memory>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <functional>
class Estimator;

//class VOStates{
//public:
//	VOStates();
//	virtual ~VOStates();
//
//};

class VoBaseInfo{
public:
	VoBaseInfo(){
		t_ = 0;
	};
	virtual ~VoBaseInfo(){};
	double t_;
};
class KeyframeInfo: public VoBaseInfo{
public:
	KeyframeInfo(){};
	virtual ~KeyframeInfo(){};
	Eigen::Vector3d P_;
	Eigen::Quaterniond R_;
	std::vector<Eigen::Matrix<double, 8, 1>> pnt_3d2ds_;

};

class ImageInfo: public VoBaseInfo{
public:
	ImageInfo(){};
	virtual ~ImageInfo(){};
	cv::Mat img_;

};

class OdomExtrinsicInfo: public VoBaseInfo{
public:
	OdomExtrinsicInfo(){};
	virtual ~OdomExtrinsicInfo(){};
	Eigen::Vector3d P_;
	Eigen::Vector3d V_;
	Eigen::Matrix3d R_;
	Eigen::Matrix3d ric_;
	Eigen::Vector3d tic_;
};

class VOInfo{
public:
	std::shared_ptr<ImageInfo> im_info_ptr_;
	std::shared_ptr<OdomExtrinsicInfo> odom_extric_info_ptr_;
	cv::Mat img_tracking_;
};


class VOStateSubscriber {
public:
	VOStateSubscriber(){};
	virtual void update_keyframe(std::shared_ptr<KeyframeInfo> kf_info_ptr)=0;
	virtual void update_img(std::shared_ptr<ImageInfo> im_info_ptr)=0;
	virtual void update_odom_extrinsic(std::shared_ptr<OdomExtrinsicInfo> info_ptr)=0;
	virtual ~VOStateSubscriber(){};
};

class VOStateSubscriber_callback: public  VOStateSubscriber{
public:
	VOStateSubscriber_callback(){};
	virtual void update_keyframe(std::shared_ptr<KeyframeInfo> kf_info_ptr){
		if(key_frame_info_f_){
			key_frame_info_f_(*kf_info_ptr);
		}
	}
	virtual void update_img(std::shared_ptr<ImageInfo> im_info_ptr){
		if(img_info_f_){
			img_info_f_(*im_info_ptr);
		}

	}
	virtual void update_odom_extrinsic(std::shared_ptr<OdomExtrinsicInfo> info_ptr){
		if (odom_extric_f_){
			odom_extric_f_(*info_ptr);
		}
	}
	virtual ~VOStateSubscriber_callback(){};

	std::function<void(OdomExtrinsicInfo &)> odom_extric_f_;
	std::function<void(KeyframeInfo &)> key_frame_info_f_;
	std::function<void(ImageInfo &)> img_info_f_;
};

class VOStateSubscribers {
public:
	VOStateSubscribers(){};
	void update_keyframe(const Estimator &estimator);
	void update_img(double t, const cv::Mat &img);
	void update_odom_extrinsic(const Estimator &estimator);
	virtual ~VOStateSubscribers(){};
	void register_sub(std::shared_ptr<VOStateSubscriber> sub_ptr){
		subs_.push_back(sub_ptr);
	};
	void do_callback();
	std::vector<std::function<void(VOInfo)>>vo_callback_subs_;
private:
	VOInfo vo_info_;
	std::vector<std::shared_ptr<VOStateSubscriber>> subs_;
};


#endif /* VINS_FUSION_VINS_ESTIMATOR_SRC_ESTIMATOR_VOSTATESUBSCRIBER_H_ */
