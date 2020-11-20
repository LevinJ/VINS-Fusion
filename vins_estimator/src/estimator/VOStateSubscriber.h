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
class Estimator;

//class VOStates{
//public:
//	VOStates();
//	virtual ~VOStates();
//
//};

class VoInfo{
public:
	VoInfo(){
		t_ = 0;
	};
	virtual ~VoInfo(){};
	double t_;
};
class KeyframeInfo: public VoInfo{
public:
	KeyframeInfo(){};
	virtual ~KeyframeInfo(){};
	Eigen::Vector3d P_;
	Eigen::Quaterniond R_;
	std::vector<Eigen::Matrix<double, 8, 1>> pnt_3d2ds_;

};

class ImageInfo: public VoInfo{
public:
	ImageInfo(){};
	virtual ~ImageInfo(){};
	cv::Mat img_;

};

class OdomExtrinsicInfo: public VoInfo{
public:
	OdomExtrinsicInfo(){};
	virtual ~OdomExtrinsicInfo(){};
	Eigen::Vector3d P_;
	Eigen::Vector3d V_;
	Eigen::Matrix3d R_;
	Eigen::Matrix3d ric_;
	Eigen::Vector3d tic_;
};




class VOStateSubscriber {
public:
	VOStateSubscriber(){};
	virtual void update_keyframe(std::shared_ptr<KeyframeInfo> kf_info_ptr)=0;
	virtual void update_img(std::shared_ptr<ImageInfo> im_info_ptr)=0;
	virtual void update_odom_extrinsic(std::shared_ptr<OdomExtrinsicInfo> info_ptr)=0;
	virtual ~VOStateSubscriber(){};
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
private:
	std::vector<std::shared_ptr<VOStateSubscriber>> subs_;
};


#endif /* VINS_FUSION_VINS_ESTIMATOR_SRC_ESTIMATOR_VOSTATESUBSCRIBER_H_ */
