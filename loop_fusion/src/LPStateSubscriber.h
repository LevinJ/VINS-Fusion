/*
 * LPStateSubscriber.h
 *
 *  Created on: Nov 20, 2020
 *      Author: levin
 */

#ifndef LOOP_FUSION_SRC_LPSTATESUBSCRIBER_H_
#define LOOP_FUSION_SRC_LPSTATESUBSCRIBER_H_

#include <memory>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
class KeyFrame;
class LPInfo{
public:
	double t_;
	Eigen::Vector3d P_;
	Eigen::Matrix3d R_;
	Eigen::Matrix3d ric_;
	Eigen::Vector3d tic_;
	cv::Mat lp_detection_img_;
	cv::Mat lp_matching_img_;
	cv::Mat find_conn_1_;
	cv::Mat find_conn_2_;
	cv::Mat find_conn_3_;
	bool bloop_detected_;
	bool bconn_founded_;
};
class LPStateSubscriber {
public:
	LPStateSubscriber();
	virtual ~LPStateSubscriber();
	void update_loop_info(KeyFrame * cur_kf, KeyFrame * old_kf, const std::string &match_file);
};

class LPStateSubscribers {
public:
	LPStateSubscribers();
	virtual ~LPStateSubscribers();
	void register_sub(std::shared_ptr<LPStateSubscriber> sub_ptr){
		subs_.push_back(sub_ptr);
	};
	void update_loop_info(KeyFrame * cur_kf, KeyFrame * old_kf, const std::string &match_file);
	void do_callback();
	LPInfo lp_info_;
	std::vector<std::function<void(LPInfo)>>lp_state_subs2;
private:
	std::vector<std::shared_ptr<LPStateSubscriber>> subs_;

};

#endif /* LOOP_FUSION_SRC_LPSTATESUBSCRIBER_H_ */
