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
private:
	std::vector<std::shared_ptr<LPStateSubscriber>> subs_;
};

#endif /* LOOP_FUSION_SRC_LPSTATESUBSCRIBER_H_ */
