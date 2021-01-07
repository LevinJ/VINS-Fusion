/*
 * PoseInfo.h
 *
 *  Created on: Dec 25, 2020
 *      Author: levin
 */

#ifndef LOOP_FUSION_SRC_UTILITY_POSEINFO_H_
#define LOOP_FUSION_SRC_UTILITY_POSEINFO_H_

#include <eigen3/Eigen/Dense>
#include "../../../vins_estimator/src/utility/utility.h"

class PoseInfo {
public:
	PoseInfo(std::string name = "pose"){
		name_ = name;
	}
	virtual ~PoseInfo(){

	}
	Eigen::Matrix3d R_;
	Eigen::Vector3d t_;
	Eigen::Matrix4d T_;
	Eigen::Quaterniond q_;
	Eigen::Vector3d ypr_;
	std::string name_;
	std::string repr_;

	PoseInfo & construct_fromRt(const Eigen::Matrix3d &R =  Eigen::Matrix3d::Identity(), const Eigen::Vector3d &t= Eigen::Vector3d::Zero()){
		R_ = R;
		t_ = t;
		T_ = Eigen::Matrix4d::Identity();
		T_.block<3, 3>(0, 0) = R;
		T_.block<3, 1>(0, 3) = t;
		q_ = R;
		ypr_ = Utility::R2ypr(R);
		std::stringstream ss;
		ss.precision(2);
		ss << std::fixed<<"name:"<<name_<<",t:" <<t_.transpose()<<",ypr:"<<ypr_.transpose();
		repr_ = ss.str();
		return *this;
	}

	PoseInfo & construct_fromT(const Eigen::Matrix4d &T =  Eigen::Matrix4d::Identity()){
		auto R = T.block<3, 3>(0, 0);
		auto t = T.block<3, 1>(0, 3);
		construct_fromRt(R, t);
		return *this;
	}

	PoseInfo & construct_fromyprt(const Eigen::Vector3d &ypr= Eigen::Vector3d::Zero(), const Eigen::Vector3d &t= Eigen::Vector3d::Zero()){
		auto R = Utility::ypr2R(ypr);
		construct_fromRt(R, t);
		return *this;
	}

	PoseInfo & construct_fromqt(const Eigen::Quaterniond &q= Eigen::Quaterniond::Identity(), const Eigen::Vector3d &t= Eigen::Vector3d::Zero()){
		construct_fromRt(q.toRotationMatrix(), t);
		return *this;
	}

	virtual PoseInfo& operator = (const PoseInfo& other) {
		R_ = other.R_;
		t_ = other.t_;
		T_ = other.T_;
		q_ = other.q_;
		ypr_ = other.ypr_;
		name_ = other.name_;
		repr_ = other.repr_;
		return *this;
	}

	virtual PoseInfo operator * (const PoseInfo& other) {
		PoseInfo temp;
		temp.construct_fromT(T_ * other.T_);
		return temp;
	}

	virtual PoseInfo  I() {
		PoseInfo temp;
		temp.construct_fromT(T_.inverse());
		return temp;
	}

};

#endif /* LOOP_FUSION_SRC_UTILITY_POSEINFO_H_ */
