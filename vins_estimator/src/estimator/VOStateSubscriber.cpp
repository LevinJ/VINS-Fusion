/*
 * VOStateSubscriber.cpp
 *
 *  Created on: Nov 16, 2020
 *      Author: levin
 */

#include "VOStateSubscriber.h"
#include "estimator.h"

using namespace Eigen;




void VOStateSubscribers::update_img(double t,  const cv::Mat &img){
	//send the data out
	auto im_info_ptr = std::make_shared<ImageInfo>();
	im_info_ptr->t_ = t;
	im_info_ptr->img_ = img;
	for(auto &sub : subs_){
		sub->update_img(im_info_ptr);
	}
}
void VOStateSubscribers::update_keyframe(const Estimator &estimator){
	// pub camera pose, 2D-3D points of keyframe
	if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR || estimator.marginalization_flag != 0){
		return;
	}

	int i = WINDOW_SIZE - 2;

	std::shared_ptr<KeyframeInfo> kf_info = std::make_shared<KeyframeInfo>();
	kf_info->t_ = estimator.Headers[WINDOW_SIZE - 2];
	kf_info->P_ = estimator.Ps[i];
	kf_info->R_ = Quaterniond(estimator.Rs[i]);
	for (auto &it_per_id : estimator.f_manager.feature)
	{
		int frame_size = it_per_id.feature_per_frame.size();
		if(it_per_id.start_frame < WINDOW_SIZE - 2 && it_per_id.start_frame + frame_size - 1 >= WINDOW_SIZE - 2 && it_per_id.solve_flag == 1)
		{

			int imu_i = it_per_id.start_frame;
			Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
			Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0])
	                                    				  + estimator.Ps[imu_i];
			int imu_j = WINDOW_SIZE - 2 - it_per_id.start_frame;

			Eigen::Matrix<double, 8, 1> pnt;
			pnt<<w_pts_i(0),w_pts_i(1),w_pts_i(2),it_per_id.feature_per_frame[imu_j].point.x(),
					it_per_id.feature_per_frame[imu_j].point.y(),it_per_id.feature_per_frame[imu_j].uv.x(),
					it_per_id.feature_per_frame[imu_j].uv.y(),it_per_id.feature_id;
			kf_info->pnt_3d2ds_.push_back(pnt);

		}

	}
	//send the data out
	for(auto &sub : subs_){
		sub->update_keyframe(kf_info);
	}

}

void VOStateSubscribers::update_odom_extrinsic(const Estimator &estimator){

	if (estimator.solver_flag != Estimator::SolverFlag::NON_LINEAR){
		return;
	}

	auto info_ptr = std::make_shared<OdomExtrinsicInfo>();
	info_ptr->t_ = estimator.Headers[WINDOW_SIZE];
	info_ptr->P_ = estimator.Ps[WINDOW_SIZE];
	info_ptr->V_ = estimator.Vs[WINDOW_SIZE];
	info_ptr->R_ = estimator.Rs[WINDOW_SIZE];

	info_ptr->ric_ = estimator.ric[0];
	info_ptr->tic_ = estimator.tic[0];
	//send the data out
	for(auto &sub : subs_){
		sub->update_odom_extrinsic(info_ptr);
	}
}

