/*
 * LPStateSubscriber.cpp
 *
 *  Created on: Nov 20, 2020
 *      Author: levin
 */

#include "LPStateSubscriber.h"
#include "keyframe.h"
#include <iostream>
using namespace std;
extern Eigen::Vector3d tic;
extern Eigen::Matrix3d qic;

LPStateSubscribers g_lp_state_subscriber;
LPStateSubscriber::LPStateSubscriber() {
	// TODO Auto-generated constructor stub

}

LPStateSubscriber::~LPStateSubscriber() {
	// TODO Auto-generated destructor stub
}
void LPStateSubscriber::update_loop_info(KeyFrame * cur_kf, KeyFrame * old_kf, const std::string &match_file){
	stringstream ss;
//	ss.precision(6);
	ss<<std::fixed<<setprecision(6);
	ss << "valid loop detected, " <<cur_kf->sequence<<", "<< cur_kf->index<< "-->"<< old_kf->sequence<<", "<<old_kf->index<<", ";

	auto &loop_info = cur_kf->loop_info;
	auto &cur_time = cur_kf->time_stamp;
	auto &map_time = old_kf->time_stamp;
	ss<<cur_time<<","<<map_time<<","<<match_file<<",";
	ss <<" xyz="<< loop_info[0] << ","<< loop_info[1] << ","<< loop_info[2] << ","
			<<"q="<< loop_info[4] << ","<< loop_info[5] << ","<< loop_info[6] << ","<< loop_info[3] << ","
			<<"yaw="<< loop_info[7]<< "," ;

	auto ypr = Utility::R2ypr(qic);
	ss<<"tic="<<tic[0]<< ","<<tic[1]<< ","<<tic[2]<< ",";
	ss<<"ric="<<ypr[0]<< ","<<ypr[1]<< ","<<ypr[2];
	ss<< endl;
	cout<<ss.str();
}

LPStateSubscribers::LPStateSubscribers() {
	// TODO Auto-generated constructor stub
	register_sub(std::make_shared<LPStateSubscriber>());
}

LPStateSubscribers::~LPStateSubscribers() {
	// TODO Auto-generated destructor stub
}

void LPStateSubscribers::do_callback(){
	for(auto &sub : lp_state_subs2){
		sub(lp_info_);
	}
}

void LPStateSubscribers::update_loop_info(KeyFrame * cur_kf, KeyFrame * old_kf, const std::string &match_file){
	for(auto &sub : subs_){
		sub->update_loop_info(cur_kf, old_kf, match_file);
	}
}

