/*
 * LoopInfoLogging.cpp
 *
 *  Created on: May 19, 2020
 *      Author: levin
 */

#include "LoopInfoLogging.h"
//#include "../parameters.h"
#include "utility.h"
#include <fstream>
using namespace std;

extern Eigen::Vector3d tic;
extern Eigen::Matrix3d qic;

LoopInfoLogging::LoopInfoLogging() {
	// TODO Auto-generated constructor stub
	conn_info_log_ = "/home/levin/raw_data/loop_detecton_info.csv";
}
void LoopInfoLogging::init(){
	std::ofstream fout(conn_info_log_, std::ios::out);
	fout << "cur_time,map_time,match_file,x,y,z,q_x,q_y,q_z,q_w,rel_yaw,tic_x,tic_y,tic_z,qic_y,qic_p,qic_r"<<endl;
	fout.close();

}
void LoopInfoLogging::append_loopinfo(double cur_time, double map_time, const std::string &match_file, const Eigen::Matrix<double, 8, 1 > &loop_info){
	ofstream loop_path_file(conn_info_log_, ios::app);
	loop_path_file.setf(ios::fixed, ios::floatfield);
	loop_path_file.precision(6);
	loop_path_file<<cur_time<<","<<map_time<<","<<match_file<<",";
	loop_path_file << loop_info[0] << ","<< loop_info[1] << ","<< loop_info[2] << ","
			<< loop_info[4] << ","<< loop_info[5] << ","<< loop_info[6] << ","<< loop_info[3] << ","
			<< loop_info[7]<< "," ;

	auto ypr = Utility::R2ypr(qic);
	loop_path_file<<tic[0]<< ","<<tic[1]<< ","<<tic[2]<< ",";
	loop_path_file<<ypr[0]<< ","<<ypr[1]<< ","<<ypr[2];
	loop_path_file<< endl;
	loop_path_file.close();

}
LoopInfoLogging::~LoopInfoLogging() {
	// TODO Auto-generated destructor stub
}

