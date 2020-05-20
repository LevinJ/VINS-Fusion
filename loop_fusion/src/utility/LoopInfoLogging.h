/*
 * LoopInfoLogging.h
 *
 *  Created on: May 19, 2020
 *      Author: levin
 */

#ifndef VINS_FUSION_LOOP_FUSION_SRC_UTILITY_LOOPINFOLOGGING_H_
#define VINS_FUSION_LOOP_FUSION_SRC_UTILITY_LOOPINFOLOGGING_H_

#include <eigen3/Eigen/Dense>

class LoopInfoLogging {
private:

public:
	LoopInfoLogging();
	void init();
	void append_loopinfo(double cur_time, double map_time, const std::string &match_file, const Eigen::Matrix<double, 8, 1 > &loop_info);
	virtual ~LoopInfoLogging();
};

#endif /* VINS_FUSION_LOOP_FUSION_SRC_UTILITY_LOOPINFOLOGGING_H_ */
