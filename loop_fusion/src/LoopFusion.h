/*
 * LoopFusion.h
 *
 *  Created on: Nov 13, 2020
 *      Author: levin
 */

#ifndef VINS_FUSION_LOOP_FUSION_SRC_LOOPFUSION_H_
#define VINS_FUSION_LOOP_FUSION_SRC_LOOPFUSION_H_
//#include "../../vins_estimator/src/estimator/estimator.h"
#include "pose_graph.h"
class LoopFusion {
public:
	LoopFusion(std::string pkg_path);
	virtual ~LoopFusion();
	PoseGraph posegraph_;
private:
	void init_params();
	std::string pkg_path_;

};

#endif /* VINS_FUSION_LOOP_FUSION_SRC_LOOPFUSION_H_ */
