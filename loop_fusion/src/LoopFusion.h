/*
 * LoopFusion.h
 *
 *  Created on: Nov 13, 2020
 *      Author: levin
 */

#ifndef VINS_FUSION_LOOP_FUSION_SRC_LOOPFUSION_H_
#define VINS_FUSION_LOOP_FUSION_SRC_LOOPFUSION_H_
#include "pose_graph.h"

class Estimator;
class VOStateSubscriberLoop;

class LoopFusion {
public:
	LoopFusion();
	void init(std::string pkg_path, Estimator &est);
	virtual ~LoopFusion();
	PoseGraph posegraph_;
	void create_map();

private:
	void new_sequence();
	void process();
	void command();
	void init_pose_graph();
	std::string pkg_path_;
	std::shared_ptr<VOStateSubscriberLoop> sub_ptr_;
	std::thread cmd_thread_;
	std::thread process_thread_;

};

#endif /* VINS_FUSION_LOOP_FUSION_SRC_LOOPFUSION_H_ */
