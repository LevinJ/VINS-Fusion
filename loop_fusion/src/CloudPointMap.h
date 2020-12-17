/*
 * CloudPointMap.h
 *
 *  Created on: Apr 16, 2020
 *      Author: levin
 */

#ifndef VINS_FUSION_LOOP_FUSION_CLOUDPOINTMAP_H_
#define VINS_FUSION_LOOP_FUSION_CLOUDPOINTMAP_H_

#include <list>
#include<string>
#include "keyframe.h"

#include "pose_graph.h"
#include <map>
#include <vector>

class CloudPointMap: public PoseGraph {
protected:
	std::map<double, std::vector<double>> point_map_;
	int detectLoop(KeyFrame* keyframe, int frame_index);
	void addKeyFrame(KeyFrame* cur_kf, bool flag_detect_loop);
public:
	void loadPointCloud();
	#ifndef WITH_ROS_SIMULATE
	void publish_cloudponint(ros::Publisher &_pub_base_point_cloud);
	#endif

	void reloc_frame(KeyFrame* keyframe);
	CloudPointMap();
	virtual ~CloudPointMap();
	void loadPoseGraph();

	void saveMap(std::list<KeyFrame*> &keyframelist);
private:
	std::string point_cloud_path_;
};

#endif /* VINS_FUSION_LOOP_FUSION_CLOUDPOINTMAP_H_ */
