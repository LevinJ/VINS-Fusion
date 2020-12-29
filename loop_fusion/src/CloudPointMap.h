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
#include <queue>
class ImageInfo;

class CloudPointMap: public PoseGraph {
protected:
	std::map<double, std::vector<double>> point_map_;
	int detectLoop(KeyFrame* keyframe, int frame_index);
public:
	void init(std::string config_file, std::string pkg_path);
	void loadPointCloud();
	#ifndef WITH_ROS_SIMULATE
	void publish_cloudponint(ros::Publisher &_pub_base_point_cloud);
	#endif
	void reloc(double _time_stamp, cv::Mat &_image);
	CloudPointMap();
	virtual ~CloudPointMap();
	void loadPoseGraph();

	void saveMap(std::list<KeyFrame*> &keyframelist);
private:
	std::thread process_thread_;
	std::mutex buf_mutext_;
	queue<std::shared_ptr<ImageInfo>> image_buf_;
	void process();
	void process_img(std::shared_ptr<ImageInfo> img_info);
	std::string point_cloud_path_;
	//store loop detection result
	cv::Mat loop_result_;
};

#endif /* VINS_FUSION_LOOP_FUSION_CLOUDPOINTMAP_H_ */
