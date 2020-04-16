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

#define PCL_NO_PRECOMPILE
//#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include "parameters.h"

struct VSlamPoint
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  uint32_t id;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
};

POINT_CLOUD_REGISTER_POINT_STRUCT (VSlamPoint,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (uint32_t, id, id)
)

class CloudPointMap {
private:
	std::string mpose_graph_path;

public:
	pcl::PointCloud<VSlamPoint>::Ptr mcloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr mcloudxyz;
	CloudPointMap(std::string pose_graph_path);
	virtual ~CloudPointMap();

	void loadPointCloud();
	void saveMap(std::list<KeyFrame*> &keyframelist);
};

#endif /* VINS_FUSION_LOOP_FUSION_CLOUDPOINTMAP_H_ */
