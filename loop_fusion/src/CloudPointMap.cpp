/*
 * CloudPointMap.cpp
 *
 *  Created on: Apr 16, 2020
 *      Author: levin
 */

#include "CloudPointMap.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

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


CloudPointMap::CloudPointMap() {
	// TODO Auto-generated constructor stub

}
void CloudPointMap::saveMap(std::list<KeyFrame*> &keyframelist){
	pcl::PointCloud<VSlamPoint> cloud;
	 std::set<int> idset;

	 for (list<KeyFrame*>::iterator it = keyframelist.begin(); it != keyframelist.end(); it++){
		 vector<cv::Point3f> &point_3d = (*it)->point_3d;
		 vector<cv::Point2f> &point_2d_uv = (*it)->point_2d_uv;
		 vector<double> &point_id = (*it)->point_id;
		 for(unsigned int i=0; i<point_id.size(); i++ ){
			 auto it = idset.find(point_id[i]);
			 if(it != idset.end()){
				 //the point has already been inserted before
				 continue;
			 }
			 idset.insert(point_id[i]);
			 VSlamPoint point;

			 point.x = point_3d[i].x;
			 point.y = point_3d[i].y;
			 point.z = point_3d[i].z;
			 point.id = point_id[i];

			 cloud.push_back(point);
		 }
	 }

	 pcl::io::savePCDFileASCII (POSE_GRAPH_SAVE_PATH + "vlsam_pcd.pcd", cloud);
	 std::cerr << "Saved " << cloud.points.size () << " data points to " << POSE_GRAPH_SAVE_PATH + "vlsam_pcd.pcd" << std::endl;
}
CloudPointMap::~CloudPointMap() {
	// TODO Auto-generated destructor stub
}

