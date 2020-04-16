/*
 * CloudPointMap.cpp
 *
 *  Created on: Apr 16, 2020
 *      Author: levin
 */

#include "CloudPointMap.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;


CloudPointMap::CloudPointMap(std::string pose_graph_path): mpose_graph_path(pose_graph_path) {
	// TODO Auto-generated constructor stub
	mcloud.reset(new  pcl::PointCloud<VSlamPoint>());
	mcloudxyz.reset(new  pcl::PointCloud<pcl::PointXYZ>());
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

	 pcl::io::savePCDFileASCII (mpose_graph_path + "vlsam_pcd.pcd", cloud);
	 std::cerr << "Saved " << cloud.points.size () << " data points to " << mpose_graph_path + "vlsam_pcd.pcd" << std::endl;
}

void CloudPointMap::loadPointCloud(){
	std::string file = mpose_graph_path + "vlsam_pcd.pcd";
	if (pcl::io::loadPCDFile<VSlamPoint> (file, *mcloud) == -1) //* load the file
	{
		cout<<"Couldn't read file "<<file<<endl;
		return;
	}
	for(unsigned int i=0; i < mcloud->size(); i++){

		pcl::PointXYZ pnt;
		pnt.x = mcloud->points[i].x;
		pnt.y = mcloud->points[i].y;
		pnt.z = mcloud->points[i].z;
		mcloudxyz->push_back(pnt);

	}
	std::cout << "Loaded " << mcloud->points.size () << " data points from "+file << std::endl;
}
CloudPointMap::~CloudPointMap() {
	// TODO Auto-generated destructor stub
}

