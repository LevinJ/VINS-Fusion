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


CloudPointMap::CloudPointMap(): PoseGraph(){
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

	 pcl::io::savePCDFileASCII (POSE_GRAPH_SAVE_PATH + "vlsam_pcd.pcd", cloud);
	 std::cerr << "Saved " << cloud.points.size () << " data points to " << POSE_GRAPH_SAVE_PATH + "vlsam_pcd.pcd" << std::endl;
}

void CloudPointMap::loadPointCloud(){
	std::string file = POSE_GRAPH_SAVE_PATH + "vlsam_pcd.pcd";
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

void CloudPointMap::loadPoseGraph()
{
    TicToc tmp_t;
    FILE * pFile;
    string file_path = POSE_GRAPH_SAVE_PATH + "pose_graph.txt";
    printf("lode pose graph from: %s \n", file_path.c_str());
    printf("pose graph loading...\n");
    pFile = fopen (file_path.c_str(),"r");
    if (pFile == NULL)
    {
        printf("lode previous pose graph error: wrong previous pose graph path or no previous pose graph \n the system will start with new pose graph \n");
        return;
    }
    int index;
    double time_stamp;
    double VIO_Tx, VIO_Ty, VIO_Tz;
    double PG_Tx, PG_Ty, PG_Tz;
    double VIO_Qw, VIO_Qx, VIO_Qy, VIO_Qz;
    double PG_Qw, PG_Qx, PG_Qy, PG_Qz;
    double loop_info_0, loop_info_1, loop_info_2, loop_info_3;
    double loop_info_4, loop_info_5, loop_info_6, loop_info_7;
    int loop_index;
    int keypoints_num;
    Eigen::Matrix<double, 8, 1 > loop_info;
    int cnt = 0;
    while (fscanf(pFile,"%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %d", &index, &time_stamp,
                                    &VIO_Tx, &VIO_Ty, &VIO_Tz,
                                    &PG_Tx, &PG_Ty, &PG_Tz,
                                    &VIO_Qw, &VIO_Qx, &VIO_Qy, &VIO_Qz,
                                    &PG_Qw, &PG_Qx, &PG_Qy, &PG_Qz,
                                    &loop_index,
                                    &loop_info_0, &loop_info_1, &loop_info_2, &loop_info_3,
                                    &loop_info_4, &loop_info_5, &loop_info_6, &loop_info_7,
                                    &keypoints_num) != EOF)
    {
        /*
        printf("I read: %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %d\n", index, time_stamp,
                                    VIO_Tx, VIO_Ty, VIO_Tz,
                                    PG_Tx, PG_Ty, PG_Tz,
                                    VIO_Qw, VIO_Qx, VIO_Qy, VIO_Qz,
                                    PG_Qw, PG_Qx, PG_Qy, PG_Qz,
                                    loop_index,
                                    loop_info_0, loop_info_1, loop_info_2, loop_info_3,
                                    loop_info_4, loop_info_5, loop_info_6, loop_info_7,
                                    keypoints_num);
        */
        cv::Mat image;
        std::string image_path, descriptor_path;
        if (DEBUG_IMAGE)
        {
            image_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_image.png";
            image = cv::imread(image_path.c_str(), 0);
        }

        Vector3d VIO_T(VIO_Tx, VIO_Ty, VIO_Tz);
        Vector3d PG_T(PG_Tx, PG_Ty, PG_Tz);
        Quaterniond VIO_Q;
        VIO_Q.w() = VIO_Qw;
        VIO_Q.x() = VIO_Qx;
        VIO_Q.y() = VIO_Qy;
        VIO_Q.z() = VIO_Qz;
        Quaterniond PG_Q;
        PG_Q.w() = PG_Qw;
        PG_Q.x() = PG_Qx;
        PG_Q.y() = PG_Qy;
        PG_Q.z() = PG_Qz;
        Matrix3d VIO_R, PG_R;
        VIO_R = VIO_Q.toRotationMatrix();
        PG_R = PG_Q.toRotationMatrix();
        Eigen::Matrix<double, 8, 1 > loop_info;
        loop_info << loop_info_0, loop_info_1, loop_info_2, loop_info_3, loop_info_4, loop_info_5, loop_info_6, loop_info_7;

        if (loop_index != -1)
            if (earliest_loop_index > loop_index || earliest_loop_index == -1)
            {
                earliest_loop_index = loop_index;
            }

        // load keypoints, brief_descriptors
        string brief_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_briefdes.dat";
        std::ifstream brief_file(brief_path, std::ios::binary);
        string keypoints_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_keypoints.txt";
        FILE *keypoints_file;
        keypoints_file = fopen(keypoints_path.c_str(), "r");
        vector<cv::KeyPoint> keypoints;
        vector<cv::KeyPoint> keypoints_norm;
        vector<BRIEF::bitset> brief_descriptors;
        for (int i = 0; i < keypoints_num; i++)
        {
            BRIEF::bitset tmp_des;
            brief_file >> tmp_des;
            brief_descriptors.push_back(tmp_des);
            cv::KeyPoint tmp_keypoint;
            cv::KeyPoint tmp_keypoint_norm;
            double p_x, p_y, p_x_norm, p_y_norm;
            if(!fscanf(keypoints_file,"%lf %lf %lf %lf", &p_x, &p_y, &p_x_norm, &p_y_norm))
                printf(" fail to load pose graph \n");
            tmp_keypoint.pt.x = p_x;
            tmp_keypoint.pt.y = p_y;
            tmp_keypoint_norm.pt.x = p_x_norm;
            tmp_keypoint_norm.pt.y = p_y_norm;
            keypoints.push_back(tmp_keypoint);
            keypoints_norm.push_back(tmp_keypoint_norm);
        }
        brief_file.close();
        fclose(keypoints_file);

        KeyFrame* keyframe = new KeyFrame(time_stamp, index, VIO_T, VIO_R, PG_T, PG_R, image, loop_index, loop_info, keypoints, keypoints_norm, brief_descriptors);
        loadKeyFrame(keyframe, 0);
//        if (cnt % 20 == 0)
//        {
//            publish();
//        }
        cnt++;
    }
    fclose (pFile);
    loadPointCloud();
    printf("load pose graph time: %f s\n", tmp_t.toc()/1000);
    base_sequence = 0;
}



