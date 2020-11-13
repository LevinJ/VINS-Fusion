/*
 * CloudPointMap.cpp
 *
 *  Created on: Apr 16, 2020
 *      Author: levin
 */

#include "CloudPointMap.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "utility/utility.h"

using namespace std;

extern int DEBUG_IMAGE;
extern std::string POSE_GRAPH_SAVE_PATH;

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

	 // write point_id, point_2d_uv, point_2d_norm
	 for (list<KeyFrame*>::iterator it = keyframelist.begin(); it != keyframelist.end(); it++){
		 auto keypoints_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_keypoints_window.txt";
		 auto keypoints_file = fopen(keypoints_path.c_str(), "w");

		 auto pointid_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_pointid.txt";
		 auto pointid_file = fopen(pointid_path.c_str(), "w");

		 for (int i = 0; i < (int)(*it)->point_id.size(); i++)
		 {
			 fprintf(pointid_file, "%f\n", (*it)->point_id[i]);

			 fprintf(keypoints_file, "%f %f %f %f\n", (*it)->point_2d_uv[i].x, (*it)->point_2d_uv[i].y,
			                                                     (*it)->point_2d_norm[i].x, (*it)->point_2d_norm[i].y);
		 }
		 fclose(keypoints_file);
		 fclose(pointid_file);
	 }
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
		uint32_t id = mcloud->points[i].id;
		if(mpointidmap.find(id) == mpointidmap.end()){
			mpointidmap[id] = {pnt.x, pnt.y,pnt.z};
		}


	}
	std::cout << "Loaded " << mcloud->points.size () << " data points from "+file << std::endl;
}
#ifndef WITH_ROS_SIMULATE
void CloudPointMap::publish_cloudponint(ros::Publisher &_pub_base_point_cloud){
	sensor_msgs::PointCloud point_cloud;
	point_cloud.header.frame_id = "world";
	point_cloud.header.stamp = ros::Time::now();
	for(unsigned int i=0; i < mcloud->size(); i++){
		geometry_msgs::Point32 pnt;
		pnt.x = mcloud->points[i].x;
		pnt.y = mcloud->points[i].y;
		pnt.z = mcloud->points[i].z;
		point_cloud.points.push_back(pnt);
	}
	_pub_base_point_cloud.publish(point_cloud);
}
#endif
CloudPointMap::~CloudPointMap() {
	// TODO Auto-generated destructor stub
}

void CloudPointMap::loadPoseGraph()
{
	loadPointCloud();
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
    	if(index < 1){
    		continue;
    	}
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

        // load window keypoints, 3dpoint, pointid
//        string brief_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_briefdes.dat";
//        std::ifstream brief_file(brief_path, std::ios::binary);
//        string keypoints_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_keypoints_window.txt";
//        FILE *keypoints_file;
//        keypoints_file = fopen(keypoints_path.c_str(), "r");
//        vector<cv::KeyPoint> keypoints;
//        vector<cv::KeyPoint> keypoints_norm;
//        vector<BRIEF::bitset> brief_descriptors;
//        for (int i = 0; i < keypoints_num; i++)
//        {
//            BRIEF::bitset tmp_des;
//            brief_file >> tmp_des;
//            brief_descriptors.push_back(tmp_des);
//            cv::KeyPoint tmp_keypoint;
//            cv::KeyPoint tmp_keypoint_norm;
//            double p_x, p_y, p_x_norm, p_y_norm;
//            if(!fscanf(keypoints_file,"%lf %lf %lf %lf", &p_x, &p_y, &p_x_norm, &p_y_norm))
//                printf(" fail to load pose graph \n");
//            tmp_keypoint.pt.x = p_x;
//            tmp_keypoint.pt.y = p_y;
//            tmp_keypoint_norm.pt.x = p_x_norm;
//            tmp_keypoint_norm.pt.y = p_y_norm;
//            keypoints.push_back(tmp_keypoint);
//            keypoints_norm.push_back(tmp_keypoint_norm);
//        }
//        brief_file.close();
//        fclose(keypoints_file);

        vector<cv::Point3f> point_3d;
		vector<cv::Point2f> point_2d_uv;
		vector<cv::Point2f> point_2d_normal;
		vector<double> point_id;

		auto keypoints_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_keypoints_window.txt";
		auto keypoints_file = fopen(keypoints_path.c_str(), "r");
		auto pointid_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_pointid.txt";
		auto pointid_file = fopen(pointid_path.c_str(), "r");

		double p_x, p_y, p_x_norm, p_y_norm;
		double id;
		std::vector<double> pnt;
		while(fscanf(keypoints_file,"%lf %lf %lf %lf", &p_x, &p_y, &p_x_norm, &p_y_norm) != EOF){
			if(!fscanf(pointid_file,"%lf", &id)){
				printf(" fail to load pose graph \n");
				return;
			}
			point_2d_uv.emplace_back(p_x, p_y);
			point_2d_normal.emplace_back(p_x_norm, p_y_norm);
			point_id.emplace_back(id);
			pnt = mpointidmap[id];
			point_3d.emplace_back(pnt[0], pnt[1],pnt[2]);

		}
		int sequence = 0;
		KeyFrame* keyframe = new KeyFrame(time_stamp, index, PG_T, PG_R, image,
									   point_3d, point_2d_uv, point_2d_normal, point_id, sequence);
		loadKeyFrame(keyframe, 0);

//        KeyFrame* keyframe = new KeyFrame(time_stamp, index, VIO_T, VIO_R, PG_T, PG_R, image, loop_index, loop_info, keypoints, keypoints_norm, brief_descriptors);
//        loadKeyFrame(keyframe, 0);
//        if (cnt % 20 == 0)
//        {
//            publish();
//        }
        cnt++;
    }
    fclose (pFile);
    printf("load pose graph time: %f s\n", tmp_t.toc()/1000);
    base_sequence = 0;
}

void CloudPointMap::reloc_frame(KeyFrame* keyframe){
	addKeyFrame(keyframe, 1);
}

void CloudPointMap::addKeyFrame(KeyFrame* cur_kf, bool flag_detect_loop)
{
    //shift to base frame
    Vector3d vio_P_cur;
    Matrix3d vio_R_cur;
    if (sequence_cnt != cur_kf->sequence)
    {
        sequence_cnt++;
        sequence_loop.push_back(0);
        w_t_vio = Eigen::Vector3d(0, 0, 0);
        w_r_vio = Eigen::Matrix3d::Identity();
        m_drift.lock();
        t_drift = Eigen::Vector3d(0, 0, 0);
        r_drift = Eigen::Matrix3d::Identity();
        m_drift.unlock();
    }

    cur_kf->getVioPose(vio_P_cur, vio_R_cur);
    vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
    vio_R_cur = w_r_vio *  vio_R_cur;
    cur_kf->updateVioPose(vio_P_cur, vio_R_cur);
    cur_kf->index = global_index;
    global_index++;
	int loop_index = -1;
    if (flag_detect_loop)
    {
        TicToc tmp_t;
        loop_index = detectLoop(cur_kf, cur_kf->index);
//        cout<<"detectloop="<<tmp_t.toc()<<endl;
    }
    else
    {
        addKeyFrameIntoVoc(cur_kf);
    }
	if (loop_index != -1)
	{
        printf(" %d detect loop with %d \n", cur_kf->index, loop_index);
        KeyFrame* old_kf = getKeyFrame(loop_index);
//        Vector3d w_P_loop;
//        Matrix3d w_R_loop;
//        old_kf->getPose(w_P_loop,w_R_loop );
//        cur_kf->updateVioPose(w_P_loop, w_R_loop);

        TicToc tmp_t;
        bool bfindconn = old_kf->findConnection(cur_kf);
//        bool bfindconn = cur_kf->findConnection(old_kf);
        cout<<"findConnection="<<tmp_t.toc()<<endl;
        if (bfindconn)
        {
            if (earliest_loop_index > loop_index || earliest_loop_index == -1)
                earliest_loop_index = loop_index;

            Vector3d w_P_old, w_P_cur, vio_P_cur;
            Matrix3d w_R_old, w_R_cur, vio_R_cur;
            old_kf->getVioPose(w_P_old, w_R_old);
            cur_kf->getVioPose(vio_P_cur, vio_R_cur);

            //from findconnection, we can relative pose, current <-- old, we will convert it old <--cur
            Vector3d relative_t;
            Quaterniond relative_q;
            relative_q = (old_kf->getLoopRelativeQ()).toRotationMatrix().transpose();
            relative_t = -(old_kf->getLoopRelativeQ()).toRotationMatrix().transpose() * old_kf->getLoopRelativeT();



            w_P_cur = w_R_old * relative_t + w_P_old;
            w_R_cur = w_R_old * relative_q;
            auto relative_yaw = Utility::normalizeAngle(Utility::R2ypr(w_R_cur).x() - Utility::R2ypr(w_R_old).x());
            cout << "current pos = " << w_P_cur.transpose()<<",yaw="<< relative_yaw << endl;

//            double shift_yaw;
//            Matrix3d shift_r;
//            Vector3d shift_t;
//            if(use_imu)
//            {
//                shift_yaw = Utility::R2ypr(w_R_cur).x() - Utility::R2ypr(vio_R_cur).x();
//                shift_r = Utility::ypr2R(Vector3d(shift_yaw, 0, 0));
//            }
//            else
//                shift_r = w_R_cur * vio_R_cur.transpose();
//            shift_t = w_P_cur - w_R_cur * vio_R_cur.transpose() * vio_P_cur;
//            // shift vio pose of whole sequence to the world frame
//            if (old_kf->sequence != cur_kf->sequence && sequence_loop[cur_kf->sequence] == 0)
//            {
//                w_r_vio = shift_r;
//                w_t_vio = shift_t;
//                vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
//                vio_R_cur = w_r_vio *  vio_R_cur;
//                cur_kf->updateVioPose(vio_P_cur, vio_R_cur);
//                list<KeyFrame*>::iterator it = keyframelist.begin();
//                for (; it != keyframelist.end(); it++)
//                {
//                    if((*it)->sequence == cur_kf->sequence)
//                    {
//                        Vector3d vio_P_cur;
//                        Matrix3d vio_R_cur;
//                        (*it)->getVioPose(vio_P_cur, vio_R_cur);
//                        vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
//                        vio_R_cur = w_r_vio *  vio_R_cur;
//                        (*it)->updateVioPose(vio_P_cur, vio_R_cur);
//                    }
//                }
//                sequence_loop[cur_kf->sequence] = 1;
//            }
////            m_optimize_buf.lock();
////            optimize_buf.push(cur_kf->index);
////            m_optimize_buf.unlock();
        }
	}
//	m_keyframelist.lock();
//    Vector3d P;
//    Matrix3d R;
//    cur_kf->getVioPose(P, R);
//    P = r_drift * P + t_drift;
//    R = r_drift * R;
//    cur_kf->updatePose(P, R);
//    Quaterniond Q{R};
//    geometry_msgs::PoseStamped pose_stamped;
//    pose_stamped.header.stamp = ros::Time(cur_kf->time_stamp);
//    pose_stamped.header.frame_id = "world";
//    pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
//    pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
//    pose_stamped.pose.position.z = P.z();
//    pose_stamped.pose.orientation.x = Q.x();
//    pose_stamped.pose.orientation.y = Q.y();
//    pose_stamped.pose.orientation.z = Q.z();
//    pose_stamped.pose.orientation.w = Q.w();
//    path[sequence_cnt].poses.push_back(pose_stamped);
//    path[sequence_cnt].header = pose_stamped.header;
//
//    if (SAVE_LOOP_PATH)
//    {
//        ofstream loop_path_file(VINS_RESULT_PATH, ios::app);
//        loop_path_file.setf(ios::fixed, ios::floatfield);
//        //TUM format
//        loop_path_file.precision(6);
//		loop_path_file << cur_kf->time_stamp << " ";
//		loop_path_file.precision(7);
//		loop_path_file  << P.x() << " "
//			  << P.y() << " "
//			  << P.z() << " "
//			  << Q.x() << " "
//			  << Q.y() << " "
//			  << Q.z() << " "
//			  << Q.w()<< endl;
//
//        loop_path_file.close();
//    }
//    //draw local connection
//    if (SHOW_S_EDGE)
//    {
//        list<KeyFrame*>::reverse_iterator rit = keyframelist.rbegin();
//        for (int i = 0; i < 4; i++)
//        {
//            if (rit == keyframelist.rend())
//                break;
//            Vector3d conncected_P;
//            Matrix3d connected_R;
//            if((*rit)->sequence == cur_kf->sequence)
//            {
//                (*rit)->getPose(conncected_P, connected_R);
//                posegraph_visualization->add_edge(P, conncected_P);
//            }
//            rit++;
//        }
//    }
//    if (SHOW_L_EDGE)
//    {
//        if (cur_kf->has_loop)
//        {
//            //printf("has loop \n");
//            KeyFrame* connected_KF = getKeyFrame(cur_kf->loop_index);
//            Vector3d connected_P,P0;
//            Matrix3d connected_R,R0;
//            connected_KF->getPose(connected_P, connected_R);
//            //cur_kf->getVioPose(P0, R0);
//            cur_kf->getPose(P0, R0);
//            if(cur_kf->sequence > 0)
//            {
//                //printf("add loop into visual \n");
//                posegraph_visualization->add_loopedge(P0, connected_P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0));
//            }
//
//        }
//    }
//    //posegraph_visualization->add_pose(P + Vector3d(VISUALIZATION_SHIFT_X, VISUALIZATION_SHIFT_Y, 0), Q);
//
//	keyframelist.push_back(cur_kf);
//    publish();
//	m_keyframelist.unlock();
}


int CloudPointMap::detectLoop(KeyFrame* keyframe, int frame_index)
{
    // put image into image_pool; for visualization
    cv::Mat compressed_image;
    if (DEBUG_IMAGE)
    {
        int feature_num = keyframe->keypoints.size();
        cv::resize(keyframe->image, compressed_image, cv::Size(376, 240));
        putText(compressed_image, "feature_num:" + to_string(feature_num), cv::Point2f(10, 10), CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
        image_pool[frame_index] = compressed_image;
    }
    TicToc tmp_t;
    //first query; then add this frame into database!
    QueryResults ret;
    TicToc t_query;
    db.query(keyframe->brief_descriptors, ret, 4, frame_index - 50);
    //printf("query time: %f", t_query.toc());
    //cout << "Searching for Image " << frame_index << ". " << ret << endl;

//    TicToc t_add;
//    db.add(keyframe->brief_descriptors);
    //printf("add feature time: %f", t_add.toc());
    // ret[0] is the nearest neighbour's score. threshold change with neighour score
    bool find_loop = false;
    cv::Mat loop_result;
    if (DEBUG_IMAGE)
    {
        loop_result = compressed_image.clone();
        if (ret.size() > 0)
            putText(loop_result, "neighbour score:" + to_string(ret[0].Score), cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
    }
    // visual loop result
    if (DEBUG_IMAGE)
    {
        for (unsigned int i = 0; i < ret.size(); i++)
        {
            int tmp_index = ret[i].Id;
            auto it = image_pool.find(tmp_index);
            cv::Mat tmp_image = (it->second).clone();
            putText(tmp_image, "index:  " + to_string(tmp_index) + "loop score:" + to_string(ret[i].Score), cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
            cv::vconcat(loop_result, tmp_image, loop_result);
        }
    }
    // a good match with its nerghbour
    if (ret.size() >= 1 &&ret[0].Score > 0.05)
        for (unsigned int i = 1; i < ret.size(); i++)
        {
            //if (ret[i].Score > ret[0].Score * 0.3)
            if (ret[i].Score > 0.015)
            {
                find_loop = true;
                int tmp_index = ret[i].Id;
                if (DEBUG_IMAGE && 0)
                {
                    auto it = image_pool.find(tmp_index);
                    cv::Mat tmp_image = (it->second).clone();
                    putText(tmp_image, "loop score:" + to_string(ret[i].Score), cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
                    cv::vconcat(loop_result, tmp_image, loop_result);
                }
            }

        }

    if (DEBUG_IMAGE)
    {
        cv::imshow("loop_result", loop_result);
        cv::waitKey(0);
    }

    if (find_loop && frame_index > 50)
    {
        int min_index = -1;
        for (unsigned int i = 0; i < ret.size(); i++)
        {
            if (min_index == -1 || (ret[i].Id < min_index && ret[i].Score > 0.015))
                min_index = ret[i].Id;
        }
        return min_index;
    }
    else
        return -1;

}


