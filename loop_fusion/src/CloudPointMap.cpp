/*
 * CloudPointMap.cpp
 *
 *  Created on: Apr 16, 2020
 *      Author: levin
 */

#include "CloudPointMap.h"
#include "utility/utility.h"
#include "utility/LoopInfoLogging.h"
#include <thread>
#include <mutex>
#include <memory>
#include "../../vins_estimator/src/estimator/VOStateSubscriber.h"
#include "utility/PoseInfo.h"

using namespace std;

extern int DEBUG_IMAGE;
extern std::string POSE_GRAPH_SAVE_PATH;
extern std::string BRIEF_PATTERN_FILE;
extern camodocal::CameraPtr m_camera;
extern LoopInfoLogging g_loop_info_logging;
extern int ROW;
extern int COL;
extern std::string VINS_LOOP_RESULT_PATH;
extern std::string RAW_DATA_PATH;

extern std::string OUTPUT_FOLDER;
extern Eigen::Vector3d tic;
extern Eigen::Matrix3d qic;

//extern int LOAD_PREVIOUS_POSE_GRAPH;
//extern int USE_IMU;

CloudPointMap::CloudPointMap(): PoseGraph(){
	// TODO Auto-generated constructor stub
	point_cloud_path_ = POSE_GRAPH_SAVE_PATH + "vlsam_pcd.pcd";
}
void CloudPointMap::saveMap(std::list<KeyFrame*> &keyframelist){

	point_map_.clear();
	//save 3d points
	 for (list<KeyFrame*>::iterator it = keyframelist.begin(); it != keyframelist.end(); it++){
		 vector<cv::Point3f> &point_3d = (*it)->point_3d;
		 vector<double> &point_id = (*it)->point_id;
		 for(unsigned int i=0; i<point_id.size(); i++ ){
			 if(point_map_.find(point_id[i]) != point_map_.end()){
				 continue;
			 }
			 point_map_[point_id[i]] = {point_3d[i].x, point_3d[i].y, point_3d[i].z};
		 }
	 }

	 auto kpoint_cloud_file = fopen(point_cloud_path_.c_str(), "w");
	 for (auto it=point_map_.begin(); it!=point_map_.end(); ++it){
		 auto id = it->first;
		 auto x =  it->second[0];
		 auto y =  it->second[1];
		 auto z =  it->second[2];
		 fprintf(kpoint_cloud_file, "%lf %lf %lf %lf\n", id, x,y,z);
	 }
	 fclose(kpoint_cloud_file);
	 std::cerr << "Saved " << point_map_.size () << " data points to " << point_cloud_path_ << std::endl;

	 // write point_id, point_2d_uv, point_2d_norm
	 for (list<KeyFrame*>::iterator it = keyframelist.begin(); it != keyframelist.end(); it++){
		 auto keypoints_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_keypoints_selected.txt";
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
	point_map_.clear();
	FILE * pFile = fopen (point_cloud_path_.c_str(),"r");
	if (pFile == NULL)
	{
		printf("load point cloud error: wrong previous pose graph path or no previous pose graph \n");
		return;
	}
	double id;
	double x;
	double y;
	double z;
	while (fscanf(pFile,"%lf %lf %lf %lf", &id, &x, &y, &z) != EOF){
		point_map_[id] = {x, y, z};
	}
	fclose (pFile);
}
#ifndef WITH_ROS_SIMULATE
void CloudPointMap::publish_cloudponint(ros::Publisher &_pub_base_point_cloud){
	sensor_msgs::PointCloud point_cloud;
	point_cloud.header.frame_id = "world";
	point_cloud.header.stamp = ros::Time::now();
	for (auto it=point_map_.begin(); it!=point_map_.end(); ++it){
		geometry_msgs::Point32 pnt;
		pnt.x = it->second[0];
		pnt.y = it->second[1];
		pnt.z = it->second[2];
		point_cloud.points.push_back(pnt);
	}
	_pub_base_point_cloud.publish(point_cloud);
}
#endif
CloudPointMap::~CloudPointMap() {
	// TODO Auto-generated destructor stub
}
void CloudPointMap::init(std::string config_file, std::string pkg_path){
	cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
	if(!fsSettings.isOpened())
	{
		std::cerr << "ERROR: Wrong path to settings" << std::endl;
	}
	ROW = fsSettings["image_height"];
	COL = fsSettings["image_width"];

	string vocabulary_file = pkg_path + "/../support_files/brief_k10L6.bin";
	cout << "vocabulary_file" << vocabulary_file << endl;
	this->loadVocabulary(vocabulary_file);

	BRIEF_PATTERN_FILE = pkg_path + "/../support_files/brief_pattern.yml";
	cout << "BRIEF_PATTERN_FILE" << BRIEF_PATTERN_FILE << endl;

	int pn = config_file.find_last_of('/');
	std::string configPath = config_file.substr(0, pn);
	std::string cam0Calib;
	fsSettings["cam0_calib"] >> cam0Calib;
	std::string cam0Path = configPath + "/" + cam0Calib;
	printf("cam calib path: %s\n", cam0Path.c_str());
	m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(cam0Path.c_str());

//	fsSettings["image0_topic"] >> IMAGE_TOPIC;
	fsSettings["pose_graph_save_path"] >> POSE_GRAPH_SAVE_PATH;
	fsSettings["output_path"] >> VINS_LOOP_RESULT_PATH;
	fsSettings["save_image"] >> DEBUG_IMAGE;
	fsSettings["raw_data_path"] >> RAW_DATA_PATH;

	cv::Mat cv_T;
	fsSettings["body_T_cam0"] >> cv_T;
	Eigen::Matrix4d T;
	cv::cv2eigen(cv_T, T);
	qic = T.block<3, 3>(0, 0);
	tic = T.block<3, 1>(0, 3);

//	LOAD_PREVIOUS_POSE_GRAPH = fsSettings["load_previous_pose_graph"];
//	FUSE_GNSS = fsSettings["fuse_gnss"];
	VINS_LOOP_RESULT_PATH = VINS_LOOP_RESULT_PATH + "/vio_reloc.csv";
	std::ofstream fout(VINS_LOOP_RESULT_PATH, std::ios::out);
	fout.close();

	point_cloud_path_ = POSE_GRAPH_SAVE_PATH + "vlsam_pcd.pcd";

	printf("load pose graph\n");
	this->loadPoseGraph();
	printf("load pose graph finish\n");
	g_loop_info_logging.init();
	process_thread_ =  std::thread(&CloudPointMap::process, this);

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

        vector<cv::Point3f> point_3d;
		vector<cv::Point2f> point_2d_uv;
		vector<cv::Point2f> point_2d_normal;
		vector<double> point_id;

		auto keypoints_path = POSE_GRAPH_SAVE_PATH + to_string(index) + "_keypoints_selected.txt";
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
			pnt = point_map_[id];
			point_3d.emplace_back(pnt[0], pnt[1],pnt[2]);

		}
		int sequence = 0;
		KeyFrame* keyframe = new KeyFrame(time_stamp, index, PG_T, PG_R, image,
									   point_3d, point_2d_uv, point_2d_normal, point_id, sequence);
		loadKeyFrame(keyframe, 0);
        cnt++;
    }
    fclose (pFile);
    printf("load pose graph time: %f s\n", tmp_t.toc()/1000);
    base_sequence = 0;
}

void CloudPointMap::process(){
	while (true)
	{
		while (image_buf_.empty()){
			std::chrono::milliseconds dura(5);
			std::this_thread::sleep_for(dura);
		}
		std::shared_ptr<ImageInfo> last_img;
		{
			const std::lock_guard<std::mutex> lock(buf_mutext_);
			last_img = image_buf_.front();
			image_buf_.pop();
		}
		process_img(last_img);
	}

}
void CloudPointMap::process_img(std::shared_ptr<ImageInfo> last_img){
	KeyFrame* cur_kf = new KeyFrame(last_img->t_, global_index, last_img->img_, 1);
	global_index++;
	TicToc tmp_t;
	auto loop_index = detectLoop(cur_kf, cur_kf->index);
	//	cout<<"detectloop="<<tmp_t.toc()<<endl;
	if (loop_index == -1){
		return;
	}

	//find connections
	KeyFrame* old_kf = getKeyFrame(loop_index);
	cout << "loop detected, " <<cur_kf->sequence<<", "<< cur_kf->index<< "-->"<< old_kf->sequence<<", "<<old_kf->index<<endl;
	tmp_t.tic();
	bool bfindconn = old_kf->findConnection(cur_kf);
	//	cout<<"findConnection="<<tmp_t.toc()<<endl;
	if (!bfindconn){
		return;
	}
	Vector3d relative_t;
	Quaterniond relative_q;
	relative_t = old_kf->getLoopRelativeT();
	relative_q = (old_kf->getLoopRelativeQ());

	auto TCO = PoseInfo().construct_fromqt(relative_q, relative_t);
	auto TOC = TCO.I();
	auto TWO = PoseInfo().construct_fromRt(old_kf->R_w_i, old_kf->T_w_i);
	auto TWC = TWO * TCO.I();

	Quaterniond Q = TWC.q_;
	Vector3d P = TWC.t_;


	//TUM format
	ofstream loop_path_file(VINS_LOOP_RESULT_PATH, ios::app);
	loop_path_file.setf(ios::fixed, ios::floatfield);
	loop_path_file.precision(6);
	loop_path_file << cur_kf->time_stamp << " ";
	loop_path_file.precision(7);
	loop_path_file  << P.x() << " "
			<< P.y() << " "
			<< P.z() << " "
			<< Q.x() << " "
			<< Q.y() << " "
			<< Q.z() << " "
			<< Q.w()<< endl;

	loop_path_file.close();
}
void CloudPointMap::reloc(double t, cv::Mat &img){
	const std::lock_guard<std::mutex> lock(buf_mutext_);
	auto im_info_ptr = std::make_shared<ImageInfo>();
	im_info_ptr->t_ = t;
	im_info_ptr->img_ = img;
	image_buf_.push(im_info_ptr);
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
	    db.query(keyframe->brief_descriptors, ret, 4, frame_index);
	    //printf("query time: %f", t_query.toc());
	    //cout << "Searching for Image " << frame_index << ". " << ret << endl;

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
	            cv::hconcat(loop_result, tmp_image, loop_result);
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
	                    cv::hconcat(loop_result, tmp_image, loop_result);
	                }
	            }

	        }

	    if (DEBUG_IMAGE)
	    {
	        cv::imshow("loop_result", loop_result);
	        cv::waitKey(20);
	    }

	    if (find_loop)
	    {
	        int min_index = -1;
	        for (unsigned int i = 0; i < ret.size(); i++)
	        {
	            if (min_index == -1 || (int(ret[i].Id) < min_index && ret[i].Score > 0.015))
	                min_index = ret[i].Id;
	        }
	        return min_index;
	    }
	    else
	        return -1;

}


