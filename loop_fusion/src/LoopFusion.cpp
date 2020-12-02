/*
 * LoopFusion.cpp
 *
 *  Created on: Nov 13, 2020
 *      Author: levin
 */

#include "LoopFusion.h"
#include <string>
#include <eigen3/Eigen/Dense>
#include "utility/LoopInfoLogging.h"
#include "camodocal/camera_models/CataCamera.h"
#include "../../vins_estimator/src/estimator/estimator.h"
#include "../../vins_estimator/src/estimator/VOStateSubscriber.h"
#include "keyframe.h"
#include <thread>
#include <mutex>

//std::string VINS_LOOP_RESULT_PATH;
//int ROW;
//int COL;
std::string BRIEF_PATTERN_FILE;
camodocal::CameraPtr m_camera;
Eigen::Vector3d tic;
Eigen::Matrix3d qic;
LoopInfoLogging g_loop_info_logging;
std::string VINS_LOOP_RESULT_PATH;
static std::mutex m_process;
int sequence = 1;
extern std::vector<std::string> CAM_NAMES;
extern int LOAD_PREVIOUS_POSE_GRAPH;
extern std::string OUTPUT_FOLDER;
extern int LOAD_PREVIOUS_POSE_GRAPH;
extern int USE_IMU;


class VOStateSubscriberLoop: public VOStateSubscriber {
public:
	VOStateSubscriberLoop(){
		frame_index_  = 0;
	};
	virtual void update_keyframe(std::shared_ptr<KeyframeInfo> kf_info_ptr){
		const std::lock_guard<std::mutex> lock(buf_mutext_);
		kf_buf_.push(kf_info_ptr);
	};
	virtual void update_img(std::shared_ptr<ImageInfo> image_buf){
		const std::lock_guard<std::mutex> lock(buf_mutext_);
		image_buf_.push(image_buf);

	};
	virtual ~VOStateSubscriberLoop(){};

	 KeyFrame* get_keyframe(){
		 std::shared_ptr<KeyframeInfo> last_kf;
		 std::shared_ptr<ImageInfo> last_img;

		 {
			 const std::lock_guard<std::mutex> lock(buf_mutext_);
			 if(image_buf_.empty() || kf_buf_.empty()){
				 return nullptr;
			 }
			 //get latest kf info and remove all of them from the queue
			 last_kf = kf_buf_.front();
			 while (!kf_buf_.empty()){
				 kf_buf_.pop();
			 }
			 //find image with same timestamp
			 if(last_kf->t_ < image_buf_.front()->t_){
				 return nullptr;
			 }
			 while(last_kf->t_ > image_buf_.front()->t_){
				 image_buf_.pop();
			 }
			 last_img = image_buf_.front();
			 image_buf_.pop();
		 }
		 //create the new keyframe
		 double t = last_img->t_;
		 cv::Mat image = last_img->img_;
		 Vector3d &T = last_kf->P_;
		 Matrix3d R = last_kf->R_.toRotationMatrix();
		 vector<cv::Point3f> point_3d;
		 vector<cv::Point2f> point_2d_uv;
		 vector<cv::Point2f> point_2d_normal;
		 vector<double> point_id;
//		 for (unsigned int i = 0; i < last_kf->pnt_3d2ds_.size(); i++)
		 for(auto &vec : last_kf->pnt_3d2ds_)
		 {
			 cv::Point3f p_3d;
			 p_3d.x = vec(0,0);
			 p_3d.y = vec(1,0);
			 p_3d.z = vec(2,0);
			 point_3d.push_back(p_3d);

			 cv::Point2f p_2d_uv, p_2d_normal;
			 double p_id;
			 p_2d_normal.x = vec(3,0);
			 p_2d_normal.y = vec(4,0);
			 p_2d_uv.x = vec(5,0);
			 p_2d_uv.y = vec(6,0);
			 p_id = vec(7,0);
			 point_2d_normal.push_back(p_2d_normal);
			 point_2d_uv.push_back(p_2d_uv);
			 point_id.push_back(p_id);

			 //printf("u %f, v %f \n", p_2d_uv.x, p_2d_uv.y);
		 }

		 KeyFrame* keyframe = new KeyFrame(t, frame_index_, T, R, image,
		                                    point_3d, point_2d_uv, point_2d_normal, point_id, sequence);
		 frame_index_ ++;
		 return keyframe;
	 }
	 void clear_buf(){
		 const std::lock_guard<std::mutex> lock(buf_mutext_);
		 	while(!kf_buf_.empty())
		 		kf_buf_.pop();
		 	while(!image_buf_.empty())
		 		image_buf_.pop();
	 }
	 virtual void update_odom_extrinsic(std::shared_ptr<OdomExtrinsicInfo> info_ptr){
		 const std::lock_guard<std::mutex> lock(m_process);
		 tic = info_ptr->tic_;
		 qic = info_ptr->ric_;
	 }
private:
	int frame_index_;
	std::mutex buf_mutext_;
	queue<std::shared_ptr<KeyframeInfo>> kf_buf_;
	queue<std::shared_ptr<ImageInfo>> image_buf_;
};


LoopFusion::LoopFusion() {
	// TODO Auto-generated constructor stub
}
void LoopFusion::init(std::string pkg_path, Estimator &est){
	sub_ptr_ =  std::make_shared<VOStateSubscriberLoop>();
	pkg_path_ = pkg_path;
	init_pose_graph();

	//start vo signal receiver
	process_thread_ =  std::thread(&LoopFusion::process, this);
//	cmd_thread_ = std::thread(&LoopFusion::command, this);

	//subscribe to vo sender
	est.vo_state_subs_.register_sub(sub_ptr_);

}
LoopFusion::~LoopFusion() {
	// TODO Auto-generated destructor stub
}

void LoopFusion::init_pose_graph(){
	string vocabulary_file = pkg_path_ + "/../support_files/brief_k10L6.bin";
	cout << "vocabulary_file" << vocabulary_file << endl;
	posegraph_.loadVocabulary(vocabulary_file);

	BRIEF_PATTERN_FILE = pkg_path_ + "/../support_files/brief_pattern.yml";
	cout << "BRIEF_PATTERN_FILE" << BRIEF_PATTERN_FILE << endl;

	std::string cam0Path = CAM_NAMES[0];
	printf("cam calib path: %s\n", cam0Path.c_str());
	m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(cam0Path.c_str());

	VINS_LOOP_RESULT_PATH = OUTPUT_FOLDER + "/vio_loop.csv";
	std::ofstream fout(VINS_LOOP_RESULT_PATH, std::ios::out);
	fout.close();

	posegraph_.setIMUFlag(USE_IMU);

	if (LOAD_PREVIOUS_POSE_GRAPH)
	{
		printf("load pose graph\n");
		posegraph_.loadPoseGraph();
		printf("load pose graph finish\n");
//		load_flag = 1;
	}
	else
	{
		printf("no previous pose graph\n");
//		load_flag = 1;
	}
	g_loop_info_logging.init();


}
void LoopFusion::process(){
	while (true)
	{
		KeyFrame* keyframe = sub_ptr_->get_keyframe();
		if(keyframe){
			const std::lock_guard<std::mutex> lock(m_process);
			posegraph_.addKeyFrame(keyframe, 1);
		}
		std::chrono::milliseconds dura(5);
		std::this_thread::sleep_for(dura);
	}

}
void LoopFusion::create_map(){
	const std::lock_guard<std::mutex> lock(m_process);
	posegraph_.savePoseGraph();
	printf("save pose graph finish\nyou can set 'load_previous_pose_graph' to 1 in the config file to reuse it next time\n");
}
void LoopFusion::command(){
	while(1)
	{
		char c = getchar();
		if (c == 's')
		{
			const std::lock_guard<std::mutex> lock(m_process);
			posegraph_.savePoseGraph();
			printf("save pose graph finish\nyou can set 'load_previous_pose_graph' to 1 in the config file to reuse it next time\n");
//			printf("program shutting down...\n");
//			ros::shutdown();
		}
		if (c == 'n')
			new_sequence();

		std::chrono::milliseconds dura(5);
		std::this_thread::sleep_for(dura);
	}

}
void LoopFusion::new_sequence(){
	printf("new sequence\n");
	sequence++;
	printf("sequence cnt %d \n", sequence);
	if (sequence > 5)
	{
		ROS_WARN("only support 5 sequences since it's boring to copy code for more sequences.");
		return;
	}
	sub_ptr_->clear_buf();
//	posegraph_.posegraph_visualization->reset();
//	posegraph.publish();

}

