#include "vslam_interface.h"
#include "estimator/estimator.h"
#include "utility/visualization.h"
#include "../../loop_fusion/src/LoopFusion.h"
#include "../../loop_fusion/src/CloudPointMap.h"
#include "../../loop_fusion/src/LPStateSubscriber.h"

#include <memory>

using namespace std;
using namespace Eigen;

extern LPStateSubscribers g_lp_state_subscriber;

static std::shared_ptr<Estimator> g_est_ptr;
static std::shared_ptr<LoopFusion> g_lf_ptr;

static std::shared_ptr<CloudPointMap> g_pcm_ptr;


void init_estimator(std::string config_file){
	g_est_ptr = std::make_shared<Estimator>();
	readParameters(config_file);
	g_est_ptr->setParameter();
//	g_est.setParameter();
}
void init_loop_fusion(std::string loop_fution_path){
//	g_loop_fusion.init(loop_fution_path, g_est);
	g_lf_ptr = std::make_shared<LoopFusion>();
	g_lf_ptr->init(loop_fution_path, *g_est_ptr);
}

void inputIMU(double t, const Vector3d &linearAcceleration, const Vector3d &angularVelocity){
	g_est_ptr->inputIMU(t, linearAcceleration, angularVelocity);
//	g_est.inputIMU(t, linearAcceleration, angularVelocity);
}
void inputImage(double t, const cv::Mat &  _img1, const cv::Mat &  _img2){
	auto img1 = _img1.clone();
	auto img2 = _img2.clone();
	g_est_ptr->inputImage(t, img1, img2);
//	g_est.inputImage(t, img1);
}

void create_map(){
	g_lf_ptr->create_map();
}
void vo_callback(std::function<void(VOInfo)> cb){
	g_est_ptr->vo_state_subs_.vo_callback_subs_.push_back(cb);
}
void register_vo_callbacks(std::function<void(OdomExtrinsicInfo &)> odom_extric_f,
		std::function<void(KeyframeInfo &)> key_frame_info_f, std::function<void(ImageInfo &)> img_info_f){
	auto vo_cbs = std::make_shared<VOStateSubscriber_callback>();
	if(odom_extric_f){
		vo_cbs->odom_extric_f_ = odom_extric_f;
	}
	if(key_frame_info_f){
		vo_cbs->key_frame_info_f_ = key_frame_info_f;
	}
	if(img_info_f){
		vo_cbs->img_info_f_ = img_info_f;
	}
	g_est_ptr->vo_state_subs_.register_sub(vo_cbs);
}

void init_reloc(std::string config_file, std::string loop_fution_path){
	g_pcm_ptr = std::make_shared<CloudPointMap>();
	g_pcm_ptr->init(config_file, loop_fution_path);
}
void reloc_image(double _time_stamp, cv::Mat &_image){
	g_pcm_ptr->reloc(_time_stamp, _image);
}

void reloc_callback(std::function<void(LPInfo)> cb){
	g_lp_state_subscriber.lp_state_subs2.push_back(cb);
}

void set_multiple_thread(int flag){
	MULTIPLE_THREAD = flag;
}
