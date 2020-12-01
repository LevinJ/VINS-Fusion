#include "vslam_interface.h"
#include "estimator/estimator.h"
#include "utility/visualization.h"
#include "../../loop_fusion/src/LoopFusion.h"

#include <memory>

using namespace std;
using namespace Eigen;

static std::shared_ptr<Estimator> g_est_ptr;
static std::shared_ptr<LoopFusion> g_lf_ptr;


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
void inputImage(double t, cv::Mat &  img1){
	g_est_ptr->inputImage(t, img1);
//	g_est.inputImage(t, img1);
}
