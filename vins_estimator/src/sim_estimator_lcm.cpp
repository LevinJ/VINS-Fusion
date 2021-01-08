#include <thread>
#include <chrono>
#include <memory>
#include "interface/handler.h"

INITIALIZE_EASYLOGGINGPP  // easy logging 初始化

int main(int argc, char** argv){
    el::Configurations conf("./data/log/log_vslam.conf");
    el::Loggers::reconfigureAllLoggers(conf);

    if(argc != 2)
	{
		LOG(ERROR) << "please intput: sim_estimator_lcm [config file] \n"
				"for example: rsim_estimator_lcm ./kitti_config00-02.yaml";
		return 1;
	}

	std::string config_file = argv[1];
	LOG(INFO) << "config_file: " << argv[1];
	init_estimator(config_file);

	lcm::LCM lm;
	if(!lm.good()){
		LOG(ERROR) << "LCM error!!!";
		return 1;
	}
	Handler handlerObject;
	lm.subscribe("imu", &Handler::handleMessageImu, &handlerObject);
	lm.subscribe("cam0", &Handler::handleMessageImage, &handlerObject);

	while(0 == lm.handle());
	return 0;
}