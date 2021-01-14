#include <stdio.h>
#include <iostream>
#include<thread>
#include <lcm/lcm-cpp.hpp>
#include "exlcm/HEADER.hpp"
#include "exlcm/Image.hpp"
#include "exlcm/Imu.hpp"

#include "modules/Common/include/easylogging++.h"
#include "lcm/lcm-cpp.hpp"
#include "modules/message/lcm/localization/exlcm/HEADER.hpp"
#include "modules/message/lcm/localization/exlcm/Imu.hpp"
#include "modules/message/lcm/localization/exlcm/Image.hpp"
//#include "modules/BasicIpc/include/channelopt.h"
//#include <opencv2/core.hpp>
//#include <opencv2/opencv.hpp>
#include "../../../vins_estimator/src/vslam_interface.h"
#include <memory>

//#include "exlcm/example_t.hpp"
using namespace std;
class Handler
{
    public:
        ~Handler() {}
        void handleMessageImu(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan,
                const exlcm::Imu* msg)
        {
            int64_t time = msg->header.nTimeStamp;
			double t = static_cast<double>(time * 1.0/1e9);
			double wx = msg->wx;
			double wy = msg->wy;
			double wz = msg->wz;
			double ax = msg->ax;
			double ay = msg->ay;
			double az = msg->az;
			Eigen::Vector3d acc(ax, ay, az);
			Eigen::Vector3d angular(wx, wy, wz);
			inputIMU(t, acc, angular);

        }

        void handleMessageImage(const lcm::ReceiveBuffer* rbuf,
                        const std::string& chan,
                        const exlcm::Image* msg)
		{
        	int64_t time = msg->header.nTimeStamp;
			double t = static_cast<double>(time * 1.0/1e9);
			cv::Mat img(msg->nHeight, msg->nWidth, CV_8UC1);
			for(int i=0; i<msg->nHeight; ++i){
				uint8_t* data = img.ptr<uint8_t>(i);
				for(int j=0; j<msg->nWidth; ++j){
					data[j] = msg->gbImageData[i][j];
				}
			}
			inputImage(t, img);
		}
};
void wait_completion(){
	while(1)
	{
		char c = getchar();
		if (c == 's')
		{
			create_map();
			break;
		}
		if (c == 'q'){
			break;
		}

		std::chrono::milliseconds dura(100);
		std::this_thread::sleep_for(dura);
	}
}
INITIALIZE_EASYLOGGINGPP  // easy logging 初始化

int main(int argc, char** argv)
{
	if(argc != 4)
	{
		LOG(ERROR) << "please intput: sim_estimator_lcm [config file] [log_file] [support file folder]\n"
				"for example: rsim_estimator_lcm ./kitti_config00-02.yaml";
		return 1;
	}

	std::string config_file = argv[1];
	LOG(INFO) << "config_file: " << argv[1];

	std::string log_file = argv[2];
	LOG(INFO) << "log_file: " << argv[2];

	std::string support_files = argv[3];
	LOG(INFO) << "support_files: " << argv[3];

	init_estimator(config_file);
//	set_multiple_thread(1);
	init_loop_fusion(support_files);

	std::shared_ptr<lcm::LCM> lcm_ptr;
	if(log_file == "live"){
		lcm_ptr = std::make_shared<lcm::LCM>();
	}else{
		stringstream ss;
		ss<<"file://";
		ss<<log_file;
		ss<<"?mode=r&speed=1";
		lcm_ptr = std::make_shared<lcm::LCM>(ss.str().c_str());
	}
    if(!lcm_ptr->good())
    {
    	LOG(ERROR) << "lcm initialization error ";
    	return 1;
    }

    Handler handlerObject;
    lcm_ptr->subscribe("imu", &Handler::handleMessageImu, &handlerObject);
    lcm_ptr->subscribe("cam0", &Handler::handleMessageImage, &handlerObject);
    while(0 == lcm_ptr->handle());
    wait_completion();
    return 0;
}
