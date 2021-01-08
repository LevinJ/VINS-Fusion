#include "handler.h"

Handler::Handler(/* args */)
{
}

Handler::~Handler()
{
}

void Handler::handleMessageImu(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const vslam::Imu* msg){
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
		//LOG(INFO) << std::fixed << std::setprecision(6) << "input imu:t " << t << " " << acc.transpose() << " " << angular.transpose();

	}

void Handler::handleMessageImage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const vslam::Image* msg){
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
		//LOG(INFO) << std::fixed << std::setprecision(6) << "input image, t " << t << " " << img.rows << " " << img.cols;
	}
