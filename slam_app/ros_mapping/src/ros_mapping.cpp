/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include "../../../vins_estimator/src/vslam_interface.h"

using namespace std;
using namespace Eigen;


cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
	cv_bridge::CvImageConstPtr ptr;
	if (img_msg->encoding == "8UC1")
	{
		sensor_msgs::Image img;
		img.header = img_msg->header;
		img.height = img_msg->height;
		img.width = img_msg->width;
		img.is_bigendian = img_msg->is_bigendian;
		img.step = img_msg->step;
		img.data = img_msg->data;
		img.encoding = "mono8";
		ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
	}
	else
		ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

	cv::Mat img = ptr->image.clone();
	return img;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "ros_mapping");
	ros::NodeHandle n("~");
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

	if(argc != 3)
	{
		printf("please intput: rosrun vins rosbagtest [config file] [data folder] \n"
				"for example: rosrun vins kitti_odom_test "
				"~/catkin_ws/src/VINS-Fusion/config/kitti_odom/kitti_config00-02.yaml "
				"/media/tony-ws1/disk_D/kitti/odometry/sequences/00/ \n");
		return 1;
	}

	string config_file = argv[1];
	printf("config_file: %s\n", argv[1]);
	string sequence = argv[2];
	printf("read sequence: %s\n", argv[2]);
	string dataPath = sequence + "/";

	init_estimator(config_file);
	init_loop_fusion("/media/levin/DATADRIVE1/aiways_tools/localization_master/CtApVSlam/support_files");


	rosbag::Bag bag;
	bag.open(sequence.c_str());  // BagMode is Read by default


	for(rosbag::MessageInstance const m: rosbag::View(bag))
	{
		std::string topic = m.getTopic();

		if(topic != "/cam0/image_raw" && topic != "/imu0"){
			continue;
		}

		auto img_msg = m.instantiate<sensor_msgs::Image>();
		if (img_msg != nullptr){

			auto seq = img_msg->header.seq;
			// std::cout<<"image seq ="<<seq<<std::endl;
			auto time = img_msg->header.stamp.toSec();
			auto im = getImageFromMsg(img_msg);
			inputImage(time, im);
		}

		auto imu_msg = m.instantiate<sensor_msgs::Imu>();
		if (imu_msg != nullptr){
			double t = imu_msg->header.stamp.toSec();
			double dx = imu_msg->linear_acceleration.x;
			double dy = imu_msg->linear_acceleration.y;
			double dz = imu_msg->linear_acceleration.z;
			double rx = imu_msg->angular_velocity.x;
			double ry = imu_msg->angular_velocity.y;
			double rz = imu_msg->angular_velocity.z;
			Vector3d acc(dx, dy, dz);
			Vector3d gyr(rx, ry, rz);
			inputIMU(t, acc, gyr);
		}
	}

	bag.close();
	std::cout<<"bag play done, optimization may not be done yet"<<std::endl;

	ros::spin();
	return 0;
}
