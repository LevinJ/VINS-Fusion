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
#include "estimator/estimator.h"
#include "utility/visualization.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

using namespace std;
using namespace Eigen;

Estimator estimator;

Eigen::Matrix3d c1Rc0, c0Rc1;
Eigen::Vector3d c1Tc0, c0Tc1;

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
	ros::init(argc, argv, "vins_estimator");
	ros::NodeHandle n("~");
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

//	ros::Publisher pubLeftImage = n.advertise<sensor_msgs::Image>("/leftImage",1000);
//	ros::Publisher pubRightImage = n.advertise<sensor_msgs::Image>("/rightImage",1000);

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

	readParameters(config_file);
	estimator.setParameter();
	registerPub(n);

	rosbag::Bag bag;
	bag.open(sequence.c_str());  // BagMode is Read by default

	for(rosbag::MessageInstance const m: rosbag::View(bag))
	{
		if(! ros::ok()){
			break;
		}
	  auto img_msg = m.instantiate<sensor_msgs::Image>();
	  if (img_msg != nullptr){
		  auto seq = img_msg->header.seq;
		  auto time = img_msg->header.stamp.toSec();
		  auto im = getImageFromMsg(img_msg);
		  estimator.inputImage(time, im);
	  }

	  auto imu_msg = m.instantiate<sensor_msgs::Imu>();
	  if (imu_msg != nullptr){
		  auto seq = imu_msg->header.seq;
		  double t = imu_msg->header.stamp.toSec();
		  double dx = imu_msg->linear_acceleration.x;
		  double dy = imu_msg->linear_acceleration.y;
		  double dz = imu_msg->linear_acceleration.z;
		  double rx = imu_msg->angular_velocity.x;
		  double ry = imu_msg->angular_velocity.y;
		  double rz = imu_msg->angular_velocity.z;
		  Vector3d acc(dx, dy, dz);
		  Vector3d gyr(rx, ry, rz);
		  estimator.inputIMU(t, acc, gyr);
	  }

	}

	bag.close();

	// load image list
	/*
	FILE* file;
	file = std::fopen((dataPath + "times.txt").c_str() , "r");
	if(file == NULL){
	    printf("cannot find file: %stimes.txt\n", dataPath.c_str());
	    ROS_BREAK();
	    return 0;          
	}
	double imageTime;
	vector<double> imageTimeList;
	while ( fscanf(file, "%lf", &imageTime) != EOF)
	{
	    imageTimeList.push_back(imageTime);
	}
	std::fclose(file);

	string leftImagePath, rightImagePath;
	cv::Mat imLeft, imRight;
	FILE* outFile;
	outFile = fopen((OUTPUT_FOLDER + "/vio.txt").c_str(),"w");
	if(outFile == NULL)
		printf("Output path dosen't exist: %s\n", OUTPUT_FOLDER.c_str());

	for (size_t i = 0; i < imageTimeList.size(); i++)
	{	
		if(ros::ok())
		{
			printf("\nprocess image %d\n", (int)i);
			stringstream ss;
			ss << setfill('0') << setw(6) << i;
			leftImagePath = dataPath + "image_0/" + ss.str() + ".png";
			rightImagePath = dataPath + "image_1/" + ss.str() + ".png";
			//printf("%lu  %f \n", i, imageTimeList[i]);
			//printf("%s\n", leftImagePath.c_str() );
			//printf("%s\n", rightImagePath.c_str() );

			imLeft = cv::imread(leftImagePath, CV_LOAD_IMAGE_GRAYSCALE );
			sensor_msgs::ImagePtr imLeftMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imLeft).toImageMsg();
			imLeftMsg->header.stamp = ros::Time(imageTimeList[i]);
			pubLeftImage.publish(imLeftMsg);

			imRight = cv::imread(rightImagePath, CV_LOAD_IMAGE_GRAYSCALE );
			sensor_msgs::ImagePtr imRightMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", imRight).toImageMsg();
			imRightMsg->header.stamp = ros::Time(imageTimeList[i]);
			pubRightImage.publish(imRightMsg);


			estimator.inputImage(imageTimeList[i], imLeft, imRight);
			
			Eigen::Matrix<double, 4, 4> pose;
			estimator.getPoseInWorldFrame(pose);
			if(outFile != NULL)
				fprintf (outFile, "%f %f %f %f %f %f %f %f %f %f %f %f \n",pose(0,0), pose(0,1), pose(0,2),pose(0,3),
																	       pose(1,0), pose(1,1), pose(1,2),pose(1,3),
																	       pose(2,0), pose(2,1), pose(2,2),pose(2,3));
			
			//cv::imshow("leftImage", imLeft);
			//cv::imshow("rightImage", imRight);
			//cv::waitKey(2);
		}
		else
			break;
	}
	if(outFile != NULL)
		fclose (outFile);
		*/
	return 0;
}
