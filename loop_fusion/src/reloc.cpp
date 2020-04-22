/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "CloudPointMap.h"

#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <ros/package.h>
#include <mutex>
#include <queue>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "keyframe.h"
#include "utility/tic_toc.h"
#include "pose_graph.h"
#include "utility/CameraPoseVisualization.h"
#include "parameters.h"
#define SKIP_FIRST_CNT 10
using namespace std;

queue<sensor_msgs::ImageConstPtr> image_buf;
queue<sensor_msgs::PointCloudConstPtr> point_buf;
queue<nav_msgs::Odometry::ConstPtr> pose_buf;
queue<Eigen::Vector3d> odometry_buf;
std::mutex m_buf;
std::mutex m_process;
int frame_index  = 0;
int sequence = 1;

int skip_first_cnt = 0;
int SKIP_CNT;
int skip_cnt = 0;
//bool load_flag = 0;
bool start_flag = 0;
double SKIP_DIS = 0;

int VISUALIZATION_SHIFT_X;
int VISUALIZATION_SHIFT_Y;
int ROW;
int COL;
int DEBUG_IMAGE;

camodocal::CameraPtr m_camera;
Eigen::Vector3d tic;
Eigen::Matrix3d qic;
ros::Publisher pub_match_img;
ros::Publisher pub_camera_pose_visual;
ros::Publisher pub_odometry_rect;

std::string BRIEF_PATTERN_FILE;
std::string POSE_GRAPH_SAVE_PATH;
std::string VINS_RESULT_PATH;
CameraPoseVisualization cameraposevisual(1, 0, 0, 1);
Eigen::Vector3d last_t(-100, -100, -100);
double last_image_time = -1;

ros::Publisher pub_point_cloud, pub_margin_cloud, g_pub_base_point_cloud;



//#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
//#include "processPointClouds.cpp"
//
//#include "ProcessPointCloudsCourse.h"
//#include "ProcessPointCloudsCourse.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TODO:: Create lidar sensor
    Lidar* lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    renderRays(viewer, lidar->position, inputCloud);
    renderPointCloud(viewer, inputCloud, "pc1", Color(0,1,0));
    // TODO:: Create point processor

//    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
//    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);
//    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(0,1,1));
//    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(1,1,0));
//
//    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);
//
//    int clusterId = 0;
//    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
//
//    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
//    {
////          std::cout << "cluster size ";
//          pointProcessor.numPoints(cluster);
//          renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
//          Box box = pointProcessor.BoundingBox(cluster);
//          renderBox(viewer,box,clusterId);
//          ++clusterId;
//    }



}

//void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessor, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
////void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
//{
//  // ----------------------------------------------------
//  // -----Open 3D viewer and display City Block     -----
//  // ----------------------------------------------------
//
////  renderPointCloud(viewer,inputCloud,"inputCloud");
//  Box box;
//  box.x_min = -10;
//  box.y_min = -6;
//  box.z_min = -2;
//  box.x_max = 30;
//  box.y_max = 7;
//  box.z_max = 3;
////  renderBox(viewer, box, 1, Color(0,1,0), 1);
//  auto filterCloud = pointProcessor->FilterCloud(inputCloud, 0.2 , Eigen::Vector4f (box.x_min, box.y_min,  box.z_min, 1),
//		  Eigen::Vector4f ( box.x_max, box.y_max , box.z_max, 1));
//  renderPointCloud(viewer,filterCloud,"filterCloud");
//
//
//  //the box for filtering the roof area
//  box.x_min = -1.5;
//  box.y_min = -1.7;
//  box.z_min = -1;
//  box.x_max = 2.6;
//  box.y_max = 1.7;
//  box.z_max = -0.4;
////  renderBox(viewer, box, 100, Color(0,1,0), 1);
//
//  //segment the cloud
//  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor->SegmentPlane(filterCloud, 100, 0.2);
//  renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
//  renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
//
//
//  //cluster the cloud
//  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 0.5, 10, 1600);
//
//  int clusterId = 0;
//  std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
//
//  for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
//  {
//		renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
//		Box box = pointProcessor->BoundingBox(cluster);
//		renderBox(viewer,box,clusterId);
//		++clusterId;
//  }
//
//}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

int cpm_init(std::string pkg_path, string config_file, CloudPointMap &posegraph)
{
//    ros::init(argc, argv, "loop_fusion");
//    ros::NodeHandle n("~");
//    posegraph.registerPub(n);

    VISUALIZATION_SHIFT_X = 0;
    VISUALIZATION_SHIFT_Y = 0;
    SKIP_CNT = 0;
    SKIP_DIS = 0;


    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    cameraposevisual.setScale(0.1);
    cameraposevisual.setLineWidth(0.01);

    std::string IMAGE_TOPIC;
    int LOAD_PREVIOUS_POSE_GRAPH;

    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];

    string vocabulary_file = pkg_path + "/../support_files/brief_k10L6.bin";
    cout << "vocabulary_file" << vocabulary_file << endl;
    posegraph.loadVocabulary(vocabulary_file);

    BRIEF_PATTERN_FILE = pkg_path + "/../support_files/brief_pattern.yml";
    cout << "BRIEF_PATTERN_FILE" << BRIEF_PATTERN_FILE << endl;

    int pn = config_file.find_last_of('/');
    std::string configPath = config_file.substr(0, pn);
    std::string cam0Calib;
    fsSettings["cam0_calib"] >> cam0Calib;
    std::string cam0Path = configPath + "/" + cam0Calib;
    printf("cam calib path: %s\n", cam0Path.c_str());
    m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(cam0Path.c_str());

    fsSettings["image0_topic"] >> IMAGE_TOPIC;
    fsSettings["pose_graph_save_path"] >> POSE_GRAPH_SAVE_PATH;
    fsSettings["output_path"] >> VINS_RESULT_PATH;
    fsSettings["save_image"] >> DEBUG_IMAGE;

    LOAD_PREVIOUS_POSE_GRAPH = 1;
    VINS_RESULT_PATH = VINS_RESULT_PATH + "/vio_loop.csv";
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

//    int USE_IMU = fsSettings["imu"];
//    posegraph.setIMUFlag(USE_IMU);
    fsSettings.release();

    posegraph.loadPoseGraph();

//    if (LOAD_PREVIOUS_POSE_GRAPH)
//    {
//        printf("load pose graph\n");
//        m_process.lock();
//        posegraph.loadPoseGraph();
//        m_process.unlock();
//        printf("load pose graph finish\n");
//        load_flag = 1;
//    }
//    else
//    {
//        printf("no previous pose graph\n");
//        load_flag = 1;
//    }

//    ros::Subscriber sub_vio = n.subscribe("/vins_estimator/odometry", 2000, vio_callback);
//    ros::Subscriber sub_image = n.subscribe(IMAGE_TOPIC, 2000, image_callback);
//    ros::Subscriber sub_pose = n.subscribe("/vins_estimator/keyframe_pose", 2000, pose_callback);
//    ros::Subscriber sub_extrinsic = n.subscribe("/vins_estimator/extrinsic", 2000, extrinsic_callback);
//    ros::Subscriber sub_point = n.subscribe("/vins_estimator/keyframe_point", 2000, point_callback);
//    ros::Subscriber sub_margin_point = n.subscribe("/vins_estimator/margin_cloud", 2000, margin_point_callback);
//
//    pub_match_img = n.advertise<sensor_msgs::Image>("match_image", 1000);
//    pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 1000);
//    pub_point_cloud = n.advertise<sensor_msgs::PointCloud>("point_cloud_loop_rect", 1000);
//    pub_margin_cloud = n.advertise<sensor_msgs::PointCloud>("margin_cloud_loop_rect", 1000);
//    pub_odometry_rect = n.advertise<nav_msgs::Odometry>("odometry_rect", 1000);

//    std::thread measurement_process;
//    std::thread keyboard_command_process;
//
//    measurement_process = std::thread(process);
//    keyboard_command_process = std::thread(command);
//
//    ros::spin();

    return 0;
}

KeyFrame*  gen_keyframe(){
	auto image = cv::imread("/home/levin/output/pose_graph/0_image.png", cv::IMREAD_GRAYSCALE);
	Vector3d T = Vector3d(0,
						  0,
						  0);
	Matrix3d R = Quaterniond(1,
							 0,
							 0,
							 0).toRotationMatrix();
	double timestamp = 1576825210.502939;
	vector<cv::Point3f> point_3d;
	vector<cv::Point2f> point_2d_uv;
	vector<cv::Point2f> point_2d_normal;
	vector<double> point_id;
	KeyFrame* keyframe = new KeyFrame(timestamp, frame_index, T, R, image,
	                                   point_3d, point_2d_uv, point_2d_normal, point_id, sequence);
	return keyframe;
}
int main (int argc, char** argv)
{

	tic = {-0.02,2.15, 0.915};

	qic <<1, 0, 0,
		0, 0, 1,
		0, -1, 0;
    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);
    std::string pkg_path = "/home/levin/workspace/ros_projects/src/VINS-Fusion/loop_fusion";
    CloudPointMap cpm;

    cpm_init(pkg_path, config_file, cpm);


//    simpleHighway(viewer);
    KeyFrame* keyframe = gen_keyframe();
    cpm.reloc_frame(keyframe);


    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	CameraAngle setAngle = TopDown;
	initCamera(setAngle, viewer);
	renderPointCloud(viewer, cpm.mcloudxyz, "pc1", Color(0,1,0));



//    viewer.

//    cityBlock(viewer);
//    ProcessPointClouds<pcl::PointXYZI> *pointProcessor = new ProcessPointClouds<pcl::PointXYZI>();

    //ProcessPointCloudsCourse uses the RANSAC, KDTreea and Eucleadian clustering implemented by myself
//    ProcessPointClouds<pcl::PointXYZI> *pointProcessor = new ProcessPointCloudsCourse<pcl::PointXYZI>();
//
//    std::vector<boost::filesystem::path> stream = pointProcessor->streamPcd("../src/sensors/data/pcd/data_1");
//    auto streamIterator = stream.begin();
//    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
//
//    while (!viewer->wasStopped ())
//    {
//    	// Clear viewer
//		viewer->removeAllPointClouds();
//		viewer->removeAllShapes();
//
//		// Load pcd and run obstacle detection process
//		inputCloudI = pointProcessor->loadPcd((*streamIterator).string());
//		cityBlock(viewer, pointProcessor, inputCloudI);
//
//		streamIterator++;
//		if(streamIterator == stream.end())
//		{
//			 std::cout << "END OF PCD STREAM" << std::endl;
//			streamIterator = stream.begin();
//		}
//
//		break;
//
//        viewer->spinOnce ();
//    }

    while (!viewer->wasStopped ()){
    	viewer->spinOnce ();
    }
}
