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

//std::string VINS_LOOP_RESULT_PATH;
//int ROW;
//int COL;
std::string BRIEF_PATTERN_FILE;
camodocal::CameraPtr m_camera;
Eigen::Vector3d tic;
Eigen::Matrix3d qic;
LoopInfoLogging g_loop_info_logging;
std::string VINS_LOOP_RESULT_PATH;
extern std::vector<std::string> CAM_NAMES;
extern int LOAD_PREVIOUS_POSE_GRAPH;
extern std::string OUTPUT_FOLDER;


LoopFusion::LoopFusion(std::string pkg_path) {
	// TODO Auto-generated constructor stub
	pkg_path_ = pkg_path;
	init_params();
}

LoopFusion::~LoopFusion() {
	// TODO Auto-generated destructor stub
}

void LoopFusion::init_params(){
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



}

