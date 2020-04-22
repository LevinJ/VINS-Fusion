/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "CameraPoseVisualization.h"

const Eigen::Vector3d CameraPoseVisualization::imlt = Eigen::Vector3d(-1.0, -0.5, 1.0);
const Eigen::Vector3d CameraPoseVisualization::imrt = Eigen::Vector3d( 1.0, -0.5, 1.0);
const Eigen::Vector3d CameraPoseVisualization::imlb = Eigen::Vector3d(-1.0,  0.5, 1.0);
const Eigen::Vector3d CameraPoseVisualization::imrb = Eigen::Vector3d( 1.0,  0.5, 1.0);
const Eigen::Vector3d CameraPoseVisualization::lt0 = Eigen::Vector3d(-0.7, -0.5, 1.0);
const Eigen::Vector3d CameraPoseVisualization::lt1 = Eigen::Vector3d(-0.7, -0.2, 1.0);
const Eigen::Vector3d CameraPoseVisualization::lt2 = Eigen::Vector3d(-1.0, -0.2, 1.0);
const Eigen::Vector3d CameraPoseVisualization::oc = Eigen::Vector3d(0.0, 0.0, 0.0);

void Eigen2Point(const Eigen::Vector3d& v, geometry_msgs::Point& p) {
    p.x = v.x();
    p.y = v.y();
    p.z = v.z();
}

CameraPoseVisualization::CameraPoseVisualization(float r, float g, float b, float a)
    : m_marker_ns("CameraPoseVisualization"), m_scale(0.2), m_line_width(0.01) {
    m_image_boundary_color.r = r;
    m_image_boundary_color.g = g;
    m_image_boundary_color.b = b;
    m_image_boundary_color.a = a;
    m_optical_center_connector_color.r = r;
    m_optical_center_connector_color.g = g;
    m_optical_center_connector_color.b = b;
    m_optical_center_connector_color.a = a;
    LOOP_EDGE_NUM = 20;
    tmp_loop_edge_num = 1;
}

void CameraPoseVisualization::setImageBoundaryColor(float r, float g, float b, float a) {
    m_image_boundary_color.r = r;
    m_image_boundary_color.g = g;
    m_image_boundary_color.b = b;
    m_image_boundary_color.a = a;
}

void CameraPoseVisualization::setOpticalCenterConnectorColor(float r, float g, float b, float a) {
    m_optical_center_connector_color.r = r;
    m_optical_center_connector_color.g = g;
    m_optical_center_connector_color.b = b;
    m_optical_center_connector_color.a = a;
}

void CameraPoseVisualization::setScale(double s) {
    m_scale = s;
}
void CameraPoseVisualization::setLineWidth(double width) {
    m_line_width = width;
}
void CameraPoseVisualization::add_edge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1){
    visualization_msgs::Marker marker;

    marker.ns = m_marker_ns;
    marker.id = m_markers.size() + 1;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.01;

    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    geometry_msgs::Point point0, point1;

    Eigen2Point(p0, point0);
    Eigen2Point(p1, point1);

    marker.points.push_back(point0);
    marker.points.push_back(point1);

    m_markers.push_back(marker);
}

void CameraPoseVisualization::add_loopedge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1){
    //m_markers.clear();
    visualization_msgs::Marker marker;

    marker.ns = m_marker_ns;
    marker.id = m_markers.size() + 1;
    //tmp_loop_edge_num++;
    //if(tmp_loop_edge_num >= LOOP_EDGE_NUM)
    //  tmp_loop_edge_num = 1;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();
    //marker.scale.x = 0.4;
    marker.scale.x = 0.02;
    marker.color.r = 1.0f;
    //marker.color.g = 1.0f;
    //marker.color.b = 1.0f;
    marker.color.a = 1.0;

    geometry_msgs::Point point0, point1;

    Eigen2Point(p0, point0);
    Eigen2Point(p1, point1);

    marker.points.push_back(point0);
    marker.points.push_back(point1);

    m_markers.push_back(marker);
}


void CameraPoseVisualization::add_pose(const Eigen::Vector3d& p, const Eigen::Quaterniond& q) {
    visualization_msgs::Marker marker;

    marker.ns = m_marker_ns;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = m_line_width;

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;


    geometry_msgs::Point pt_lt, pt_lb, pt_rt, pt_rb, pt_oc, pt_lt0, pt_lt1, pt_lt2;

    Eigen2Point(q * (m_scale *imlt) + p, pt_lt);
    Eigen2Point(q * (m_scale *imlb) + p, pt_lb);
    Eigen2Point(q * (m_scale *imrt) + p, pt_rt);
    Eigen2Point(q * (m_scale *imrb) + p, pt_rb);
    Eigen2Point(q * (m_scale *lt0 ) + p, pt_lt0);
    Eigen2Point(q * (m_scale *lt1 ) + p, pt_lt1);
    Eigen2Point(q * (m_scale *lt2 ) + p, pt_lt2);
    Eigen2Point(q * (m_scale *oc  ) + p, pt_oc);

    // image boundaries
    marker.points.push_back(pt_lt);
    marker.points.push_back(pt_lb);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);

    marker.points.push_back(pt_lb);
    marker.points.push_back(pt_rb);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);

    marker.points.push_back(pt_rb);
    marker.points.push_back(pt_rt);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);

    marker.points.push_back(pt_rt);
    marker.points.push_back(pt_lt);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);

    // top-left indicator
    marker.points.push_back(pt_lt0);
    marker.points.push_back(pt_lt1);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);

    marker.points.push_back(pt_lt1);
    marker.points.push_back(pt_lt2);
    marker.colors.push_back(m_image_boundary_color);
    marker.colors.push_back(m_image_boundary_color);

    // optical center connector
    marker.points.push_back(pt_lt);
    marker.points.push_back(pt_oc);
    marker.colors.push_back(m_optical_center_connector_color);
    marker.colors.push_back(m_optical_center_connector_color);


    marker.points.push_back(pt_lb);
    marker.points.push_back(pt_oc);
    marker.colors.push_back(m_optical_center_connector_color);
    marker.colors.push_back(m_optical_center_connector_color);

    marker.points.push_back(pt_rt);
    marker.points.push_back(pt_oc);
    marker.colors.push_back(m_optical_center_connector_color);
    marker.colors.push_back(m_optical_center_connector_color);

    marker.points.push_back(pt_rb);
    marker.points.push_back(pt_oc);
    marker.colors.push_back(m_optical_center_connector_color);
    marker.colors.push_back(m_optical_center_connector_color);

    m_markers.push_back(marker);
}

void CameraPoseVisualization::reset() {
	m_markers.clear();
    //image.points.clear();
    //image.colors.clear();
}

void CameraPoseVisualization::publish_by( ros::Publisher &pub, const std_msgs::Header &header ) {
	visualization_msgs::MarkerArray markerArray_msg;
	//int k = (int)m_markers.size();
  /*
  for (int i = 0; i < 5 && k > 0; i++)
  {
    k--;
    m_markers[k].header = header;
    markerArray_msg.markers.push_back(m_markers[k]);
  }
  */

  
	for(auto& marker : m_markers) {
		marker.header = header;
		markerArray_msg.markers.push_back(marker);
	}
  
	pub.publish(markerArray_msg);
}

void CameraPoseVisualization::publish_image_by( ros::Publisher &pub, const std_msgs::Header &header ) {
    image.header = header;

    pub.publish(image);
}
void CameraPoseVisualization::publish_parking_lot(ros::Publisher &marker_pub,double yaw,const Eigen::Vector3d& p){
	visualization_msgs::Marker line_strip;
	line_strip.header.frame_id = "world";
	line_strip.header.stamp = ros::Time::now();
	line_strip.ns = "parking_lot";
	line_strip.action = visualization_msgs::Marker::ADD;


	Eigen::AngleAxisd rotation_vector ( yaw, Eigen::Vector3d ( 0,0,1 ) );     //沿 Z 轴旋转 yaw

	Eigen::Quaterniond q = Eigen::Quaterniond(rotation_vector);
	line_strip.pose.orientation.w = 1;

	line_strip.id = 0;

	line_strip.type = visualization_msgs::Marker::LINE_STRIP;

	// LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
	line_strip.scale.x = 0.1;

	// Line strip is blue
	line_strip.color.r = 1.0;
	line_strip.color.a = 1.0;

	geometry_msgs::Point p1,p2,p3,p4;

	int w = 5;
	int h = 8;
	int z = 0;
	float scale = 1;
	const Eigen::Vector3d p1f = Eigen::Vector3d(0, 0, z);
	const Eigen::Vector3d p2f = Eigen::Vector3d(0, h, z);
	const Eigen::Vector3d p3f = Eigen::Vector3d(w, h, z);
	const Eigen::Vector3d p4f = Eigen::Vector3d(w, 0, z);

	Eigen2Point(q * (scale *p1f) + p, p1);
	Eigen2Point(q * (scale *p2f) + p, p2);
	Eigen2Point(q * (scale *p3f) + p, p3);
	Eigen2Point(q * (scale *p4f) + p, p4);

	line_strip.points.push_back(p1);
	line_strip.points.push_back(p2);
	line_strip.points.push_back(p3);
	line_strip.points.push_back(p4);
	marker_pub.publish(line_strip);
}
/*
void CameraPoseVisualization::add_image(const Eigen::Vector3d& T, const Eigen::Matrix3d& R, const cv::Mat &src)
{
    //image.points.clear();
    //image.colors.clear();

    image.ns = "image";
    image.id = 0;
    image.action = visualization_msgs::Marker::ADD;
    image.type = visualization_msgs::Marker::TRIANGLE_LIST;
    image.scale.x = 1;
    image.scale.y = 1;
    image.scale.z = 1;

    geometry_msgs::Point p;
    std_msgs::ColorRGBA crgb;

    double center_x = src.rows / 2.0;
    double center_y = src.cols / 2.0;

    //double scale = 0.01;
    double scale = IMAGE_VISUAL_SCALE;

    for(int r = 0; r < src.cols; ++r) {
        for(int c = 0; c < src.rows; ++c) {
          float intensity = (float)( src.at<uchar>(c, r));
          crgb.r = (float)intensity / 255.0;
          crgb.g = (float)intensity / 255.0;
          crgb.b = (float)intensity / 255.0;
          crgb.a = 1.0;

          Eigen::Vector3d p_cam, p_w;
          p_cam.z() = 0;
          p_cam.x() = (r - center_x) * scale;
          p_cam.y() = (c - center_y) * scale; 
          p_w = R * p_cam + T;
          p.x = p_w(0);
          p.y = p_w(1);
          p.z = p_w(2);
          image.points.push_back(p);
          image.colors.push_back(crgb);

          p_cam.z() = 0;
          p_cam.x() = (r - center_x + 1) * scale;
          p_cam.y() = (c - center_y) * scale; 
          p_w = R * p_cam + T;
          p.x = p_w(0);
          p.y = p_w(1);
          p.z = p_w(2);
          image.points.push_back(p);
          image.colors.push_back(crgb);

          p_cam.z() = 0;
          p_cam.x() = (r - center_x) * scale;
          p_cam.y() = (c - center_y + 1) * scale; 
          p_w = R * p_cam + T;
          p.x = p_w(0);
          p.y = p_w(1);
          p.z = p_w(2);
          image.points.push_back(p);
          image.colors.push_back(crgb);

          p_cam.z() = 0;
          p_cam.x() = (r - center_x + 1) * scale;
          p_cam.y() = (c - center_y) * scale; 
          p_w = R * p_cam + T;
          p.x = p_w(0);
          p.y = p_w(1);
          p.z = p_w(2);
          image.points.push_back(p);
          image.colors.push_back(crgb);

          p_cam.z() = 0;
          p_cam.x() = (r - center_x + 1) * scale;
          p_cam.y() = (c - center_y + 1) * scale; 
          p_w = R * p_cam + T;
          p.x = p_w(0);
          p.y = p_w(1);
          p.z = p_w(2);
          image.points.push_back(p);
          image.colors.push_back(crgb);

          p_cam.z() = 0;
          p_cam.x() = (r - center_x) * scale;
          p_cam.y() = (c - center_y + 1) * scale; 
          p_w = R * p_cam + T;
          p.x = p_w(0);
          p.y = p_w(1);
          p.z = p_w(2);
          image.points.push_back(p);
          image.colors.push_back(crgb);
        }
    }
}
*/
