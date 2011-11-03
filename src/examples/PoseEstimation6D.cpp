/*
 * PoseEstimation6D.cpp
 *
 *  Created on: Nov 3, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#include "PoseEstimation6D.h"

namespace BRICS_3D {

PoseEstimation6D::PoseEstimation6D() {
	this->hsvBasedRoiExtractor.setMinH(0);
	this->hsvBasedRoiExtractor.setMaxH(0);
	this->hsvBasedRoiExtractor.setMinS(0);
	this->hsvBasedRoiExtractor.setMaxS(0);
}

int PoseEstimation6D::getMaxNoOfObjects() const
{
	return maxNoOfObjects;
}

std::string PoseEstimation6D::getRegionLabel() const
{
	return regionLabel;
}

void PoseEstimation6D::setRegionLabel(std::string regionLabel)
{
	this->regionLabel = regionLabel;
}

void PoseEstimation6D::setMaxNoOfObjects(int maxNoOfObjects)
{
	this->maxNoOfObjects = maxNoOfObjects;
}

PoseEstimation6D::~PoseEstimation6D() {
	// TODO Auto-generated destructor stub
}

void PoseEstimation6D::initializeLimits(float minLimitH, float maxLimitH, float minLimitS, float maxLimitS){
	this->hsvBasedRoiExtractor.setMinH(minLimitH);
	this->hsvBasedRoiExtractor.setMaxH(maxLimitH);
	this->hsvBasedRoiExtractor.setMinS(minLimitS);
	this->hsvBasedRoiExtractor.setMaxS(maxLimitS);
}

void PoseEstimation6D::kinectCloudCallback(const sensor_msgs::PointCloud2 &cloud){

	std::cout << "Estimating Pose 6D :) :)"<< std::endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyz_rgb_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());


	BRICS_3D::ColoredPointCloud3D *in_cloud = new BRICS_3D::ColoredPointCloud3D();
	BRICS_3D::ColoredPointCloud3D *color_based_roi = new BRICS_3D::ColoredPointCloud3D();

	//Transform sensor_msgs::PointCloud2 msg to pcl::PointCloud
	pcl::fromROSMsg (cloud, *cloud_xyz_rgb_ptr);

	// cast PCL to BRICS_3D type
	pclTypeCaster.convertToBRICS3DDataType(cloud_xyz_rgb_ptr, in_cloud);
	ROS_INFO("Size of input cloud: %d ", in_cloud->getSize());

	//perform HSV color based extraction
	hsvBasedRoiExtractor.extractColorBasedROI(in_cloud, color_based_roi);
	ROS_INFO("[%s] Size of extracted cloud : %d ", regionLabel.c_str(),color_based_roi->getSize());


	delete in_cloud;
	delete color_based_roi;

}

}
