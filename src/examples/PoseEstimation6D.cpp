/*
 * PoseEstimation6D.cpp
 *
 *  Created on: Nov 3, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#include "PoseEstimation6D.h"

namespace BRICS_3D {

PoseEstimation6D::PoseEstimation6D() {
	// TODO Auto-generated constructor stub

}

int PoseEstimation6D::getMaxNoOfObjects() const
{
	return maxNoOfObjects;
}

void PoseEstimation6D::setMaxNoOfObjects(int maxNoOfObjects)
{
	this->maxNoOfObjects = maxNoOfObjects;
}

PoseEstimation6D::~PoseEstimation6D() {
	// TODO Auto-generated destructor stub
}

void PoseEstimation6D::kinectCloudCallback(const sensor_msgs::PointCloud2 &cloud){
	//ToDo Implement
	std::cout << "Estimating Pose 6D :) :)"<< std::endl;
}

}
