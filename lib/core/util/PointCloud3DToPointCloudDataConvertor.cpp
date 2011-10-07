/*
 * PointCloud3DToPointCloudDataConvertor.cpp
 *
 *  Created on: Oct 7, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#include "PointCloud3DToPointCloudDataConvertor.h"

namespace BRICS_3D {

template <class PointT>
PointCloud3DToPointCloudDataConvertor<PointT>::PointCloud3DToPointCloudDataConvertor() {

}


template <>
void PointCloud3DToPointCloudDataConvertor<pcl::PointXYZRGB>::convert(){

	destinationCloudPtr->width = sourceCloud->getSize();
	destinationCloudPtr->height  = 1;
	destinationCloudPtr->is_dense  = true;
	destinationCloudPtr->points.resize (destinationCloudPtr->width * destinationCloudPtr->height);

	for (unsigned int i = 0; i < sourceCloud->getSize(); i++) {

		destinationCloudPtr->points[i].x = sourceCloud->getPointCloud()->data()[i].getX();
		destinationCloudPtr->points[i].y = sourceCloud->getPointCloud()->data()[i].getY();
		destinationCloudPtr->points[i].z = sourceCloud->getPointCloud()->data()[i].getZ();

			float rgb24bit=0;
			//convertRGBToRGB24bit();
			destinationCloudPtr->points[i].rgb = rgb24bit;

	}

}


template <>
void PointCloud3DToPointCloudDataConvertor<pcl::PointXYZRGB>::revert(){
//Todo change
	sourceCloud->getPointCloud()->clear();
	sourceCloud->getPointCloud()->resize(destinationCloudPtr->size());

	for (unsigned int i = 0; i < destinationCloudPtr->size(); i++) {
		sourceCloud->getPointCloud()->data()[i].setX(destinationCloudPtr->points[i].x);
		sourceCloud->getPointCloud()->data()[i].setY(destinationCloudPtr->points[i].y);
		sourceCloud->getPointCloud()->data()[i].setZ(destinationCloudPtr->points[i].z);
	}
}



template <>
void PointCloud3DToPointCloudDataConvertor<pcl::PointXYZ>::convert(){

	destinationCloudPtr->width = sourceCloud->getSize();
	destinationCloudPtr->height  = 1;
	destinationCloudPtr->is_dense  = true;
	destinationCloudPtr->points.resize (destinationCloudPtr->width * destinationCloudPtr->height);

	for (unsigned int i = 0; i < sourceCloud->getSize(); i++) {

		destinationCloudPtr->points[i].x = sourceCloud->getPointCloud()->data()[i].getX();
		destinationCloudPtr->points[i].y = sourceCloud->getPointCloud()->data()[i].getY();
		destinationCloudPtr->points[i].z = sourceCloud->getPointCloud()->data()[i].getZ();
	}
}


template <>
void PointCloud3DToPointCloudDataConvertor<pcl::PointXYZ>::revert(){

	sourceCloud->getPointCloud()->clear();
	sourceCloud->getPointCloud()->resize(destinationCloudPtr->size());

	for (unsigned int i = 0; i < destinationCloudPtr->size(); i++) {
		sourceCloud->getPointCloud()->data()[i].setX(destinationCloudPtr->points[i].x);
		sourceCloud->getPointCloud()->data()[i].setY(destinationCloudPtr->points[i].y);
		sourceCloud->getPointCloud()->data()[i].setZ(destinationCloudPtr->points[i].z);
	}
}



}
