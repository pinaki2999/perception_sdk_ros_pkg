/*
 * PoseEstimationICP.cpp
 *
 *  Created on: Oct 28, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#include "PoseEstimationICP.h"

namespace BRICS_3D {

namespace SDK {

PoseEstimationICP::PoseEstimationICP() {
	distance = 0.1;
	transformationEpsilon = 1e-6;
	maxIterations = 1000;
}


PoseEstimationICP::~PoseEstimationICP() {
	// TODO Auto-generated destructor stub
}


void PoseEstimationICP::estimatePose(BRICS_3D::PointCloud3D *inCloud, BRICS_3D::PointCloud3D *outCloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
	BRICS_3D::PCLTypecaster pclTypecaster;

	pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> icp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr objectModelPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> ::Ptr FinalCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);

    pclTypecaster.convertToPCLDataType(targetCloudPtr, inCloud);
    std::cout << "object model cloud size: " << objectModel->getSize()<< std::endl;;
    pclTypecaster.convertToPCLDataType(objectModelPtr, objectModel);

     icp.setInputCloud(objectModelPtr);
     icp.setInputTarget(targetCloudPtr);
     icp.setMaxCorrespondenceDistance(distance);
     icp.setTransformationEpsilon (transformationEpsilon);
     icp.setMaximumIterations (maxIterations);

     icp.align(*FinalCloudPtr);

     this->fitnessScore=icp.getFitnessScore();
     this->finalTransformation = icp.getFinalTransformation();

     //Fixme if required ,)

     pclTypecaster.convertToPCLDataType(FinalCloudPtr, outCloud);

}

}

}
