/*
 * PoseEstimation6DExample.cpp
 *
 *  Created on: Oct 28, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#include "PoseEstimation6DExample.h"

namespace BRICS_3D {

PoseEstimation6DExample::PoseEstimation6DExample() {

	cubeModelGenerator.setPointsOnEachSide(100);
	cubeModelGenerator.setCubeSideLength(0.06);

	cubeModelGenerator.setNumOfFaces(2);
	cubeModelGenerator.generatePointCloud(cube2D);

	cubeModelGenerator.setNumOfFaces(3);
	cubeModelGenerator.generatePointCloud(cube3D);

	reliableScoreThreshold = 0.00008;

}

PoseEstimation6DExample::~PoseEstimation6DExample() {
	// TODO Auto-generated destructor stub
	delete cube2D;
	delete cube3D;
}

void PoseEstimation6DExample::kinectCloudCallback(const sensor_msgs::PointCloud2 &cloud){


	//Transforming Input message to BRICS_3D format
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr estimated_model_ptr(new pcl::PointCloud<pcl::PointXYZ>());

    BRICS_3D::PointCloud3D *in_cloud = new BRICS_3D::PointCloud3D();

	    //Transform sensor_msgs::PointCloud2 msg to pcl::PointCloud
	    pcl::fromROSMsg (cloud, *cloud_xyz_ptr);

	    // cast PCL to BRICS_3D type
	    pclTypecaster.convertToBRICS3DDataType(cloud_xyz_ptr, in_cloud);
	    ROS_INFO("Size of input cloud: %d ", in_cloud->getSize());


	//Performing 2D model alignment
	BRICS_3D::PointCloud3D *finalModel2D = new BRICS_3D::PointCloud3D();
	poseEstimatorICP.setDistance(5);
	//poseEstimatorICP.setDistance(0.01);
	poseEstimatorICP.setMaxIterations(1000);
	poseEstimatorICP.setObjectModel(cube2D);
	poseEstimatorICP.estimatePose(in_cloud, finalModel2D);
	float score2D = poseEstimatorICP.getFitnessScore();
/*

	//Performing 2D model alignment
		BRICS_3D::PointCloud3D *finalModel3D = new BRICS_3D::PointCloud3D();
		poseEstimatorICP.setDistance(0.01);
		poseEstimatorICP.setMaxIterations(1000);
		poseEstimatorICP.setObjectModel(cube3D);
		poseEstimatorICP.estimatePose(in_cloud, finalModel3D);
		float score3D = poseEstimatorICP.getFitnessScore();

	if(score2D<score3D){
		//publish model estimated using two sided cube
		if(score2D > reliableScoreThreshold){
			ROS_INFO("[%s] Approximate Model Found!! Object May Not be visible enough...", modelPublisher->getTopic().c_str());
		} else {
			ROS_INFO("[%s] Reliable Model Found :) ", modelPublisher->getTopic().c_str());
		}
		pclTypecaster.convertToPCLDataType(estimated_model_ptr,finalModel2D);
	} else {
		//publish model estimated using three sided cube
		if(score3D > reliableScoreThreshold){
			ROS_INFO("[%s] Approximate Model Found!! Object May Not be visible enough...", modelPublisher->getTopic().c_str());
		}else {
			ROS_INFO("[%s] Reliable Model Found :) ", modelPublisher->getTopic().c_str());
		}
		pclTypecaster.convertToPCLDataType(estimated_model_ptr,finalModel3D);
	}

	estimated_model_ptr->header.frame_id = "/openni_rgb_optical_frame";
	modelPublisher->publish(*estimated_model_ptr);
*/
	delete in_cloud;
	delete finalModel2D;
//	delete finalModel3D;

	}
}

