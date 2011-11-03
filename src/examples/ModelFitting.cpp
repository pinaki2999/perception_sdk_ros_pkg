/*
 * PoseEstimation6DExample.cpp
 *
 *  Created on: Oct 28, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#include "ModelFitting.h"

namespace BRICS_3D {

ModelFitting::ModelFitting() {

	cube2D = new BRICS_3D::PointCloud3D();
	cube3D = new BRICS_3D::PointCloud3D();

	cubeModelGenerator.setPointsOnEachSide(5);
	cubeModelGenerator.setCubeSideLength(0.06);

	cubeModelGenerator.setNumOfFaces(2);
	cubeModelGenerator.generatePointCloud(cube2D);

	cubeModelGenerator.setNumOfFaces(3);
	cubeModelGenerator.generatePointCloud(cube3D);

	Eigen::Matrix4f  tempHomogenousMatrix;
	calculateHomogeneousMatrix(90,0,0,0,0,0,tempHomogenousMatrix,true);
	BRICS_3D::HomogeneousMatrix44* homogeneousTrans = new HomogeneousMatrix44(
			tempHomogenousMatrix[0], tempHomogenousMatrix[4], tempHomogenousMatrix[8],
			tempHomogenousMatrix[1], tempHomogenousMatrix[5], tempHomogenousMatrix[9],
			tempHomogenousMatrix[2], tempHomogenousMatrix[6], tempHomogenousMatrix[10],
			0,0,0);

	cube2D->homogeneousTransformation(homogeneousTrans);
	cube3D->homogeneousTransformation(homogeneousTrans);

	ROS_INFO("Initialization Done....");
	reliableScoreThreshold = 0.00008;
	bestScore = 1000;
}

ModelFitting::~ModelFitting() {
	// TODO Auto-generated destructor stub
	delete cube2D;
	delete cube3D;
}

void ModelFitting::kinectCloudCallback(const sensor_msgs::PointCloud2 &cloud){

	ROS_INFO("Looking For: %s ", modelPublisher->getTopic().c_str());
	//Transforming Input message to BRICS_3D format
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_ptr(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr estimated_model_ptr(new pcl::PointCloud<pcl::PointXYZ>());

	BRICS_3D::PointCloud3D *in_cloud = new BRICS_3D::PointCloud3D();

	//Transform sensor_msgs::PointCloud2 msg to pcl::PointCloud
	pcl::fromROSMsg (cloud, *cloud_xyz_ptr);

	// cast PCL to BRICS_3D type
	pclTypecaster.convertToBRICS3DDataType(cloud_xyz_ptr, in_cloud);
	ROS_INFO("Size of input cloud: %d ", in_cloud->getSize());


	//=============================================================================================
	BRICS_3D::PointCloud3D *finalModel2D = new BRICS_3D::PointCloud3D();
	BRICS_3D::PointCloud3D *finalModel3D = new BRICS_3D::PointCloud3D();
	BRICS_3D::PointCloud3D *transformedCubeModel2D = new BRICS_3D::PointCloud3D();
	BRICS_3D::PointCloud3D *transformedCubeModel3D = new BRICS_3D::PointCloud3D();

	BRICS_3D::Centroid3D centroidEstimator;
	Eigen::Vector3d centroid3d = centroidEstimator.computeCentroid(in_cloud);
	float xTrans = centroid3d[0];
	float yTrans = centroid3d[1];
	float zTrans = centroid3d[2];

	ROS_INFO("Initial estimate \n\tTranslation-[x,y,z,]=[%f,%f,%f]",xTrans, yTrans, zTrans);



	/**Translate the cube models which will be our initial estimate for ICP*/


	Eigen::Matrix4f  tempHomogenousMatrix;
	calculateHomogeneousMatrix(0,0,0,xTrans,yTrans,zTrans,tempHomogenousMatrix,true);
	BRICS_3D::HomogeneousMatrix44* homogeneousTrans = new HomogeneousMatrix44(
			1, 0, 0,
			0, 1, 0,
			0, 0, 1,
			xTrans,yTrans,zTrans);

	for(unsigned int i=0; i<cube2D->getSize();i++){
		BRICS_3D::Point3D *tempPoint = new BRICS_3D::Point3D(cube2D->getPointCloud()->data()[i].getX(),
				cube2D->getPointCloud()->data()[i].getY(),
				cube2D->getPointCloud()->data()[i].getZ());
		transformedCubeModel2D->addPoint(tempPoint);
		delete tempPoint;
	}
	transformedCubeModel2D->homogeneousTransformation(homogeneousTrans);


	for(unsigned int i=0; i<cube3D->getSize();i++){
		BRICS_3D::Point3D *tempPoint = new BRICS_3D::Point3D(cube3D->getPointCloud()->data()[i].getX(),
				cube3D->getPointCloud()->data()[i].getY(),
				cube3D->getPointCloud()->data()[i].getZ());
		transformedCubeModel3D->addPoint(tempPoint);
		delete tempPoint;
	}
	ROS_INFO("Resultant cloud size: %d", transformedCubeModel3D->getSize());
	transformedCubeModel3D->homogeneousTransformation(homogeneousTrans);



	//Performing 2D model alignment
	poseEstimatorICP.setDistance(0.1);
	poseEstimatorICP.setMaxIterations(1000);
	poseEstimatorICP.setObjectModel(transformedCubeModel2D);
	poseEstimatorICP.estimateBestFit(in_cloud, finalModel2D);
	float score2D = poseEstimatorICP.getFitnessScore();

	//Performing 3D model alignment
	poseEstimatorICP.setObjectModel(transformedCubeModel3D);
	poseEstimatorICP.estimateBestFit(in_cloud, finalModel3D);
	float score3D = poseEstimatorICP.getFitnessScore();

	if(score2D<score3D){
		//publish model estimated using two sided cube
		if(score2D < bestScore){
			if(score2D > reliableScoreThreshold){
				ROS_INFO("[%s] Approximate Model Found(2D)!! Object May Not be visible enough...",
						modelPublisher->getTopic().c_str());
			} else {
				ROS_INFO("[%s] Reliable Model Found(2D) :) ", modelPublisher->getTopic().c_str());
			}
		}
		pclTypecaster.convertToPCLDataType(estimated_model_ptr,finalModel2D);
		ROS_INFO("Best score found by 3D model : %f", score2D);
		bestScore = score2D;

	} else {
		//publish model estimated using three sided cube
		if(score3D<bestScore){
			if(score3D > reliableScoreThreshold){
				ROS_INFO("[%s] Approximate Model Found(3D)!! Object May Not be visible enough...",
						modelPublisher->getTopic().c_str());
			}else {
				ROS_INFO("[%s] Reliable Model Found(3D) :) ", modelPublisher->getTopic().c_str());
			}
		}
		pclTypecaster.convertToPCLDataType(estimated_model_ptr,finalModel3D);
		ROS_INFO("Best score found by 3D model : %f", score3D);
		bestScore=score3D;

	}

	ROS_INFO("Resultant cloud size: %d", transformedCubeModel3D->getSize());
	estimated_model_ptr->header.frame_id = "/openni_rgb_optical_frame";
	modelPublisher->publish(*estimated_model_ptr);


	delete in_cloud;
	delete finalModel2D;
	delete finalModel3D;
	delete transformedCubeModel2D;
	delete transformedCubeModel3D;
}
}
