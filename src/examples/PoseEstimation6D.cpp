/*
 * PoseEstimation6D.cpp
 *
 *  Created on: Nov 3, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#include "PoseEstimation6D.h"

namespace BRICS_3D {

PoseEstimation6D::PoseEstimation6D() {
	//default initialization of roi extractor
	this->hsvBasedRoiExtractor.setMinH(0);
	this->hsvBasedRoiExtractor.setMaxH(0);
	this->hsvBasedRoiExtractor.setMinS(0);
	this->hsvBasedRoiExtractor.setMaxS(0);

	//default initialization of cluster extractor
	this->euclideanClusterExtractor.setMinClusterSize(0);
	this->euclideanClusterExtractor.setMaxClusterSize(0);
	this->euclideanClusterExtractor.setClusterTolerance(0);

	poseEstimatorICP = new BRICS_3D::SDK::IterativeClosestPoint();

	//default initialization of cube models
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
	maxNoOfObjects = 0;

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
	delete cube2D;
	delete cube3D;
	delete poseEstimatorICP;
}

void PoseEstimation6D::initializeLimits(float minLimitH, float maxLimitH, float minLimitS,
		float maxLimitS){
	this->hsvBasedRoiExtractor.setMinH(minLimitH);
	this->hsvBasedRoiExtractor.setMaxH(maxLimitH);
	this->hsvBasedRoiExtractor.setMinS(minLimitS);
	this->hsvBasedRoiExtractor.setMaxS(maxLimitS);
}

void PoseEstimation6D::initializeClusterExtractor(int minClusterSize, int maxClusterSize,
		float clusterTolerance){

	this->euclideanClusterExtractor.setMinClusterSize(minClusterSize);
	this->euclideanClusterExtractor.setMaxClusterSize(maxClusterSize);
	this->euclideanClusterExtractor.setClusterTolerance(clusterTolerance);
}

void PoseEstimation6D::estimatePose(BRICS_3D::PointCloud3D *in_cloud, int objCount){

	Eigen::Vector3d centroid3d = centroid3DEstimator.computeCentroid(in_cloud);
	float xTrans = centroid3d[0];
	float yTrans = centroid3d[1];
	float zTrans = centroid3d[2];

	//	ROS_INFO("Initial estimate \n\tTranslation-[x,y,z,]=[%f,%f,%f]",xTrans, yTrans, zTrans);

	//Translate the cube models which will be our initial estimate for ICP
	Eigen::Matrix4f  tempHomogenousMatrix;
	calculateHomogeneousMatrix(0,0,0,xTrans,yTrans,zTrans,tempHomogenousMatrix,true);
	BRICS_3D::HomogeneousMatrix44* homogeneousTrans = new HomogeneousMatrix44(
			1, 0, 0,
			0, 1, 0,
			0, 0, 1,
			xTrans,yTrans,zTrans);

	BRICS_3D::PointCloud3D *transformedCubeModel2D = new BRICS_3D::PointCloud3D();
	BRICS_3D::PointCloud3D *transformedCubeModel3D = new BRICS_3D::PointCloud3D();

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

	BRICS_3D::PointCloud3D *finalModel2D = new BRICS_3D::PointCloud3D();
	BRICS_3D::PointCloud3D *finalModel3D = new BRICS_3D::PointCloud3D();

	//Performing 2D model alignment
	poseEstimatorICP->setDistance(0.1);
	poseEstimatorICP->setMaxIterations(1000);
	poseEstimatorICP->setObjectModel(transformedCubeModel2D);
	poseEstimatorICP->estimateBestFit(in_cloud, finalModel2D);
	float score2D = poseEstimatorICP->getFitnessScore();

	//Performing 3D model alignment
	poseEstimatorICP->setObjectModel(transformedCubeModel3D);
	poseEstimatorICP->estimateBestFit(in_cloud, finalModel3D);
	float score3D = poseEstimatorICP->getFitnessScore();

	if(score2D<score3D){
		//publish model estimated using two sided cube
		if(score2D < bestScore){
			if(score2D > reliableScoreThreshold){
				ROS_INFO("[%s_%d] Approximate Model Found(2D)!! Object May Not be visible enough...",
						regionLabel.c_str(),objCount);
			} else {
				ROS_INFO("[%s_%d] Reliable Model Found(2D) :) ", regionLabel.c_str(), objCount);
			}
		}
		ROS_INFO("[%s_%d] Best score found by 2D model : %f", regionLabel.c_str(), objCount,score2D);
		bestScore = score2D;

	} else {
		//publish model estimated using three sided cube
		if(score3D<bestScore){
			if(score3D > reliableScoreThreshold){
				ROS_INFO("[%s_%d] Approximate Model Found(3D)!! Object May Not be visible enough...",
						regionLabel.c_str(),objCount);
			}else {
				ROS_INFO("[%s_%d] Reliable Model Found(3D) :) ", regionLabel.c_str(), objCount);
			}
		}
		ROS_INFO("[%s_%d] Best score found by 3D model : %f", regionLabel.c_str(), objCount, score3D);
		bestScore=score3D;
	}


	delete finalModel2D;
	delete finalModel3D;
	delete transformedCubeModel2D;
	delete transformedCubeModel3D;

}

void PoseEstimation6D::kinectCloudCallback(const sensor_msgs::PointCloud2 &cloud){

	std::cout << "Estimating Pose 6D :) :)"<< std::endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyz_rgb_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

	BRICS_3D::ColoredPointCloud3D *in_cloud = new BRICS_3D::ColoredPointCloud3D();
	BRICS_3D::PointCloud3D *color_based_roi = new BRICS_3D::ColoredPointCloud3D();
	std::vector<BRICS_3D::PointCloud3D*> extracted_clusters;

	//Transform sensor_msgs::PointCloud2 msg to pcl::PointCloud
	pcl::fromROSMsg (cloud, *cloud_xyz_rgb_ptr);

	// cast PCL to BRICS_3D type
	pclTypecaster.convertToBRICS3DDataType(cloud_xyz_rgb_ptr, in_cloud);
	ROS_INFO("Size of input cloud: %d ", in_cloud->getSize());

	/**==================================================================================================== [color based roi extraction]**/
	//perform HSV color based extraction
	hsvBasedRoiExtractor.extractColorBasedROI(in_cloud, color_based_roi);
	ROS_INFO("[%s] Size of extracted cloud : %d ", regionLabel.c_str(),color_based_roi->getSize());

	/**==================================================================================================== [cluster extraction]**/
	//extract the clusters
	euclideanClusterExtractor.extractClusters(color_based_roi, &extracted_clusters);
	ROS_INFO("[%s] No of clusters found: %d", regionLabel.c_str(), extracted_clusters.size());


	/**==================================================================================================== [6D pose estimation]**/
	//Estimate 6D Pose
	int regions;
	if(extracted_clusters.size() < abs(maxNoOfObjects)) {
		regions = extracted_clusters.size();
	} else {
		regions = maxNoOfObjects;
	}

	for (int i = 0; i < regions; i++){
		if(extracted_clusters[i]->getSize() > 0){
			//Estimate Pose
			estimatePose(extracted_clusters[i],i+1);
		}
	}


	delete in_cloud;
	delete color_based_roi;
	extracted_clusters.clear();

}

}
