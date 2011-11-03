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

	this->euclideanClusterExtractor.setMinClusterSize(0);
	this->euclideanClusterExtractor.setMaxClusterSize(0);
	this->euclideanClusterExtractor.setClusterTolerance(0);
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

	//perform HSV color based extraction
	hsvBasedRoiExtractor.extractColorBasedROI(in_cloud, color_based_roi);
	ROS_INFO("[%s] Size of extracted cloud : %d ", regionLabel.c_str(),color_based_roi->getSize());


    //extract the clusters
    euclideanClusterExtractor.extractClusters(color_based_roi, &extracted_clusters);
    ROS_INFO("[%s] No of clusters found: %d", regionLabel.c_str(), extracted_clusters.size());



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


//    	Eigen::Vector3d centroid3d = centroid3DEstimator.computeCentroid(extracted_clusters[i]);
//
//    	ROS_INFO("Cluster Position 3D: [%f, %f, %f]", centroid3d[0], centroid3d[1], centroid3d[2]);

//         static tf::TransformBroadcaster br;
//         tf::Transform transform;
//         transform.setOrigin( tf::Vector3(centroid3d[0], centroid3d[1], centroid3d[2]) );
//         //Todo stop using Quaternion
//         transform.setRotation( tf::Quaternion(0, 0, 0) );
//         br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/openni_rgb_optical_frame",
//        		 extractedClusterPublisher[i].getTopic()));
    	}
    }


	delete in_cloud;
	delete color_based_roi;
	extracted_clusters.clear();

}

}
