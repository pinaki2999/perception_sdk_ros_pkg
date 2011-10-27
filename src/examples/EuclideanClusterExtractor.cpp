/*
 * EuclideanClusterExtractor.cpp
 *
 *  Created on: Oct 14, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#include "EuclideanClusterExtractor.h"

namespace BRICS_3D{

EuclideanClusterExtractor::EuclideanClusterExtractor() {}

ros::Publisher *EuclideanClusterExtractor::getExtractedClusterPublisher() const
{
	return extractedClusterPublisher;
}

int EuclideanClusterExtractor::getMaxNoOfObjects() const
{
	return maxNoOfObjects;
}

void EuclideanClusterExtractor::setMaxNoOfObjects(int maxNoOfObjects)
{
	this->maxNoOfObjects = maxNoOfObjects;
}

void EuclideanClusterExtractor::setExtractedClusterPublisher(ros::Publisher *extractedClusterPublisher){
	this->extractedClusterPublisher = extractedClusterPublisher;
}

EuclideanClusterExtractor::~EuclideanClusterExtractor() {}


void EuclideanClusterExtractor::kinectCloudCallback(const sensor_msgs::PointCloud2 &cloud){

	//ToDo assert if every thing is initialized or not

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyz_rgb_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

    BRICS_3D::PointCloud3D *in_cloud = new BRICS_3D::PointCloud3D();
    std::vector<BRICS_3D::PointCloud3D*> extracted_clusters;


    //Transform sensor_msgs::PointCloud2 msg to pcl::PointCloud
    pcl::fromROSMsg (cloud, *cloud_xyz_rgb_ptr);
    // cast PCL to BRICS_3D type
    pclTypecaster.convertToBRICS3DDataType(cloud_xyz_rgb_ptr, in_cloud);
    //extract the clusters
    euclideanClusterExtractor.extractClusters(in_cloud, &extracted_clusters);
    ROS_INFO("No of clusters found: %d", extracted_clusters.size());
    //Publish the extracted clusters
	pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (unsigned int i = 0; i < extracted_clusters.size(); i++){
    	Eigen::Vector3d centroid3d = centroid3DEstimator.computeCentroid(extracted_clusters[i]);

    	pclTypecaster.convertToPCLDataType(tempCloud, extracted_clusters[i]);
    	tempCloud->header.frame_id = "/openni_rgb_optical_frame";
    	extractedClusterPublisher[i].publish(*tempCloud);

    	ROS_INFO("Cluster Position 3D: [%f, %f, %f]", centroid3d[0], centroid3d[1], centroid3d[2]);

         static tf::TransformBroadcaster br;
         tf::Transform transform;
         transform.setOrigin( tf::Vector3(centroid3d[0], centroid3d[1], centroid3d[2]) );
         //Todo stop using Quaternion
         transform.setRotation( tf::Quaternion(0, 0, 0) );
         br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/openni_rgb_optical_frame",
        		 extractedClusterPublisher[i].getTopic()));
    }

    tempCloud.reset();
    delete(in_cloud);
    extracted_clusters.clear();

}

void EuclideanClusterExtractor::initializeExtractor(int maxNoObjects,
		ros::Publisher *extractedClusterPublisher, int minClusterSize, int maxClusterSize,
																		float clusterTolerance){

	setMinClusterSize(minClusterSize);
	setMaxClusterSize(maxClusterSize);
	setClusterTolerance(clusterTolerance);
	setMaxNoOfObjects(maxNoObjects);
	setExtractedClusterPublisher(extractedClusterPublisher);

}

}
