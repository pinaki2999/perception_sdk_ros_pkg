/*
 * PoseEstimation6DNode.cpp
 *
 *  Created on: Oct 28, 2011
 *      Author: Pinaki Sunil Banerjee
 */


//ROS specific Headers
#include <ros/ros.h>

//PCL specific Headers
#include <pcl_ros/point_cloud.h>
#include "pcl/point_types.h"

//BRICS_3D specific headers
#include "examples/PoseEstimation6DExample.h"
//#include "examples/PoseEstimation6DExample_PCL.h"

//Sytem-wide Standard Headers
#include <fstream>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <stdlib.h>
#include <sstream>



//global variables


//void kinectCloudCallback(const sensor_msgs::PointCloud2 &cloud){
//	for(int i=0; i<noOfRegions;i++){
//		ROS_INFO("received a kinect message...");
//		objectClusterExtractor[i].kinectCloudCallback(cloud);
//	}
//}

int main(int argc, char* argv[]){


	int maxNoOfObjects;
	int noOfRegions;

	ros::init(argc, argv, "poseEstimator6D");
	ros::NodeHandle nh;

	if(argc == 2){
		noOfRegions = atoi(argv[1]);
		maxNoOfObjects = 1;
	} else if(argc == 3){
		noOfRegions = atoi(argv[1]);
		maxNoOfObjects = atoi(argv[2]);
	} else {
		ROS_INFO("Using default values");
		noOfRegions = 1;
		maxNoOfObjects = 1;
	}

	ROS_INFO("Finding pose for at most [%d] object-cluster(s) in [%d] extracted regions",
																maxNoOfObjects, noOfRegions);

	//Define the publishers for each estimated model
	ros::Publisher estimatedModelPublisher[noOfRegions][maxNoOfObjects];

	//Define the model estimators
	//Todo replace malloc
	BRICS_3D::PoseEstimation6DExample poseEstimator[noOfRegions][maxNoOfObjects];
//	if( (poseEstimator = (BRICS_3D::PoseEstimation6DExample *)malloc
//			(noOfRegions*maxNoOfObjects*sizeof(BRICS_3D::PoseEstimation6DExample ))) == NULL ){
//		ROS_ERROR("Memory Allocation Error!!");
//		exit(0);
//	}


	for (int i = 0; i<noOfRegions; i++){
		//initialize the publishers
		for(int j=0; j<maxNoOfObjects; j++){
			std::stringstream pubTopic;
			pubTopic.str("");
			pubTopic.clear();
			pubTopic << "region_" << i+1 << "_obj_model_" << j+1;
			estimatedModelPublisher[i][j] = nh.advertise< pcl::PointCloud<pcl::PointXYZ> >
																				(pubTopic.str(), 1);
		}

		//initialize the model estimators
//		objectClusterExtractor[i].initializeExtractor(maxNoOfObjects,extractedClusterPublisher[i],
//																				200,25000, 0.01);
	}

	//subscribe to kinect point cloud messages
	//extracted_region_

    ros::Subscriber  objectClusterSubscriber[noOfRegions*maxNoOfObjects];

    for (int i = 0; i < noOfRegions ; i++){
    	for (int j = 0; j < maxNoOfObjects; j++) {
        	std::stringstream subTopic;
        	subTopic.str("");
        	subTopic.clear();
        	subTopic << "region_"<< i+1 << "_obj_cluster_" << j+1;
        	objectClusterSubscriber[i]= nh.subscribe(subTopic.str(), 1,
        			&BRICS_3D::PoseEstimation6DExample::kinectCloudCallback, &poseEstimator[i][j]);
        	poseEstimator[i][j].setModelPublisher(&estimatedModelPublisher[i][j]);
		}
    }

    ROS_INFO("Now estimating models;)");

	ros::spin();

	//free(objectClusterExtractor);
	return 0;
}
