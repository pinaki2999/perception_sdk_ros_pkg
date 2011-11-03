/*
 * PoseEstimation6DNode.cpp
 *
 *  Created on: Oct 28, 2011
 *      Author: Pinaki Sunil Banerjee
 */


//ROS specific Headers
#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "perception_sdk_ros_pkg/Coordination.h"

//PCL specific Headers
#include <pcl_ros/point_cloud.h>
#include "pcl/point_types.h"

//BRICS_3D specific headers
#include "examples/PoseEstimation6D.h"

//Sytem-wide Standard Headers
#include <fstream>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <stdlib.h>
#include <sstream>



//global variables
std::vector<BRICS_3D::PoseEstimation6D*> poseEstimators;
bool perceptionPaused;
int maxNoOfObjects;
int noOfRegions;


void kinectCloudCallback(const sensor_msgs::PointCloud2 &cloud){
	if(!perceptionPaused){
		for(int i=0; i<noOfRegions;i++){
//			ROS_INFO("received a kinect message...");
			poseEstimators[i]->kinectCloudCallback(cloud);
		}
	} else {
		ROS_INFO("Perception Engine Paused");
	}
}

void perceptionControlCallback(const perception_sdk_ros_pkg::Coordination message){

	if(!message.command.compare("pause")){
		perceptionPaused=true;
	}else if(!message.command.compare("resume")){
		perceptionPaused=false;
	}

}


int main(int argc, char* argv[]){

	perceptionPaused = false;

	ros::init(argc, argv, "PoseEstimation6D");
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

	ROS_INFO("Finding pose for at most [%d] object(s) in [%d] color based extracted regions",
			maxNoOfObjects, noOfRegions);

	//Define the model estimators
	for (int i = 0; i< noOfRegions; i++){
		poseEstimators.push_back(new BRICS_3D::PoseEstimation6D());
	}


	//subscribe to perception engine control messsage
	ros::Subscriber  perceptionControlSubscriber;
		perceptionControlSubscriber= nh.subscribe("/perceptionControl", 1,&perceptionControlCallback);

	//subscribe to kinect point cloud messages
	ros::Subscriber  kinectCloudSubscriber[noOfRegions];
	for (int i = 0; i < noOfRegions ; i++){
		kinectCloudSubscriber[i]= nh.subscribe("/camera/rgb/points", 1,&kinectCloudCallback);
		poseEstimators[i]->setMaxNoOfObjects(maxNoOfObjects);
	}

	ros::spin();

	return 0;
}
