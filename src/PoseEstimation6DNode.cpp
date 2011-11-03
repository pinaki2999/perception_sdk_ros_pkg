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
#include "ros/publisher.h"

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
	noOfRegions = 0;
	maxNoOfObjects = 0;

	if(argc < 4){


		ROS_ERROR("Not enough arguments:\n Usage:\t "
				"rosrun poseEstimator6D <no_of_regions> <maxNoOfObjects> <region_1_config_file> "
				"<region_2_config_file>.... <region_label_1> <region_label_2>\n"
				"\nExample: rosrun poseEstimator6D 2 3 redRoiConfig.cfg greenRoiConfig.cfg red green");
		exit(0);
	} else if(argc != (3+ 2*atoi(argv[1]))){
		ROS_ERROR("Not enough arguments:\n Usage:\t "
						"rosrun colorBasedRoiExtractor <no_of_regions> <maxNoOfObjects> <region_1_config_file> "
						"<region_2_config_file>.... <region_label_1> <region_label_2>");
				exit(0);
	}
	noOfRegions = atoi(argv[1]);
	maxNoOfObjects = atoi(argv[2]);

	ROS_INFO("Finding pose for at most [%d] object(s) in [%d] color based extracted regions",
			maxNoOfObjects, noOfRegions);

	//define the HSV limit variables;
	float minLimitH[noOfRegions], minLimitS[noOfRegions],
			maxLimitH[noOfRegions], maxLimitS[noOfRegions];

	//parse the configuration files and set up the HSV limits and set up the pose estimators
	std::ifstream configFileStream;
	for (int i = 0; i< noOfRegions; i++){

		poseEstimators.push_back(new BRICS_3D::PoseEstimation6D());

		configFileStream.open(argv[i+3]);
		      if ( configFileStream.is_open() ) {     //if file exists
		          std::string s;
		          while(getline(configFileStream, s)){    //extract the values of the parameters
		              	std::vector< std::string > tempVec;
		                  boost::split(tempVec, s, boost::is_any_of("="));
		                  if(!tempVec[0].compare("minH")) {
		                       minLimitH[i] = atof(tempVec[1].c_str());
		                  } else if(!tempVec[0].compare("maxH")) {
		                       maxLimitH[i] = atof(tempVec[1].c_str());
		                  } else if(!tempVec[0].compare("minS")) {
		                       minLimitS[i] = atof(tempVec[1].c_str());
		                  } else if(!tempVec[0].compare("maxS")) {
		                       maxLimitS[i] = atof(tempVec[1].c_str());
		                  }
		          }
		          configFileStream.close();
		      } else {
		    	  ROS_ERROR("Configuration file: %s not found!!", argv[i+3]);
		    	  exit(0);
		      }

		poseEstimators[i]->setRegionLabel(argv[3+noOfRegions+i]);
		poseEstimators[i]->initializeLimits(minLimitH[i], maxLimitH[i], minLimitS[i], maxLimitS[i]);
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
