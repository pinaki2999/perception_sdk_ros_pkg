/*
 * ControlPerception.cpp
 *
 *  Created on: Nov 4, 2011
 *      Author: Pinaki Sunil Banerjee
 */

//ROS specific Headers
#include <ros/ros.h>
#include "perception_sdk_ros_pkg/Configuration.h"
#include "ros/publisher.h"
#include <iostream>
int main(int argc, char* argv[]){

	ros::init(argc, argv, "ConfigurePerception");
	ros::NodeHandle nh;

	ros::Publisher perceptionCommandPublisher = nh.advertise<perception_sdk_ros_pkg::Configuration>  ("/perceptionConfiguration", 1);


	std::string choice;
	bool configure = true;
	perception_sdk_ros_pkg::Configuration message;

	while(configure){

		std::cout << "\n Please enter the maximum number of regions : \n ";
		std::cin >> choice;
		message.no_of_regions = atoi(choice.c_str());

		std::cout << "Please enter the maximum number of objects to be searched for in each region"
				" : \n";
		std::cin >> choice;
		message.max_no_of_objects = atoi(choice.c_str());

		std::cout << "Enter the  HSV-Limits configuration filenames for all the regions  "
				"(separated by space) : \n";
		std::cin >> choice;
		message.config_files = choice;

		std::cout << "Enter the  labels for each region to be used for broadcasting the transforms"
				" for the objects (separated by space) : \n";
		std::cin >> choice;
		message.labels = choice;

		std::cout << "Send Configuration Settings? (y/n)  : ";
		std::cin >> choice;
		if(!choice.compare("y")){
			perceptionCommandPublisher.publish(message);
		}


		configure = false;
		std::cout << "Configure Again? (y/n)  : ";
		std::cin >> choice;
		if(!choice.compare("y")){
			configure = true;
		}
	}

}
