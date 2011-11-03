/*
 * ControlPerception.cpp
 *
 *  Created on: Nov 3, 2011
 *      Author: Pinaki Sunil Banerjee
 */

//ROS specific Headers
#include <ros/ros.h>
#include "perception_sdk_ros_pkg/Coordination.h"
#include "ros/publisher.h"
#include <iostream>
int main(int argc, char* argv[]){

	ros::init(argc, argv, "ControlPerception");
	ros::NodeHandle nh;

	ros::Publisher perceptionCommandPublisher = nh.advertise<perception_sdk_ros_pkg::Coordination>  ("/perceptionControl", 1);


	std::cout << "\n\t\tEnter 1 to pause the perception engine"
			"\n\t\tEnter 2 to resume the perception engine"
			"\n\t\tEnter 0 to exixt perception-engine controller\n\n";


	int choice = 5;
	perception_sdk_ros_pkg::Coordination message;

	while(choice != 0){
		std::cout << "Your command: \n";
		std::cin >> choice;

		if(choice == 1){
			//Pause
			message.command = "pause";
			perceptionCommandPublisher.publish(message);
		} else if(choice == 2) {
			//Resume
			message.command = "resume";
			perceptionCommandPublisher.publish(message);
		} else if(choice == 0) {
			//Resume
			exit(0);
		}else {

			std::cout << "\n\t\tEnter 1 to pause the perception engine"
						"\n\t\tEnter 2 to resume the perception engine"
						"\n\t\tEnter 0 to exixt perception-engine controller\n\n";
			choice = 5;
		}
	}

}
