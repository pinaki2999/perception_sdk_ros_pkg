/*
 * PoseEstimation6D.h
 *
 *  Created on: Nov 3, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#ifndef POSEESTIMATION6D_H_
#define POSEESTIMATION6D_H_

//ros headers
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"


//standard headers
#include <iostream>

namespace BRICS_3D {

class PoseEstimation6D {

	/**
	 * Maximum number of objects to be searched for.
	 */
	int maxNoOfObjects;



public:
	PoseEstimation6D();
	virtual ~PoseEstimation6D();
    int getMaxNoOfObjects() const;
    void setMaxNoOfObjects(int maxNoOfObjects);

	/**
	 * Callback function for Kinect data.
	 * @param cloud input cloud from Kinect
	 */
	void kinectCloudCallback(const sensor_msgs::PointCloud2 &cloud);
};

}

#endif /* POSEESTIMATION6D_H_ */
