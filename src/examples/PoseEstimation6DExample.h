/*
 * PoseEstimation6DExample.h
 *
 *  Created on: Oct 28, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#ifndef POSEESTIMATION6DEXAMPLE_H_
#define POSEESTIMATION6DEXAMPLE_H_

#include "PoseEstimationICP.h"

namespace BRICS_3D {

class PoseEstimation6DExample {
public:
	PoseEstimation6DExample();
	virtual ~PoseEstimation6DExample();

	void kinectCloudCallback(const sensor_msgs::PointCloud2 &cloud);
};

}

#endif /* POSEESTIMATION6DEXAMPLE_H_ */
