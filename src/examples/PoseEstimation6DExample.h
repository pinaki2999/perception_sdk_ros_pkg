/*
 * PoseEstimation6DExample.h
 *
 *  Created on: Oct 28, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#ifndef POSEESTIMATION6DEXAMPLE_H_
#define POSEESTIMATION6DEXAMPLE_H_

#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "ros/publisher.h"
#include "pcl_ros/point_cloud.h"

#include "PoseEstimationICP.h"
#include "util/SimplePointCloudGeneratorCube.h"
#include "util/PCLTypecaster.h"
#include "core/PointCloud3D.h"
namespace BRICS_3D {

class PoseEstimation6DExample {

	BRICS_3D::SDK::PoseEstimationICP poseEstimatorICP;

	BRICS_3D::PointCloud3D *cube2D;
	BRICS_3D::PointCloud3D *cube3D;

	BRICS_3D::SimplePointCloudGeneratorCube cubeModelGenerator;

	BRICS_3D::PCLTypecaster pclTypecaster;

	float reliableScoreThreshold;

	ros::Publisher *modelPublisher;
public:
	PoseEstimation6DExample();
	virtual ~PoseEstimation6DExample();
	void kinectCloudCallback(const sensor_msgs::PointCloud2 &cloud);

	void setModelPublisher(ros::Publisher *pub){
		this->modelPublisher = pub;
	}

};

}

#endif /* POSEESTIMATION6DEXAMPLE_H_ */
