/*
 * ModelFitting.h
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

#include "IterativeClosestPoint.h"
#include "util/SimplePointCloudGeneratorCube.h"
#include "util/PCLTypecaster.h"
#include "core/PointCloud3D.h"
#include "algorithm/featureExtraction/Centroid3D.h"
#include "core/HomogeneousMatrix44.h"

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/registration.h>
#include <pcl/common/transform.h>

#include <Eigen/Geometry>


namespace BRICS_3D {

class ModelFitting {

	BRICS_3D::SDK::IterativeClosestPoint poseEstimatorICP;

	BRICS_3D::PointCloud3D *cube2D;
	BRICS_3D::PointCloud3D *cube3D;

	BRICS_3D::SimplePointCloudGeneratorCube cubeModelGenerator;

	BRICS_3D::PCLTypecaster pclTypecaster;

	float reliableScoreThreshold;

	ros::Publisher *modelPublisher;

	float bestScore;


	void calculateHomogeneousMatrix(float xRot,float yRot,
			float zRot, float xtrans, float ytrans, float ztrans, Eigen::Matrix4f  &homogeneousMatrix, bool inDegrees){
		if(inDegrees){
			float PI = 3.14159265;
			xRot = xRot*(PI/180);
			yRot *= yRot*(PI/180);
			zRot *= zRot*(PI/180);
		}
		/*
		 * layout:
		 * 0 4 8  12
		 * 1 5 9  13
		 * 2 6 10 14
		 * 3 7 11 15
		 */


		homogeneousMatrix[3]=0;
		homogeneousMatrix[7]=0;
		homogeneousMatrix[11]=0;
		homogeneousMatrix[15]=1;

	    //translation
	    homogeneousMatrix[12]=xtrans;
		homogeneousMatrix[13]=ytrans;
		homogeneousMatrix[14]=ztrans;

		//rotation
		homogeneousMatrix[0] = cos(yRot)*cos(zRot);
		homogeneousMatrix[1] = cos(yRot)*sin(zRot);
		homogeneousMatrix[2] = -sin(yRot);

		homogeneousMatrix[4] = -cos(xRot)*sin(zRot) + sin(xRot)*sin(yRot)*cos(zRot);
		homogeneousMatrix[5] = cos(xRot)*cos(zRot) + sin(xRot)*sin(yRot)*sin(zRot);
		homogeneousMatrix[6] = sin(xRot)*cos(yRot);

		homogeneousMatrix[8] = sin(xRot)*sin(zRot) + cos(xRot)*sin(yRot)*cos(zRot);
		homogeneousMatrix[9] = -sin(xRot)*cos(zRot) + cos(xRot)*sin(yRot)*sin(zRot);
		homogeneousMatrix[10]= cos(xRot)*cos(yRot);


	}

public:
	ModelFitting();
	virtual ~ModelFitting();
	void kinectCloudCallback(const sensor_msgs::PointCloud2 &cloud);

	void setModelPublisher(ros::Publisher *pub){
		this->modelPublisher = pub;
	}



};

}

#endif /* POSEESTIMATION6DEXAMPLE_H_ */
