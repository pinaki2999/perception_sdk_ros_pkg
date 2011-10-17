/*
 * EuclideanClusterExtractor.h
 *
 *  Created on: Oct 14, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#ifndef EUCLIDEANCLUSTEREXTRACTION_H_
#define EUCLIDEANCLUSTEREXTRACTION_H_

#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "ros/publisher.h"
#include "pcl_ros/point_cloud.h"
#include <tf/transform_broadcaster.h>

#include <stdlib.h>
#include <stdio.h>
#include <vector>

#include "segmentation/EuclideanClusterExtraction.h"
#include "features/Centroid3DEstimation.h"
#include "util/PCLTypecaster.h"

//Todo add the comments
namespace BRICS_3D {

class EuclideanClusterExtractor {
private:

	ros::Publisher *extractedClusterPublisher;

	BRICS_3D::PCLTypecaster pclTypecaster;
	BRICS_3D::EucledeanClusterExtraction euclideanClusterExtractor;
	BRICS_3D::Centroid3DEstimation centroid3DEstimator;
	int maxNoOfObjects;

public:
	EuclideanClusterExtractor();
	virtual ~EuclideanClusterExtractor();

	void kinectCloudCallback(const sensor_msgs::PointCloud2 &cloud);


    void setExtractedClusterPublisher(ros::Publisher *extractedClusterPublisher);
    ros::Publisher *getExtractedClusterPublisher() const;

    int getMaxNoOfObjects() const;
    void setMaxNoOfObjects(int maxNoOfObjects);

    inline void setMinClusterSize(int size){
    	this->euclideanClusterExtractor.setMinClusterSize(size);
    }


    inline int getMinCLusterSize(){
    	return this->euclideanClusterExtractor.getMinClusterSize();
    }

    inline void setMaxClusterSize(int size){
        	this->euclideanClusterExtractor.setMaxClusterSize(size);
        }

    inline int getMaxClusterSize(){
    	return this->euclideanClusterExtractor.getMaxClusterSize();
    }

    inline void setClusterTolerance(float tolerance){
        	this->euclideanClusterExtractor.setClusterTolerance(tolerance);
        }

    inline float getClusterTolerance(){
     return	this->euclideanClusterExtractor.getClusterTolerance();
    }

    void initializeExtractor(int maxNoObjects, ros::Publisher *extractedClusterPublisher,
    		int minClusterSize, int maxClusterSize, float clusterTolerance);

};

}

#endif /* EUCLIDEANCLUSTEREXTRACTION_H_ */
