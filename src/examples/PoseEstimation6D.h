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


#include "algorithm/filtering/ColorBasedROIExtractorHSV.h"
#include "util/PCLTypecaster.h"
#include "core/ColoredPointCloud3D.h"
#include "EuclideanClustering.h"
#include "algorithm/featureExtraction/Centroid3D.h"

//standard headers
#include <iostream>
#include <vector>
namespace BRICS_3D {

class PoseEstimation6D {

	/**
	 * Maximum number of objects to be searched for.
	 */
	int maxNoOfObjects;

	/**
	 * The label that will be used to publish the transforms
	 */
	std::string regionLabel;

	/**
	 * object for extracting ROIs based on HSV-color space limits
	 */
	BRICS_3D::ColorBasedROIExtractorHSV hsvBasedRoiExtractor;

	/**
	 * Utility object for type-casting data between BRICS_3D and PCL
	 */
	BRICS_3D::PCLTypecaster pclTypecaster;

	/**
	 * Indicates if the color based ROI extractor is intialized with proper limits
	 */
	bool initializedRoiExtractor;

	/**
	 * Object for extracting euclidean clusters
	 */
	BRICS_3D::SDK::EuclideanClustering euclideanClusterExtractor;

	/**
	 * Object for estimating 3D centroids of the estimated object clusters
	 */
	BRICS_3D::Centroid3D centroid3DEstimator;


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


	/**
	 * Initializes the HSV color-space based ROI extractor with Hue ans Saturation limits
	 * @param minLimitH	minimum Hue value
	 * @param maxLimitH maximum Hue value
	 * @param minLimitS minimum saturation value
	 * @param maxLimitS maximum saturation value
	 *
	 */
	void initializeLimits(float minLimitH, float maxLimitH, float minLimitS, float maxLimitS);

	void initializeClusterExtractor(int minClusterSize, int maxClusterSize, float clusterTolerance);


	std::string getRegionLabel() const;
    void setRegionLabel(std::string regionLabel);
};

}

#endif /* POSEESTIMATION6D_H_ */
