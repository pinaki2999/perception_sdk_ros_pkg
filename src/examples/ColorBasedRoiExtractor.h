/*
 * ColorBasedRoiExtractor.h
 *
 *  Created on: Oct 13, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#ifndef COLORBASEDROIEXTRACTOR_H_
#define COLORBASEDROIEXTRACTOR_H_

#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "ros/publisher.h"

#include "pcl/filters/passthrough.h"
#include "pcl/io/pcd_io.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/PointIndices.h"
#include "pcl/filters/extract_indices.h"

#include "filtering/ColorBasedROIExtractorHSV.h"
#include "util/ColorSpaceConvertor.h"
#include "util/PCLTypecaster.h"
#include "core/ColoredPointCloud3D.h"

namespace BRICS_3D {



class ColorBasedRoiExtractor {
private:

	ros::Publisher *extractedRegionPublisher;
	BRICS_3D::ColorBasedROIExtractorHSV hsvBasedRoiExtractor;
	BRICS_3D::ColorSpaceConvertor colorSpaceConvertor;
	BRICS_3D::PCLTypecaster pclTypeCaster;

public:
	ColorBasedRoiExtractor();
	virtual ~ColorBasedRoiExtractor();

	void kinectCloudCallback(const sensor_msgs::PointCloud2 &cloud);

	void initializeLimits(float minLimitH, float maxLimitH, float minLimitS, float maxLimitS);

	ros::Publisher* getExtractedRegionPublisher() const;
    void setExtractedRegionPublisher(ros::Publisher *extractedRegionPublisher);



};

}

#endif /* COLORBASEDROIEXTRACTOR_H_ */
