/*
 * EuclideanClusterExtraction.h
 *
 *  Created on: Oct 14, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#ifndef EUCLIDEANCLUSTEREXTRACTION_H_
#define EUCLIDEANCLUSTEREXTRACTION_H_

namespace BRICS_3D {

class EuclideanClusterExtraction {
private:

	ros::Publisher *extractedClusterPublisher;

	int maxNoOfObjects;

public:
	EuclideanClusterExtraction();
	virtual ~EuclideanClusterExtraction();

	void kinectCloudCallback(const sensor_msgs::PointCloud2 &cloud);
	ros::Publisher* getExtractedRegionPublisher() const;
    void setExtractedClusterPublisher(ros::Publisher *extractedClusterPublisher);
};

}

#endif /* EUCLIDEANCLUSTEREXTRACTION_H_ */
