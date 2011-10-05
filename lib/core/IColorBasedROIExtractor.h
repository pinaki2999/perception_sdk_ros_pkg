/*
 * IColorBasedROIExtractor.h
 *
 *  Created on: Oct 5, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#ifndef ICOLORREGIONEXTRACTOR_H_
#define ICOLORREGIONEXTRACTOR_H_

#include <core/ColoredPointCloud3D.h>
#include "ColorSpaceConvertor.h"
#include <stdlib.h>
#include <math.h>

namespace BRICS_3D {

class IColorBasedROIExtractor {
public:
	IColorBasedROIExtractor();
	virtual ~IColorBasedROIExtractor();

	/**
	 * Extracts subset of input point cloud based on color-properties
	 * @param in_cloud Input pointcloud (Colored)
	 * @param out_cloud Extracted Subset (Color Information Discarded)
	 */
	void extractColorBasedROI(BRICS_3D::ColoredPointCloud3D &in_cloud, BRICS_3D::PointCloud3D &out_cloud);
};

}

#endif /* ICOLORREGIONEXTRACTOR_H_ */
