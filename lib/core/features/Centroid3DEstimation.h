/*
 * Centroid3DEstimation.h
 *
 *  Created on: Oct 17, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#ifndef CENTROID3DESTIMATION_H_
#define CENTROID3DESTIMATION_H_

#include "core/PointCloud3D.h"
#include "core/ColoredPointCloud3D.h"

#include <Eigen/Dense>
namespace BRICS_3D {

class Centroid3DEstimation {
public:
	Centroid3DEstimation();
	virtual ~Centroid3DEstimation();


	/**
	 * Estimates 3D centroid of the input point cloud
	 * @param inCloud The input cloud of which the centroid will be calculated
	 * @return the calculated 3D centroid
	 */
	Eigen::Vector3d estmateCentroid(BRICS_3D::PointCloud3D *inCloud);


	/**
	 * Estimates 3D centroid of the input point cloud
	 * @param inCloud The input cloud of which the centroid will be calculated
	 * @return the calculated 3D centroid
	 */
	Eigen::Vector3d estmateCentroid(BRICS_3D::ColoredPointCloud3D *inCloud);

};

}

#endif /* CENTROID3DESTIMATION_H_ */
