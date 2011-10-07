/*
 * PCLTypeCaster.cpp
 *
 *  Created on: Oct 7, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#include "PCLTypecaster.h"

namespace BRICS_3D {

PCLTypecaster::PCLTypecaster() {}

PCLTypecaster::~PCLTypecaster() {}


void convertToPCLDataType(pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloudPtr,
		BRICS_3D::PointCloud3D* pointCloud3DPtr ){

	pclCloudPtr->width = pointCloud3DPtr->getSize();
	pclCloudPtr->height = 1;
	pclCloudPtr->points.resize( pclCloudPtr->width * pclCloudPtr->height );

	for (unsigned int i =0 ; i<pointCloud3DPtr->getSize() ; i++){
		pclCloudPtr->points[i].x = pointCloud3DPtr->getPointCloud()->data()[i].getX();
		pclCloudPtr->points[i].y = pointCloud3DPtr->getPointCloud()->data()[i].getY();
		pclCloudPtr->points[i].z = pointCloud3DPtr->getPointCloud()->data()[i].getZ();
	}
}



void convertToBRICS3DDataTyoe(pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloudPtr,
		BRICS_3D::PointCloud3D* pointCloud3DPtr ){

	pointCloud3DPtr->getPointCloud()->resize(pclCloudPtr->size());

	for (unsigned int i =0 ; i < pclCloudPtr->size()  ; i++){
			pointCloud3DPtr->getPointCloud()->data()[i].setX(pclCloudPtr->points[i].x);
			pointCloud3DPtr->getPointCloud()->data()[i].setY(pclCloudPtr->points[i].y);
			pointCloud3DPtr->getPointCloud()->data()[i].setZ(pclCloudPtr->points[i].z);
		}
}




/**
 * Converts from XYZRGB-cloud format of BRICS_3D to XYZRGB-cloud format of PCL
 * @param pclCloudPtr		point cloud data in PCL format
 * @param pointCloud3DPtr	point cloud data in BRICS_3D format
 */
void convertToPCLDataType(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloudPtr,
		BRICS_3D::ColoredPointCloud3D* pointCloud3DPtr ){

	BRICS_3D::ColorSpaceConvertor colorSpaceConvertor;

	pclCloudPtr->width = pointCloud3DPtr->getSize();
	pclCloudPtr->height = 1;
	pclCloudPtr->points.resize( pclCloudPtr->width * pclCloudPtr->height );

	float rgbVal24bit=0;

	for (unsigned int i =0 ; i<pointCloud3DPtr->getSize() ; i++){
		pclCloudPtr->points[i].x = pointCloud3DPtr->getPointCloud()->data()[i].getX();
		pclCloudPtr->points[i].y = pointCloud3DPtr->getPointCloud()->data()[i].getY();
		pclCloudPtr->points[i].z = pointCloud3DPtr->getPointCloud()->data()[i].getZ();
		colorSpaceConvertor.rgbToRGB24Bit(&rgbVal24bit,
					pointCloud3DPtr->getPointCloud()->data()[i].getR(),
					pointCloud3DPtr->getPointCloud()->data()[i].getG(),
					pointCloud3DPtr->getPointCloud()->data()[i].getB());
		pclCloudPtr->points[i].rgb = rgbVal24bit;
	}
}


/**
 * Converts from XYZRGB-cloud format of PCL to XYZRGB-cloud format of BRICS_3D
 * @param pclCloudPtr		point cloud data in PCL format
 * @param pointCloud3DPtr	point cloud datda in BRICS_3D format
 */
void convertToBRICS3DDataTyoe(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloudPtr,
		BRICS_3D::ColoredPointCloud3D* pointCloud3DPtr ){

	int r, g, b;
	BRICS_3D::ColorSpaceConvertor colorSpaceConvertor;

	for (unsigned int i =0 ; i < pclCloudPtr->size()  ; i++){
		pointCloud3DPtr->getPointCloud()->data()[i].setX(pclCloudPtr->points[i].x);
		pointCloud3DPtr->getPointCloud()->data()[i].setY(pclCloudPtr->points[i].y);
		pointCloud3DPtr->getPointCloud()->data()[i].setZ(pclCloudPtr->points[i].z);
		colorSpaceConvertor.rgb24bitToRGB(pclCloudPtr->points[i].rgb, &r, &g, &b);
		pointCloud3DPtr->getPointCloud()->data()[i].setR(r);
		pointCloud3DPtr->getPointCloud()->data()[i].setG(g);
		pointCloud3DPtr->getPointCloud()->data()[i].setB(b);
	}
}

}
