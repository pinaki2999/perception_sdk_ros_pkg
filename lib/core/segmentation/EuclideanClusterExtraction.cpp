/*
 * EuclideanClusterExtraction.cpp
 *
 *  Created on: Oct 14, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#include "EuclideanClusterExtraction.h"

namespace BRICS_3D {

EucledeanClusterExtraction::EucledeanClusterExtraction() {
	// TODO Auto-generated constructor stub

}

EucledeanClusterExtraction::~EucledeanClusterExtraction() {
	// TODO Auto-generated destructor stub
}

void EucledeanClusterExtraction::extractClusters(BRICS_3D::PointCloud3D *inCloud,
		std::vector<BRICS_3D::PointCloud3D*> *extractedClusters){

	BRICS_3D::PCLTypecaster pclTypecaster;

	//converting input cloud into PCL format
	pcl::PointCloud<pcl::PointXYZ>::Ptr inCloudPclPtr(new pcl::PointCloud<pcl::PointXYZ> ());
	pclTypecaster.convertToPCLDataType(inCloudPclPtr, inCloud);

	// Creating the KdTree object for the search method of the extraction
	pcl::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
	tree->setInputCloud (inCloudPclPtr);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclideanClusterExtractor;
	euclideanClusterExtractor.setClusterTolerance (this->clusterTolerance); // 2cm
	euclideanClusterExtractor.setMinClusterSize (this->minClusterSize);
	euclideanClusterExtractor.setMaxClusterSize (this->maxClusterSize);
	euclideanClusterExtractor.setSearchMethod (tree);
	euclideanClusterExtractor.setInputCloud(inCloudPclPtr);
	euclideanClusterExtractor.extract (cluster_indices);

	printf("Number of objects found: %d", cluster_indices.size());

	//Building up the pointclouds for corresponding clusters
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
		BRICS_3D::PointCloud3D tempPointCloud3D;
		tempPointCloud3D.getPointCloud()->clear();
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
			tempPointCloud3D.addPoint(new Point3D(	inCloudPclPtr->points[*pit].x,
													inCloudPclPtr->points[*pit].y,
													inCloudPclPtr->points[*pit].z) );
		}
		extractedClusters->push_back(&tempPointCloud3D);
	}
}


void EucledeanClusterExtraction::extractClusters(BRICS_3D::ColoredPointCloud3D *inCloud,
		std::vector<BRICS_3D::ColoredPointCloud3D*> *extractedClusters){

	BRICS_3D::PCLTypecaster pclTypecaster;
	BRICS_3D::ColorSpaceConvertor colorSpaceConvertor;
	//converting input cloud into PCL format
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr inCloudPclPtr(new pcl::PointCloud<pcl::PointXYZRGB> ());
	pclTypecaster.convertToPCLDataType(inCloudPclPtr, inCloud);

	// Creating the KdTree object for the search method of the extraction
	pcl::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
	tree->setInputCloud (inCloudPclPtr);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> euclideanClusterExtractor;
	euclideanClusterExtractor.setClusterTolerance (this->clusterTolerance); // 2cm
	euclideanClusterExtractor.setMinClusterSize (this->minClusterSize);
	euclideanClusterExtractor.setMaxClusterSize (this->maxClusterSize);
	euclideanClusterExtractor.setSearchMethod (tree);
	euclideanClusterExtractor.setInputCloud(inCloudPclPtr);
	euclideanClusterExtractor.extract (cluster_indices);

	printf("Number of objects found: %d", cluster_indices.size());

	//Building up the pointclouds for corresponding clusters
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
		BRICS_3D::ColoredPointCloud3D tempPointCloud3D;
		tempPointCloud3D.getPointCloud()->clear();
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){

			int rgbVal= *reinterpret_cast<int*>(&inCloudPclPtr->points[*pit].rgb);
			uint8_t r, g, b;
			colorSpaceConvertor.rgb24bitToRGB(rgbVal, &r, &g, &b);
			unsigned char red= *reinterpret_cast<unsigned char*>(&r);
			unsigned char green= *reinterpret_cast<unsigned char*>(&g);
			unsigned char blue= *reinterpret_cast<unsigned char*>(&b);
	 		tempPointCloud3D.addPoint(new ColoredPoint3D(new Point3D(inCloudPclPtr->points[*pit].x,
					inCloudPclPtr->points[*pit].y, inCloudPclPtr->points[*pit].z),
					red, green, blue));

		}
		extractedClusters->push_back(&tempPointCloud3D);
	}
}

}
