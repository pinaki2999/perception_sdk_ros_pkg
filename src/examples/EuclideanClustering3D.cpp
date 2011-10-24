/*
 * EuclideanClustering3D.cpp
 *
 *  Created on: Oct 24, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#include "EuclideanClustering3D.h"

namespace BRICS_3D {

EuclideanClustering3D::EuclideanClustering3D() {
	// TODO Auto-generated constructor stub

}

EuclideanClustering3D::~EuclideanClustering3D() {
	// TODO Auto-generated destructor stub
}

void EuclideanClustering3D::extractClusters(BRICS_3D::PointCloud3D *inCloud,
		std::vector<BRICS_3D::PointCloud3D*> *extractedClusters){

	int k = inCloud->getSize();
	BRICS_3D::NearestNeighborANN nearestneighborSearch;
	BRICS_3D::Point3D querryPoint3D;
	vector<int> neighborIndices;

	nearestneighborSearch.setData(inCloud);
	nearestneighborSearch.setMaxDistance(this->clusterTolerance);

	// Create a bool vector of processed point indices, and initialize it to false
	std::vector<bool> processed (inCloud->getSize(), false);
	std::vector<int> nn_indices;
	std::vector<float> nn_distances;

	// Process all points in the indices vector
	for (size_t i = 0; i < inCloud->getSize(); ++i)
	{
		if (processed[i])
			continue;

		std::vector<int> seed_queue;
		int sq_idx = 0;
		seed_queue.push_back (i);

		processed[i] = true;

		while (sq_idx < (int)seed_queue.size ())
		{

			// Search for sq_idx
			neighborIndices.clear();
			nearestneighborSearch.findNearestNeighbors(&querryPoint3D, &neighborIndices, k);

			//if (!tree->radiusSearch (seed_queue[sq_idx], tolerance, nn_indices, nn_distances))
			if(neighborIndices.size()==0)
			{
				sq_idx++;
				continue;
			}

			for (size_t j = 0; j < neighborIndices.size(); ++j)             // nn_indices[0] should be sq_idx
			{
				if (processed[neighborIndices[j]])                             // Has this point been processed before ?
					continue;

				// Perform a simple Euclidean clustering
				seed_queue.push_back (neighborIndices[j]);
				processed[neighborIndices[j]] = true;
			}

			sq_idx++;
		}

		// If this queue is satisfactory, add to the clusters
		if (seed_queue.size () >= minClusterSize && seed_queue.size () <= maxClusterSize)
		{

			extractedClusters->resize(extractedClusters->size()+1);

			//			pcl::PointIndices r;
			//			r.indices.resize (seed_queue.size ());
			for (size_t j = 0; j < seed_queue.size (); ++j) {

				BRICS_3D::Point3D *tempPoint =  new BRICS_3D::Point3D(inCloud->getPointCloud()->data()[seed_queue[j]].getX(),
						inCloud->getPointCloud()->data()[seed_queue[j]].getY(),
						inCloud->getPointCloud()->data()[seed_queue[j]].getZ());

				(*extractedClusters)[extractedClusters->size()-1]->addPoint(tempPoint);

				delete tempPoint;

				//			r.indices[j] = seed_queue[j];
				//
				//			std::sort (r.indices.begin (), r.indices.end ());
				//			r.indices.erase (std::unique (r.indices.begin (), r.indices.end ()), r.indices.end ());
				//
				//			r.header = cloud.header;
				//			clusters.push_back (r);   // We could avoid a copy by working directly in the vector
			}
		}

	}
}
void EuclideanClustering3D::extractClusters(BRICS_3D::ColoredPointCloud3D *inCloud,
		std::vector<BRICS_3D::ColoredPointCloud3D*> *extractedClusters){
	printf("Nothing is implemented \n");

}

}
