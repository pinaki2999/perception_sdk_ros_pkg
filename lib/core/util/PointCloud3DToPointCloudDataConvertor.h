/*
 * PointCloud3DToPointCloudDataConvertor.h
 *
 *  Created on: Oct 7, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#ifndef POINTCLOUD3DTOPOINTCLOUDDATACONVERTOR_H_
#define POINTCLOUD3DTOPOINTCLOUDDATACONVERTOR_H_




namespace BRICS_3D {

template <class PointT>
class PointCloud3DToPointCloudDataConvertor : public  IPointCloud3DConvertor {

	typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;

private:

	/**
	 * The destination Pointcloud
	 */
	PointCloudPtr destinationCloudPtr;

public:

	PointCloud3DToPointCloudDataConvertor();
	virtual ~PointCloud3DToPointCloudDataConvertor();


	/**
	 * Converts the point-cloud in BRICS_3D library format to the output-cloud type.
	 * The output cloud will be pointed by "destinationCloud"
	 * The output cloud type will be implementation specific.
	 */
	void convert();


	/**
	 * Converts the pointcloud of in the output format to the BRICS_3D format
	 * The resultant cloud will be pointed by "sourceCloud"
	 */
	void revert();


	/**
	 * Set the boost-shared pointer for the destination pointcloud in pcl format
	 * @param inCloudPtr boost-shared pointer for the destination pointcloud
	 */
	inline void setDestinationCloud(PointCloudPtr inCloudPtr){
		this->destinationCloudPtr = inCloudPtr;
	}


	/**
	 * Set the boost-shared pointer for the destination pointcloud in pcl format
	 * @return boost-shared pointer for the destination pointcloud
	 */
	inline PointCloudPtr getDestinationCloud(){
			return this->destinationCloudPtr;
	}


};

}

#endif /* POINTCLOUD3DTOPOINTCLOUDDATACONVERTOR_H_ */
