/*
 * PoseEstimationICP.h
 *
 *  Created on: Oct 28, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#ifndef POSEESTIMATIONICP_H_
#define POSEESTIMATIONICP_H_


#include "core/PointCloud3D.h"
#include "util/PCLTypecaster.h"


#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/registration.h>
#include <pcl/common/transform.h>


//ToDo Add comments

namespace BRICS_3D {

namespace SDK {

class PoseEstimationICP {
private:

	BRICS_3D::PointCloud3D *objectModel;

	float distance;

	float transformationEpsilon;

	int maxIterations;

	float fitnessScore;

	Eigen::Matrix4f finalTransformation;

public:
	PoseEstimationICP();
	virtual ~PoseEstimationICP();

	void estimatePose(BRICS_3D::PointCloud3D *inCloud, BRICS_3D::PointCloud3D *outCloud);
    Eigen::Matrix4f getFinalTransformation() const
    {
        return finalTransformation;
    }

    float getFitnessScore() const
    {
        return fitnessScore;
    }

    int getMaxIterations() const
    {
        return maxIterations;
    }

    float getTransformationEpsilon() const
    {
        return transformationEpsilon;
    }

    void setFinalTransformation(Eigen::Matrix4f finalTransformation)
    {
        this->finalTransformation = finalTransformation;
    }

    void setFitnessScore(float fitnessScore)
    {
        this->fitnessScore = fitnessScore;
    }

    void setMaxIterations(int maxIterations)
    {
        this->maxIterations = maxIterations;
    }

    void setTransformationEpsilon(float transformationEpsilon)
    {
        this->transformationEpsilon = transformationEpsilon;
    }

    BRICS_3D::PointCloud3D* getObjectModel() const
    {
        return objectModel;
    }

    void setObjectModel(BRICS_3D::PointCloud3D *objectModel)
    {
        this->objectModel = objectModel;
    }

    float getDistance() const
    {
        return distance;
    }

    void setDistance(float distance)
    {
        this->distance = distance;
    }

};

}

}

#endif /* POSEESTIMATIONICP_H_ */
