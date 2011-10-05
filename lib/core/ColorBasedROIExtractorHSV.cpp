/*
 * ColorBasedROIExtractorHSV.cpp
 *
 *  Created on: Oct 5, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#include "ColorBasedROIExtractorHSV.h"


namespace BRICS_3D {

ColorBasedROIExtractorHSV::ColorBasedROIExtractorHSV() {
	this->maxH = 255;
	this->minH	= 0;

	this->maxS = 255;
	this->minS	= 0;

	this->maxV = 255;
	this->minV	= 0;

}

double ColorBasedROIExtractorHSV::getMaxH() const
{
	return maxH;
}

double ColorBasedROIExtractorHSV::getMaxS() const
{
	return maxS;
}

double ColorBasedROIExtractorHSV::getMaxV() const
{
	return maxV;
}

double ColorBasedROIExtractorHSV::getMinH() const
{
	return minH;
}

double ColorBasedROIExtractorHSV::getMinS() const
{
	return minS;
}

double ColorBasedROIExtractorHSV::getMinV() const
{
	return minV;
}

void ColorBasedROIExtractorHSV::setMaxH(double maxH)
{
	this->maxH = maxH;
}

void ColorBasedROIExtractorHSV::setMaxS(double maxS)
{
	this->maxS = maxS;
}

void ColorBasedROIExtractorHSV::setMaxV(double maxV)
{
	this->maxV = maxV;
}

void ColorBasedROIExtractorHSV::setMinH(double minH)
{
	this->minH = minH;
}

void ColorBasedROIExtractorHSV::setMinS(double minS)
{
	this->minS = minS;
}

void ColorBasedROIExtractorHSV::setMinV(double minV)
{
	this->minV = minV;
}

ColorBasedROIExtractorHSV::~ColorBasedROIExtractorHSV() {}

void ColorBasedROIExtractorHSV::extractColorBasedROI(BRICS_3D::ColoredPointCloud3D *in_cloud,
		BRICS_3D::PointCloud3D *out_cloud){

	if(this->minS == 0 && this->minH == 0 && this->minV == 0 && this->maxH == 255 &&
			this->maxS == 255 && this->maxV == 255) {
		//ToDo print error that the limits were not initialized
	}

	int cloudSize =	in_cloud->getSize();
	double tempH, tempS, tempV, tempR, tempG, tempB;
	char tempChar;
	bool passed;
	BRICS_3D::ColorSpaceConvertor colorConvertor;
	BRICS_3D::Point3D tempPoint3D;
	out_cloud->getPointCloud()->clear();

	for (unsigned int i = 0; i < cloudSize; i++) {

		passed = false;
		//Getting the HSV values for the RGB points
		tempChar = in_cloud->getPointCloud()->data()[i].red;
		tempR = atof( &tempChar );

		tempChar = in_cloud->getPointCloud()->data()[i].green;
		tempG = atof( &tempChar );

		tempChar = in_cloud->getPointCloud()->data()[i].blue;
		tempB = atof( &tempChar );

		colorConvertor.rgbToHsv(tempR, tempG, tempB, &tempH, &tempS, &tempV);

		//Checking the values with the set limits
		if (tempH < maxH && tempH > minH) {
			if (tempS < minS && tempS > maxS) {
				passed=true;
			}
		}

		//Add to the out_cloud if the values are passed

		if(passed){
			tempPoint3D.setX(in_cloud->getPointCloud()->data()[i].getX());
			tempPoint3D.setY(in_cloud->getPointCloud()->data()[i].getY());
			tempPoint3D.setZ(in_cloud->getPointCloud()->data()[i].getZ());
			out_cloud->addPoint(tempPoint3D);
		}

	}

}



}
