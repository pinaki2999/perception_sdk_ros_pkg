/*
 * ColorBasedROIExtractorHSV.cpp
 *
 *  Created on: Oct 5, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#include "ColorBasedROIExtractorHSV.h"


namespace BRICS_3D {

ColorBasedROIExtractorHSV::ColorBasedROIExtractorHSV() {
	this->max_h = 255;
	this->min_h	= 0;

	this->max_s = 255;
	this->min_s	= 0;

	this->max_v = 255;
	this->min_v	= 0;

}

double ColorBasedROIExtractorHSV::getMax_h() const
{
	return max_h;
}

double ColorBasedROIExtractorHSV::getMax_s() const
{
	return max_s;
}

double ColorBasedROIExtractorHSV::getMax_v() const
{
	return max_v;
}

double ColorBasedROIExtractorHSV::getMin_h() const
{
	return min_h;
}

double ColorBasedROIExtractorHSV::getMin_s() const
{
	return min_s;
}

double ColorBasedROIExtractorHSV::getMin_v() const
{
	return min_v;
}

void ColorBasedROIExtractorHSV::setMax_h(double max_h)
{
	this->max_h = max_h;
}

void ColorBasedROIExtractorHSV::setMax_s(double max_s)
{
	this->max_s = max_s;
}

void ColorBasedROIExtractorHSV::setMax_v(double max_v)
{
	this->max_v = max_v;
}

void ColorBasedROIExtractorHSV::setMin_h(double min_h)
{
	this->min_h = min_h;
}

void ColorBasedROIExtractorHSV::setMin_s(double min_s)
{
	this->min_s = min_s;
}

void ColorBasedROIExtractorHSV::setMin_v(double min_v)
{
	this->min_v = min_v;
}

ColorBasedROIExtractorHSV::~ColorBasedROIExtractorHSV() {}

void ColorBasedROIExtractorHSV::extractColorBasedROI(BRICS_3D::ColoredPointCloud3D &in_cloud,
		BRICS_3D::PointCloud3D &out_cloud){

	if(this->min_s == 0 && this->min_h == 0 && this->min_v == 0 && this->max_h == 255 &&
			this->max_s == 255 && this->max_v == 255) {
		//ToDo print error that the limits were not initialized
	}

	int cloudSize =	in_cloud.getSize();
	double temp_h, temp_s, temp_v, temp_r, temp_g, temp_b;
	char temp_c;
	bool passed;
	BRICS_3D::ColorSpaceConvertor colorConvertor;
	BRICS_3D::Point3D temp_point3D;
	out_cloud.getPointCloud()->clear();

	for (unsigned int i = 0; i < cloudSize; i++) {

		passed = false;
		//Getting the HSV values for the RGB points
		temp_c = in_cloud.getPointCloud()->data()[i].red;
		temp_r = atof( &temp_c );

		temp_c = in_cloud.getPointCloud()->data()[i].green;
		temp_g = atof( &temp_c );

		temp_c = in_cloud.getPointCloud()->data()[i].blue;
		temp_b = atof( &temp_c );

		colorConvertor.rgbToHsv(temp_r, temp_g, temp_b, temp_h, temp_s, temp_v);

		//Checking the values with the set limits
		if (temp_h < max_h && temp_h > min_h) {
			if (temp_s < min_s && temp_s > max_s) {
				passed=true;
			}
		}

		//Add to the out_cloud if the values are passed

		if(passed){
			temp_point3D.setX(in_cloud.getPointCloud()->data()[i].getX());
			temp_point3D.setY(in_cloud.getPointCloud()->data()[i].getY());
			temp_point3D.setZ(in_cloud.getPointCloud()->data()[i].getZ());
			out_cloud.addPoint(temp_point3D);
		}

	}

}



}
