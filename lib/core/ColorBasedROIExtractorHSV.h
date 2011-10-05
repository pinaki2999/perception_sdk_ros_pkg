/*
 * ColorBasedROIExtractorHSV.h
 *
 *  Created on: Oct 5, 2011
 *      Author: Pinaki Sunil Banerjee
 */

#ifndef COLORBASEDROIEXTRACTORHSV_H_
#define COLORBASEDROIEXTRACTORHSV_H_

#include "IColorBasedROIExtractor.h"

namespace BRICS_3D {

class ColorBasedROIExtractorHSV : public IColorBasedROIExtractor {

private:
	/**
	 * Maximum and minimum values for Hue
	 */
	double max_h;
	double min_h;

	/**
	 * Maximum and minimum values for Saturation
	 */
	double max_s;
	double min_s;

	/**
	 * Maximum and minimum values for Lightness(V)
	 */
	double max_v;
	double min_v;



public:


	ColorBasedROIExtractorHSV();


	virtual ~ColorBasedROIExtractorHSV();


	/**
	 * Extracts subset of input point cloud based on color-properties
	 * @param in_cloud Input pointcloud (Colored)
	 * @param out_cloud Extracted Subset (Color Information Discarded)
	 */
	void extractColorBasedROI(BRICS_3D::ColoredPointCloud3D &in_cloud, BRICS_3D::PointCloud3D &out_cloud);


	/**
	 *
	 * @return maximum Hue allowed
	 */
    double getMax_h() const;


    /**
     *
     * @return maximum Saturation allowed
     */
    double getMax_s() const;


    /**
     *
     * @return maximum Lightness allowed
     */
    double getMax_v() const;


    /**
     *
     * @return minimum Hue allowed
     */
    double getMin_h() const;


    /**
     *
     * @return minimum Saturation allowed
     */
    double getMin_s() const;


    /**
     *
     * @return minimum Lightness allowed
     */
    double getMin_v() const;


    /**
     *
     * @param max_h maximum Hue allowed
     */
    void setMax_h(double max_h);


    /**
     *
     * @param max_s maximum Saturation allowed
     */
    void setMax_s(double max_s);


    /**
     *
     * @param max_v maximum Lightness allowed
     */
    void setMax_v(double max_v);


    /**
     *
     * @param min_h minimum Hue allowed
     */
    void setMin_h(double min_h);


    /**
     *
     * @param min_s minimum Saturation allowed
     */
    void setMin_s(double min_s);


    /**
     *
     * @param min_v minimum Lightness allowed
     */
    void setMin_v(double min_v);



};

}

#endif /* COLORBASEDROIEXTRACTORHSV_H_ */
