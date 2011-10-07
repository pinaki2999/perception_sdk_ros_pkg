/**
 * ColorSpaceConvertor.cpp
 *
 *  Created on: Oct 5, 2011
 *  Author: PinakiSunilBanerjee
 */

#include "ColorSpaceConvertor.h"

#include <stdint.h>
namespace BRICS_3D {


ColorSpaceConvertor::ColorSpaceConvertor() {
	 this-> epsilon = 0.00001;
}


ColorSpaceConvertor::~ColorSpaceConvertor() {}


void ColorSpaceConvertor::rgbToHsv(double r, double g, double b, double *h, double *s, double *v){

	double max, min;

	r = r/255.0;
	g = g/255.0;
	b = b/255.0;

	max = maxVal(r, g, b);
	min = minVal(r, g, b);

	*v = max;

	if(fabs(*v)<= epsilon){
		*s= (*v-min)/(*v);
	} else {
		s=0;
	}
	if(fabs(*v-r) <= epsilon){
		*h= (g-b)*60.0/(*s);
	} else if(fabs((*v)-g) <= epsilon) {
		*h= 180.0+(b-r)*60.0/(*s);
	} else if(fabs((*v)-b) <= epsilon) {
		*h=240+(r - g)*60.0/(*s);
	}

	if((*h)<0.0)	(*h) = (*h)+360.0;

	*h /= 2.0;
	*s *= 255.0;
	*v *= 255.0;

}

void ColorSpaceConvertor::rgb24bitToRGB(float rgb24Bit,  int *r, int *g, int *b){

		int rgb_val = *reinterpret_cast<int*>(&rgb24Bit);
		*r = ((rgb_val >> 16) & 0xff);
		*g = ((rgb_val >> 8) & 0xff);
		*b = (rgb_val & 0xff);
}


void ColorSpaceConvertor::rgbToRGB24Bit(float *rgb24Bit, int r, int g,
		int b){
		*rgb24Bit = (r << 16) | (g << 8) | b;
}

}
