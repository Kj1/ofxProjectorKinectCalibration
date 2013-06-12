/*
 WRAPPER CLASS FOR CALIBRATING 3D CAMERAS TO PROJECTORS
 ofxKinect
*/

#pragma once

#include "RGBDCamCalibWrapper.h"

using namespace cv;
using namespace ofxCv;

class RGBDCamCalibWrapperOfxKinect : public RGBDCamCalibWrapper {

private:
	ofxCvColorImage				colorImage;

public:    
	
	//Casting your backend
	 void setup(void* _backend){
	 }

	//getting the calibrated color/depth image
	ofxCvColorImage	getColorImageCalibrated() {
		return colorImage;
	}

	//coordinate getters
	ofPoint	getWorldFromRgbCalibrated(ofPoint p) {
		return ofPoint(0,0);
	}

};
