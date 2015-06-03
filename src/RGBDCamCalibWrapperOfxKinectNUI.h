/*
 WRAPPER CLASS FOR CALIBRATING 3D CAMERAS TO PROJECTORS
 ofxKinectNui: Kinect Microsoft SDK 1.5+
*/

#pragma once

#include "RGBDCamCalibWrapper.h"

#include <Shlobj.h>
#include "ofxKinectNui.h"

using namespace cv;
using namespace ofxCv;

class RGBDCamCalibWrapperOfxKinectNUI : public RGBDCamCalibWrapper {

private:
	ofxKinectNui*				backend;
	ofxCvColorImage				colorImage;
	bool						ready;

public:    
	
	//Casting your backend
	 void setup(void* _backend){
		 ready = false;

		 //cast the backend
		 backend = static_cast<ofxKinectNui*>(_backend);
		 
		 //check if the backend is connected & capturing calibrated video
		 if(!backend->isConnected() || !backend->isOpened() || !backend->isInited()) {
			ofLog(OF_LOG_ERROR,"Please open the kinect prior to setting the RGBDcamWrapper");
			return;			 
		 }
		 if(!backend->grabsCalibratedVideo()) {
			ofLog(OF_LOG_ERROR,"Please enable the grabs calibrated video setting in for ofxKinectNui");
			return;			 
		 }

		 //allocate buffers
		 colorImage.allocate(backend->getDepthResolutionWidth(),backend->getDepthResolutionHeight());
		 
		 ready = true;
	 }

	//getting the calibrated color/depth image
	ofxCvColorImage	getColorImageCalibrated() {
		if (!ready) {
			ofLog(OF_LOG_ERROR,"Please open the kinect prior to setting the RGBDcamWrapper");
			return colorImage;
		}
		ofPixels p = backend->getCalibratedVideoPixels();
		colorImage.setFromPixels(p);
		return colorImage;
	}

	//coordinate getters
	ofPoint	getWorldFromRgbCalibrated(ofPoint p) {
		if (!ready) {
			ofLog(OF_LOG_ERROR,"Please open the kinect prior to setting the RGBDcamWrapper");
			return ofPoint(0,0);
		}
		return backend->getWorldCoordinateFor(p.x,p.y);
	}

};
