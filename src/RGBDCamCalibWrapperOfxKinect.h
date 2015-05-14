/*
 WRAPPER CLASS FOR CALIBRATING 3D CAMERAS TO PROJECTORS
 ofxKinect : libfreenect drivers, now part of OpenFrameworks core.
*/

#pragma once

#include "RGBDCamCalibWrapper.h"
#include "ofxKinect.h"

using namespace cv;
using namespace ofxCv;

class RGBDCamCalibWrapperOfxKinect : public RGBDCamCalibWrapper {

private:
	ofxCvColorImage colorImage;
    ofxKinect * backend;
    bool ready;

public:    
	
	// Casting your backend
	void setup(void* _backend){
		ready = false;
        
        // cast the backend
        backend = static_cast <ofxKinect *>(_backend);
        
        // check if the backend is connected & capturing calibrated video
        if(!backend->isConnected()){
            ofLog(OF_LOG_ERROR, "Please open the kinect prior to setting the RGBDcamWrapper");
            return;
        }

        // allocate buffers
        colorImage.allocate(backend->getWidth(), backend->getHeight());
        
        ready = true;
	}

	// getting the calibrated color/depth image
	ofxCvColorImage	getColorImageCalibrated() {
		if (!ready) {
			ofLog(OF_LOG_ERROR,"Please open the kinect prior to setting the RGBDcamWrapper");
			return colorImage;
		}
		colorImage.setFromPixels(backend->getPixelsRef());
		return colorImage;
	}

	// coordinate getters
	ofPoint	getWorldFromRgbCalibrated(ofPoint p) {
		if (!ready) {
			ofLog(OF_LOG_ERROR,"Please open the kinect prior to setting the RGBDcamWrapper");
			return ofPoint(0,0);
		}
		return backend->getWorldCoordinateAt(p.x, p.y);
	}

};
