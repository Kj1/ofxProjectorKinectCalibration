/*
 WRAPPER CLASS FOR CALIBRATING 3D CAMERAS TO PROJECTORS

 Meant to make the driver backend decoupled from the implementation. 

 Currently supported backends: 
 - ofxKinectNui: Kinect Microsoft SDK 1.5+
                                                     
 Plannned support                                    
 - ofxKinect: Kinect libfreenect drivers            
 - ofxOpenNi: Kinect openNI 1.5 drivers 

*/


#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"

using namespace cv;
using namespace ofxCv;

class RGBDCamCalibWrapper {

public:    
	
	//setting up your backend
	virtual void				setup(void* backend) = 0;

	//getting the calibrated color/depth image
	virtual ofxCvColorImage		getColorImageCalibrated(bool mirrorHoriz = false, bool mirrorVert = false)= 0;    //////

	//coordinate getters
	virtual ofPoint				getWorldFromRgbCalibrated(ofPoint p, bool mirrorHoriz = false, bool mirrorVert = false)= 0; ///

};
