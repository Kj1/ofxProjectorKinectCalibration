/*
 WRAPPER CLASS FOR CALIBRATING 3D CAMERAS TO PROJECTORS
 ofxopenni
 */

#pragma once

#include "RGBDCamCalibWrapper.h"
#include "ofxOpenNI.h"

using namespace cv;
using namespace ofxCv;

class RGBDCamCalibWrapperOfxOpenNi : public RGBDCamCalibWrapper {
    
private:
    ofxCvColorImage colorImage;
    ofxOpenNI * backend;
    bool ready;
    
public:
    
    //Casting your backend
    void setup(void * _backend){
        ready = false;
        
        //cast the backend
        backend = static_cast <ofxOpenNI *>(_backend);
        
        //check if the backend is connected & capturing calibrated video
        if(!backend->isContextReady()){
            ofLog(OF_LOG_ERROR, "Please open the kinect prior to setting the RGBDcamWrapper");
            return;
        }
        if(!backend->getRegister()){
            ofLog(OF_LOG_ERROR, "Please enable the grabs setRegister(true) for ofxOpenNI");
            return;
        }
        
        //allocate buffers
        colorImage.allocate(backend->getWidth(), backend->getHeight());
        
        ready = true;
    }
    
    //getting the calibrated color/depth image
    ofxCvColorImage getColorImageCalibrated(){
        if(!ready){
            ofLog(OF_LOG_ERROR, "Please open the kinect prior to setting the RGBDcamWrapper");
            return colorImage;
        }
        ofPixels p = backend->getImagePixels();
        colorImage.setFromPixels(p);
        return colorImage;
    }
    
    //coordinate getters
    ofPoint getWorldFromRgbCalibrated(ofPoint p){
        if(!ready){
            ofLog(OF_LOG_ERROR, "Please open the kinect prior to setting the RGBDcamWrapper");
            return ofPoint(0, 0);
        }
        return backend->cameraToWorld(p);
    }
    
};
