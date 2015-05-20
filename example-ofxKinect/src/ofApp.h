#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"

#include "ofxUI.h"
#include "ofxSecondWindow.h"

#include "ofxKinect.h"
#include "RGBDCamCalibWrapperOfxKinect.h"
#include "ofxKinectProjectorCalibration.h"

#include "ofxXmlSettings.h"

using namespace cv;

class ofApp : public ofBaseApp{

	public:
		void setup();
    
		void update();
		void draw();
        void exit();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
    
    private:
        
        // kinect & the wrapper
        
        ofxKinect                   kinect;
        ofxCvColorImage				kinectColorImage;
        ofxCvGrayscaleImage			kinectDepthImage;
        
        RGBDCamCalibWrapper*		kinectWrapper;
        
        // calibration
        KinectProjectorCalibration	kinectProjectorCalibration;
        bool						enableCalibration;
        ofxCv::ContourFinder        contourFinder;
        
        // output
        KinectProjectorOutput		kinectProjectorOutput;
        bool						enableTestmode;
        
        float                       threshold;
        float                       maxReprojError;
    
        // settings
        int                         projectorWidth;
        int                         projectorHeight;
        
        // gui
        void setupGui();
        ofxUICanvas *               gui;
        void guiEvent(ofxUIEventArgs &e);
        void guiUpdateLabels();
        
        // second window
        void setupSecondWindow();
        ofxSecondWindow             secondWindow;
		
};
