/** OFXKINECTPROJECTORCALIBRATION **/
/** work in progress, not even beta! **/
/** Kj1, www.hangaar.net **/

#pragma once

#include "ofMain.h"

#include "ofxOpenCv.h"
#include "ofxUI.h"

// for some reason has to be included after ofxUI and ofxOpenNI
#include "ofxCv.h"

#include "ofxOpenNI.h"
#include "ofxFensterManager.h"

#include "RGBDCamCalibWrapperOfxOpenNi.h"
#include "ofxKinectProjectorCalibration.h"

#include "SecondWindow.h"


using namespace cv;
//using namespace ofxCv;

class testApp : public ofBaseApp 
{
    public:

        void setup();

        void update();
        void draw();
        void exit();

        void keyPressed(int key);
        void mouseDragged(int x, int y, int button);
        void mousePressed(int x, int y, int button);
        void mouseReleased(int x, int y, int button);
        void windowResized(int w, int h);

	private:
    
		//kinect & the wrapper
    
		ofxOpenNI				kinect;
		ofxCvColorImage				kinectCalibratedColorImage;
		ofxCvGrayscaleImage			kinectLabelImageGray;
    
		RGBDCamCalibWrapper*		kinectWrapper;
    
		//calibration
		KinectProjectorCalibration	kinectProjectorCalibration;
		bool						enableCalibration;
		ofxCvContourFinder			contourFinder;

		//output
		KinectProjectorOutput		kinectProjectorOutput;
		bool						enableTestmode;

        float threshold;
    
		//settings
		int projectorWidth;
		int projectorHeight;

		//gui
		void setupGui();
	    ofxUICanvas *gui;
		void guiEvent(ofxUIEventArgs &e);   
		void guiUpdateLabels();   

		//second window
		void setupSecondWindow();
		SecondWindow secondWindow;
		ofFbo secondWindowFbo;

};
