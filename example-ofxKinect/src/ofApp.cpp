#include "ofApp.h"

using namespace ofxCv;
using namespace cv;

//--------------------------------------------------------------
void ofApp::setup(){
    
    // OF basics
    ofSetFrameRate(60);
    ofBackground(0);
	ofSetVerticalSync(true);
    ofSetLogLevel(OF_LOG_VERBOSE);
    ofSetLogLevel("ofThread", OF_LOG_WARNING);
    ofSetWindowTitle("Kinect Projector Calibration demo");
	
	// settings and defaults
	projectorWidth = 1920;
	projectorHeight = 1080;
	enableCalibration = false;
	enableTestmode	  = false;
	
    // kinect: configure your backend
    kinect.init();
    kinect.setRegistration(true);
    kinect.open();
    int kinectWidth = kinect.getWidth();
    int kinectHeight = kinect.getHeight();
    
    // allocations
    kinectColorImage.allocate(kinectWidth, kinectHeight);
    kinectDepthImage.allocate(kinectWidth, kinectHeight);
	thresholdedKinect.allocate(kinectWidth, kinectHeight, OF_IMAGE_GRAYSCALE);
    
    // contourFinder config
	chessboardThreshold = 60;
	lowThresh = 0.2;
	highThresh = 1.0;
	contourFinder.setMinAreaRadius(12);
	contourFinder.setThreshold(3);
	contourFinder.getTracker().setPersistence(25);
	contourFinder.getTracker().setMaximumDistance(150);
	contourFinder.setFindHoles(false);
	contourFinder.setInvert(false);
	
    // make the wrapper (to make calibration independant of the drivers...)
    kinectWrapper = new RGBDCamCalibWrapperOfxKinect();
    kinectWrapper->setup(&kinect);
    kinectProjectorCalibration.setup(kinectWrapper, projectorWidth, projectorHeight);
    
	// some default config
	kinectProjectorCalibration.chessboardSize = 100;
	kinectProjectorCalibration.chessboardColor = 175;
	kinectProjectorCalibration.setStabilityTimeInMs(500);
	maxReprojError = 2.0f;
	
    // sets the output
    kinectProjectorOutput.setup(kinectWrapper, projectorWidth, projectorHeight);
    //kinectProjectorOutput.load("kinectProjector.yml");
    
    // setup the second window
    secondWindow.setup("Projector", 500, 50, projectorWidth, projectorHeight, false);
    
    // setup the gui
    setupGui();
}

//--------------------------------------------------------------
void ofApp::update(){
    
	kinect.update();
	kinectColorImage.setFromPixels(kinect.getPixelsRef());
	convertColor(kinectColorImage, thresholdedKinect, CV_RGB2GRAY);
	ofxCv::threshold(thresholdedKinect, chessboardThreshold);
	thresholdedKinect.update();
	kinectDepthImage.setFromPixels(kinect.getDepthPixelsRef());
	
    // if calibration active
    if (enableCalibration) {

        // do a very-fast check if chessboard is found
        bool stableBoard = kinectProjectorCalibration.doFastCheck();
        
        //if it is stable, add it.
        if (stableBoard) {
            kinectProjectorCalibration.addCurrentFrame();
        }
    }
    
    // if the test mode is activated, the settings are loaded automatically (see gui function)
    if (enableTestmode) {
        
        // find our contours in the label image
		if (highThresh != 1.0) cvThreshold(kinectDepthImage.getCvImage(), kinectDepthImage.getCvImage(), highThresh * 255, 255, CV_THRESH_TOZERO_INV);
		if (lowThresh != 0.0) cvThreshold(kinectDepthImage.getCvImage(), kinectDepthImage.getCvImage(), lowThresh * 255, 255, CV_THRESH_TOZERO);
		contourFinder.findContours(kinectDepthImage);
		
    }
    // update the gui labels with the result of our calibraition
    guiUpdateLabels();
}

//--------------------------------------------------------------
void ofApp::draw(){
    
	ofBackground(0);
	ofSetColor(255);
	
	ofTranslate(320,0);
	ofDrawBitmapString("Kinect Input",0,20);
	kinectColorImage.draw(0,40,320,240);
	//ofCircle(160, 160, 5); // Kinect center
	
	//if calibrating, then we draw our fast check results here
	if (enableCalibration) {
		
		// draw the chessboard to our second window
		secondWindow.begin();
		ofClear(0);
		kinectProjectorCalibration.drawChessboard();
		secondWindow.end();
		
		ofTranslate(0,40);
		vector<ofVec2f> pts = kinectProjectorCalibration.getFastCheckResults();
		for (int i = 0; i < pts.size(); i++) {
			ofSetColor(0,255,0);
			ofFill();
			ofCircle(pts[i].x/2.0, pts[i].y / 2.0, 5);
			ofNoFill();
		}
		ofTranslate(0,-40);
		
		ofSetColor(255);
		
		// draw our calibration gui
		ofDrawBitmapString("Thresholded Kinect Input",320+20,20);
		//kinectProjectorCalibration.drawChessboardDebug(320+20,40,320,240);
		thresholdedKinect.draw(320+20,40,320,240);
		
		ofDrawBitmapString("Processed Input",0,20+240+20+40);
		kinectProjectorCalibration.drawProcessedInputDebug(0,20+240+40+40,320,240);
		
		ofDrawBitmapString("Reprojected points",320+20,20+240+20+40);
		//kinectProjectorCalibration.drawReprojectedPointsDebug(320+20,20+240+40+40,320,240);
	} else if (enableTestmode) {
		ofDrawBitmapString("Grayscale Image",0,20+240+20+40);
		kinectDepthImage.draw(0,20+240+40+40,320,240);
		
		ofDrawBitmapString("Contours",320+20,20+240+20+40);
		
		ofTranslate(320+20, 20+240+20+40+20);
		ofScale(0.5, 0.5);
		contourFinder.draw();
		ofScale(2.0, 2.0);
		ofTranslate(-(320+20), -(20+240+20+40+20));
		
		//draw the calibrated contours to our second window
		secondWindow.begin();
		ofClear(0);
		ofSetColor(255, 190, 70);
		
		for (int i = 0; i < contourFinder.size(); i++) {
			
			ofPolyline blobContour = contourFinder.getPolyline(i);
			if(!blobContour.isClosed()){
				blobContour.close();
			}
			
			ofBeginShape();
			for (int j = 0; j < blobContour.size() - 1; j++) {
				ofPoint currVertex = kinectProjectorOutput.projectFromDepthXY(blobContour[j]);
				ofVertex(currVertex.x, currVertex.y);
				
			}
			ofEndShape();
			
		}
		ofSetColor(255);
		secondWindow.end();
	} else {
		secondWindow.begin();
		ofBackground(255);
		secondWindow.end();
	}
	ofTranslate(-320,0);
	ofSetColor(255);
	
    //gui->draw();
}

//--------------------------------------------------------------
void ofApp::exit(){
	
    kinect.close();
    delete gui;
	delete guiImageSettings;
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}

//--------------------------------------------------------------
void ofApp::setupGui() {
    
    float dim = 16;
    float length = 300;
    
    gui = new ofxUISuperCanvas("- ofxKinect example -");
    gui->setColorBack(ofColor(51, 55, 56, 200));
	
	gui->addSpacer(length, 2);
	gui->addWidgetDown(new ofxUILabelButton("Select another Kinect", false));
	gui->addWidgetDown(new ofxUIMinimalSlider("Kinect tilt angle", -27, 27, 0.0, length, dim));
	
    gui->addSpacer(length, 2);
    gui->addWidgetDown(new ofxUILabel("Calibration instructions", OFX_UI_FONT_MEDIUM));
    gui->addSpacer(length, 2);
    gui->addWidgetDown(new ofxUILabel("sdf", "1) Move 2nd window to projector", OFX_UI_FONT_SMALL));
    gui->addWidgetDown(new ofxUILabel("sdf", "2) Set it to fullscreen", OFX_UI_FONT_SMALL));
    gui->addWidgetDown(new ofxUILabel("sdf", "3) Activate calibration", OFX_UI_FONT_SMALL));
    gui->addWidgetDown(new ofxUILabel("sdf", "4) Hold flat board ", OFX_UI_FONT_SMALL));
    gui->addWidgetDown(new ofxUILabel("sdf", "5) Keep still during some time to capture", OFX_UI_FONT_SMALL));
    gui->addWidgetDown(new ofxUILabel("sdf", "6) Make 15 captures, then clean the dataset", OFX_UI_FONT_SMALL));
	
	gui->addWidgetDown(new ofxUILabel(" ", OFX_UI_FONT_MEDIUM));
	gui->addWidgetDown(new ofxUILabel("Mode", OFX_UI_FONT_MEDIUM));
	gui->addSpacer(length, 2);
	gui->addWidgetDown(new ofxUIToggle("Activate calibration mode", &enableCalibration, dim, dim));
	gui->addWidgetDown(new ofxUIToggle("Activate test mode", &enableTestmode, dim, dim));
	
	gui->addWidgetDown(new ofxUILabel(" ", OFX_UI_FONT_MEDIUM));
	gui->addWidgetDown(new ofxUILabel("Calibration", OFX_UI_FONT_MEDIUM));
	gui->addSpacer(length, 2);
	gui->addWidgetDown(new ofxUISlider("Max reproj. error", 0.50, 5.00, &maxReprojError, length, dim));
	gui->addWidgetDown(new ofxUIButton("Clean dataset (remove > max reproj error)", false, dim, dim));
	gui->addWidgetDown(new ofxUIButton("Clear dataset (remove all)", false, dim, dim));
	gui->addWidgetDown(new ofxUILabel(" ", OFX_UI_FONT_LARGE));
	gui->addWidgetDown(new ofxUILabel("errorLabel", "Avg Reprojection error: 0.0", OFX_UI_FONT_SMALL));
	gui->addWidgetDown(new ofxUILabel("capturesLabel", "Number of captures: 0", OFX_UI_FONT_SMALL));
	
	gui->addWidgetDown(new ofxUILabel(" ", OFX_UI_FONT_MEDIUM));
	gui->addSpacer(length, 2);
	gui->addWidgetDown(new ofxUIFPS(OFX_UI_FONT_MEDIUM));
	
    gui->autoSizeToFitWidgets();
    
    ofAddListener(gui->newGUIEvent,this,&ofApp::guiEvent);
	
	///////////////////////////////////////////////
	
	guiImageSettings = new ofxUISuperCanvas("Image settings");
	guiImageSettings->setColorBack(ofColor(51, 55, 56, 200));
	
	guiImageSettings->addSpacer(length, 2);
	guiImageSettings->addWidgetDown(new ofxUISlider("Chessboard size", 0, 255, &kinectProjectorCalibration.chessboardSize, length, dim));
	guiImageSettings->addWidgetDown(new ofxUISlider("Chessboard luminosity", 0, 255, &kinectProjectorCalibration.chessboardColor, length, dim));
	guiImageSettings->addWidgetDown(new ofxUISlider("Chessboard detec. threshold", 0, 255, &chessboardThreshold, length, dim));
	guiImageSettings->addWidgetDown(new ofxUIToggle("Adaptative threshold", &kinectProjectorCalibration.b_CV_CALIB_CB_ADAPTIVE_THRESH, dim, dim));
	guiImageSettings->addWidgetDown(new ofxUIToggle("Normalize the image", &kinectProjectorCalibration.b_CV_CALIB_CB_NORMALIZE_IMAGE, dim, dim));
	guiImageSettings->addSpacer(length, 2);
	guiImageSettings->addWidgetDown(new ofxUI2DPad("Chessboard position offset", ofVec2f(-1.0, 1.0), ofVec2f(-1.0, 1.0), ofVec2f(0, 0), length, length * 3 / 4));
	
	guiImageSettings->addSpacer(length, 2);
	guiImageSettings->addWidgetDown(new ofxUILabel("lbl", "Tracking options", OFX_UI_FONT_MEDIUM));
	guiImageSettings->addWidgetDown(new ofxUIRangeSlider("Thresholds", 0.0, 1.0, &lowThresh, &highThresh, length, dim));
	
	guiImageSettings->autoSizeToFitWidgets();
	guiImageSettings->setPosition(1024 - guiImageSettings->getRect()->getWidth(), 768 - guiImageSettings->getRect()->getHeight());
	//guiImageSettings->toggleMinified();
	ofAddListener(guiImageSettings->newGUIEvent,this,&ofApp::guiEvent);
    
}

//--------------------------------------------------------------
void ofApp::guiUpdateLabels() {
    ofxUILabel* l;
    l = (ofxUILabel*) gui->getWidget("errorLabel");
    l->setLabel("Avg Reprojection error: " + ofToString(kinectProjectorCalibration.getReprojectionError(), 2));
    
    l = (ofxUILabel*) gui->getWidget("capturesLabel");
    l->setLabel("Number of captures: " + ofToString(kinectProjectorCalibration.getDatabaseSize()));
}

//--------------------------------------------------------------
void ofApp::guiEvent(ofxUIEventArgs &e) {
	string name = e.widget->getName();
	int kind = e.widget->getKind();
	if (name == "Clean dataset (remove > max reproj error)") {
		ofxUIButton* b = (ofxUIButton*)e.widget;
		if(b->getValue()) kinectProjectorCalibration.clean(maxReprojError);
	} else if (name == "Clear dataset (remove all)") {
		ofxUIButton* b = (ofxUIButton*)e.widget;
		if(b->getValue()) kinectProjectorCalibration.clearAll();
	} else if (name == "Select another Kinect") {
		ofxUIButton* b = (ofxUIButton*) e.widget;
		if(b->getValue()) {
			int nextID = kinect.nextAvailableId();
			if (nextID != -1 && nextID != kinect.getDeviceId()) {
				kinect.close();
				ofLog(OF_LOG_VERBOSE, "Kinect %i is about to be opened", nextID);
				kinect.open(nextID);
			} else {
				ofLog(OF_LOG_WARNING, "No other Kinect is currently available");
			}
		}
	} else if (name == "Activate test mode") {
		ofxUIButton* b = (ofxUIButton*)e.widget;
		if(b->getValue()) {
			enableCalibration = false;
			kinectProjectorOutput.load("kinectProjector.yml");
		}
	} else if (name == "Activate calibration mode") {
		ofxUIButton* b = (ofxUIButton*)e.widget;
		if(b->getValue())  enableTestmode = false;
	} else if (name == "Kinect tilt angle") {
		ofxUISlider *slider = (ofxUISlider *) e.widget;
		kinect.setCameraTiltAngle(slider->getValue());
	} else if (name == "Chessboard position offset") {
		ofxUI2DPad *pad = (ofxUI2DPad *) e.widget;
		ofVec2f trans = pad->getValue();
		kinectProjectorCalibration.setChessboardTranslation((trans.x * projectorWidth) - projectorWidth / 2, (trans.y * projectorHeight) - projectorHeight / 2);
	}
}
