#include "ofApp.h"

using namespace ofxCv;
using namespace cv;

//--------------------------------------------------------------
void ofApp::setup(){
    
    // settings and defaults
    projectorWidth = 1920;
    projectorHeight = 1080;
    enableCalibration = false;
    enableTestmode	  = true;
    
    // OF basics
    ofSetFrameRate(60);
    ofBackground(100);
    ofSetLogLevel(OF_LOG_VERBOSE);
    ofSetLogLevel("ofThread", OF_LOG_WARNING);
    ofSetWindowTitle("Kinect Projector Calibration demo");
    
    // kinect: configure your backend
    kinect.init();
    kinect.setRegistration(true);
    kinect.open();
    int kinectWidth = kinect.getWidth();
    int kinectHeight = kinect.getHeight();
    
    // allocations
    kinectColorImage.allocate(kinectWidth, kinectHeight);
    kinectDepthImage.allocate(kinectWidth, kinectHeight);
    
    // contourFinder config
    threshold = 220;
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
    kinectProjectorCalibration.setStabilityTimeInMs(500);
    maxReprojError = 2.0f;
    
    // sets the output
    kinectProjectorOutput.setup(kinectWrapper, projectorWidth, projectorHeight);
    //kinectProjectorOutput.load("kinectProjector.yml");
    
    // setup the second window
    secondWindow.setup("Projector", 50, 50, projectorWidth, projectorHeight, false);
    
    // setup the gui
    setupGui();
}

//--------------------------------------------------------------
void ofApp::update(){
    
    kinect.update();
    kinectColorImage.setFromPixels(kinect.getPixelsRef());
    kinectDepthImage.setFromPixels(kinect.getDepthPixelsRef());
    
    /*
    ofPixels& depthPixels = kinectDepthImage.getPixelsRef();
    ofShortPixels& rawDepthPixels = kinect.getRawDepthPixelsRef();
    unsigned char p;
    const short minDepth = 450;
    const short maxDepth = 2000;
    for (int i=0; i<640*480; i++) {
        short s = rawDepthPixels[i];
        char x = (s < minDepth || s > maxDepth) ? 255 : powf(s - minDepth, 0.7f);
        depthPixels[i] = 255 - x;
    }
    kinectDepthImage.flagImageChanged();
     */
    
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
        kinectDepthImage.threshold(threshold, false);
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
        ofDrawBitmapString("Chessboard (2nd screen)",320+20,20);
        kinectProjectorCalibration.drawChessboardDebug(320+20,40,320,240);
        
        ofDrawBitmapString("Processed Input",0,20+240+20+40);
        kinectProjectorCalibration.drawProcessedInputDebug(0,20+240+40+40,320,240);
        
        ofDrawBitmapString("Reprojected points",320+20,20+240+20+40);
        kinectProjectorCalibration.drawReprojectedPointsDebug(320+20,20+240+40+40,320,240);
    }
    
    if (enableTestmode) {
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
    }
    ofTranslate(-320,0);
    ofSetColor(255);	
    
    //gui->draw();
}

//--------------------------------------------------------------
void ofApp::exit(){
    
    kinect.close();
    delete gui;
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
    gui->addWidgetDown(new ofxUILabel("Chessboard settings", OFX_UI_FONT_MEDIUM));
    gui->addSpacer(length, 2);
    gui->addWidgetDown(new ofxUIMinimalSlider("Board size", 0, 255, &kinectProjectorCalibration.chessboardSize, length, dim));
    gui->addWidgetDown(new ofxUIMinimalSlider("Board luminosity", 0, 255, &kinectProjectorCalibration.chessboardColor, length, dim));
    gui->addWidgetDown(new ofxUIMinimalSlider("Kinect depth threshold", 0, 255, &threshold, length, dim));
    gui->addWidgetDown(new ofxUIMinimalSlider("Max reproj. error", 0.50, 5.00, &maxReprojError, length, dim));
    
    gui->addWidgetDown(new ofxUIToggle("CV_CALIB_CB_ADAPTIVE_THRESH",&kinectProjectorCalibration.b_CV_CALIB_CB_ADAPTIVE_THRESH, dim, dim));
    gui->addWidgetDown(new ofxUIToggle("CV_CALIB_CB_NORMALIZE_IMAGE",&kinectProjectorCalibration.b_CV_CALIB_CB_NORMALIZE_IMAGE, dim, dim));
    
    gui->addWidgetDown(new ofxUILabel(" ", OFX_UI_FONT_MEDIUM));
    gui->addWidgetDown(new ofxUILabel("Calibration settings", OFX_UI_FONT_MEDIUM));
    gui->addSpacer(length, 2);
    gui->addWidgetDown(new ofxUIToggle("CV_CALIB_FIX_PRINCIPAL_POINT",&kinectProjectorCalibration.b_CV_CALIB_FIX_PRINCIPAL_POINT, dim, dim));
    gui->addWidgetDown(new ofxUIToggle("CV_CALIB_FIX_ASPECT_RATIO",&kinectProjectorCalibration.b_CV_CALIB_FIX_ASPECT_RATIO, dim, dim));
    gui->addWidgetDown(new ofxUIToggle("CV_CALIB_ZERO_TANGENT_DIST",&kinectProjectorCalibration.b_CV_CALIB_ZERO_TANGENT_DIST, dim, dim));
    gui->addWidgetDown(new ofxUIToggle("CV_CALIB_FIX_K1",&kinectProjectorCalibration.b_CV_CALIB_FIX_K1, dim, dim));
    gui->addWidgetDown(new ofxUIToggle("CV_CALIB_FIX_K2",&kinectProjectorCalibration.b_CV_CALIB_FIX_K2, dim, dim));
    gui->addWidgetDown(new ofxUIToggle("CV_CALIB_FIX_K3",&kinectProjectorCalibration.b_CV_CALIB_FIX_K3, dim, dim));
    gui->addWidgetDown(new ofxUIToggle("CV_CALIB_RATIONAL_MODEL",&kinectProjectorCalibration.b_CV_CALIB_RATIONAL_MODEL, dim, dim));
    
    gui->addWidgetDown(new ofxUILabel(" ", OFX_UI_FONT_MEDIUM));
    gui->addWidgetDown(new ofxUILabel("Calibration", OFX_UI_FONT_MEDIUM));
    gui->addSpacer(length, 2);
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
        
    } else if (name == "Activate test mode") {
        ofxUIButton* b = (ofxUIButton*)e.widget;
        if(b->getValue()) {
            enableCalibration = false;
            kinectProjectorOutput.load("kinectProjector.yml");
        }
    }
    else if (name == "Activate calibration mode") {
        ofxUIButton* b = (ofxUIButton*)e.widget;
        if(b->getValue())  enableTestmode = false;
    }	
}
