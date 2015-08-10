/** OFXKINECTPROJECTORCALIBRATION **/
/** work in progress, not even beta! **/
/** Kj1, www.hangaar.net **/

#include "testApp.h"

using namespace cv;
using namespace ofxCv;


void testApp::setup() 
{
	//settings and defaults
	projectorWidth = 1280;
	projectorHeight = 800;
	enableCalibration = false;
	enableTestmode	  = true;

	//OF basics
    ofSetFrameRate(60);
    ofBackground(100);
	ofSetLogLevel(OF_LOG_VERBOSE);
	ofSetWindowTitle("Kinect Projector Calibration demo");

	//kinect: configure your backend
	ofxKinectNui::InitSetting initSetting;
	initSetting.grabVideo = true;
	initSetting.grabDepth = true;
	initSetting.grabAudio = false;
	initSetting.grabLabel = false;
	initSetting.grabSkeleton = false;
	initSetting.grabCalibratedVideo = true;
	initSetting.grabLabelCv = false;
	initSetting.videoResolution = NUI_IMAGE_RESOLUTION_640x480;
	initSetting.depthResolution = NUI_IMAGE_RESOLUTION_640x480;
	kinect.init(initSetting);
	kinect.open();
	kinect.setAngle(-20);

	//allocate some images
	kinectCalibratedColorImage.allocate(kinect.getDepthResolutionWidth(), kinect.getDepthResolutionHeight());
	kinectLabelImageGray.allocate(kinect.getDepthResolutionWidth(), kinect.getDepthResolutionHeight());
	
	//make the wrapper (to make calibration independant of the drivers...)
	RGBDCamCalibWrapper* kinectWrapper = new RGBDCamCalibWrapperOfxKinectNUI();
	kinectWrapper->setup(&kinect);
	kinectProjectorCalibration.setup(kinectWrapper, projectorWidth, projectorHeight);
	
	//sets the output alinger
	kinectProjectorOutput.setup(kinectWrapper, projectorWidth, projectorHeight);	
	//kinectProjectorOutput.load("kinectProjector.yml");

	//setup our second window
	setupSecondWindow();
	secondWindowFbo.allocate(projectorWidth, projectorHeight);
	secondWindow.setFbo(&secondWindowFbo);
	
	//setup the gui
	setupGui();
}

void testApp::update() 
{	
	kinect.update();
	kinectCalibratedColorImage.setFromPixels(kinect.getCalibratedVideoPixels());
	kinectLabelImageGray.setFromPixels(kinect.getDepthPixels());
	
	//if calibration active
	if (enableCalibration) {
		//draw the chessboard to our second window
		secondWindowFbo.begin();
			ofClear(0);
			kinectProjectorCalibration.drawChessboard();
		secondWindowFbo.end();

		//do a very-fast check if chessboard is found
		bool stableBoard = kinectProjectorCalibration.doFastCheck();	

		//if it is stable, add it.
		if (stableBoard) {
			kinectProjectorCalibration.addCurrentFrame();	
		}		
	}

	//if the test mode is activated, the settings are loaded automatiically (see gui function)
	// kinectProjectorOutput.load("kinectProjector.yml");
	if (enableTestmode) {

		//find our contours in the label image
		kinectLabelImageGray.threshold(50,false);
		contourFinder.findContours(kinectLabelImageGray, 100, 640*480, 4, false, true);
		


		//draw the calibrated contours to our second window
		secondWindowFbo.begin();
			ofClear(0);
			ofSetColor(255);
			ofSetLineWidth(3);

			for (int i = 0; i < contourFinder.nBlobs; i++) {
				for (int j = 0; j < contourFinder.blobs[i].nPts - 1; j++) {
					//we get our original points
					ofPoint originalFrom = contourFinder.blobs[i].pts[j];
					ofPoint originalTo = contourFinder.blobs[i].pts[j+1];
					
					//we project from our depth xy to projector space
					ofPoint projectedFrom = kinectProjectorOutput.projectFromDepthXY(originalFrom);
					ofPoint projectedTo =   kinectProjectorOutput.projectFromDepthXY(originalTo);
					
					//todo soon method with opengl matrixes (more performant)

					//for some reason it mirros, dunno why
					projectedFrom.x = 1280-projectedFrom.x;
					projectedTo.x = 1280-projectedTo.x;
					ofLine(projectedFrom, projectedTo);
				}
			}
		secondWindowFbo.end();
		
	}

	//update the gui labels with the result of our calibraition
	guiUpdateLabels() ;  
}

void testApp::draw() 
{    
    ofBackground(0);	
	ofSetColor(255);

	ofTranslate(320,0);
	ofDrawBitmapString("Kinect Input",0,20);
	kinectCalibratedColorImage.draw(0,40,320,240);

	//if calibrating, then we draw our fast check results here
	if (enableCalibration) {
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

		//draw our calibration gui
		ofDrawBitmapString("Chessboard (2nd screen)",320+20,20);
		kinectProjectorCalibration.drawChessboardDebug(320+20,40,320,240);

		ofDrawBitmapString("Processed Input",0,20+240+20+40);
		kinectProjectorCalibration.drawProcessedInputDebug(0,20+240+40+40,320,240);

		ofDrawBitmapString("Reprojected points",320+20,20+240+20+40);
		kinectProjectorCalibration.drawReprojectedPointsDebug(320+20,20+240+40+40,320,240);
	}	
	if (enableTestmode) {
		ofDrawBitmapString("Grayscale Image",0,20+240+20+40);
		kinectLabelImageGray.draw(0,20+240+40+40,320,240);
		
		ofDrawBitmapString("Contours",320+20,20+240+20+40);
	}

	ofTranslate(-320,0);
	ofSetColor(255);	

	//because we use ofxFenster, we need to draw the gui manually
	gui->draw();
}

void testApp::exit() 
{	
	kinect.close();
}

void testApp::keyPressed (int key) 
{
	if (key == 'f') {
		secondWindow.toggleFullScreen();
	}
}

void testApp::mousePressed(int x, int y, int button)
{
}

void testApp::mouseDragged(int x, int y, int button)
{
}

void testApp::mouseReleased(int x, int y, int button)
{
}

void testApp::windowResized(int w, int h)
{
}



void testApp::setupGui() {
	
	float dim = 16; 
	float xInit = OFX_UI_GLOBAL_WIDGET_SPACING; 
    float length = 300-xInit; 

	gui = new ofxUICanvas(0, 0, length+xInit, ofGetHeight()); 
	gui->setDrawBack(true);
	gui->setTheme(OFX_UI_THEME_HACKER);
		
	
	gui->addWidgetDown(new ofxUILabel("CALIBRATION INSTRUCTIONS", OFX_UI_FONT_LARGE)); 
    gui->addSpacer(length-xInit, 2);
	gui->addWidgetDown(new ofxUILabel("sdf", "1) Move 2nd window to projector", OFX_UI_FONT_SMALL));
	gui->addWidgetDown(new ofxUILabel("sdf", "2) Press f to go fullscreen", OFX_UI_FONT_SMALL));
	gui->addWidgetDown(new ofxUILabel("sdf", "3) Activate calibration", OFX_UI_FONT_SMALL));
	gui->addWidgetDown(new ofxUILabel("sdf", "4) Hold flat board so it contains", OFX_UI_FONT_SMALL));
	gui->addWidgetDown(new ofxUILabel("sdf", "   contains the projected chessboard", OFX_UI_FONT_SMALL));
	gui->addWidgetDown(new ofxUILabel("sdf", "   and can be seen by the kinect", OFX_UI_FONT_SMALL));
	gui->addWidgetDown(new ofxUILabel("sdf", "   Adjust size slider if needed", OFX_UI_FONT_SMALL));
	gui->addWidgetDown(new ofxUILabel("sdf", "5) Keep still for 2 seconds to make capture", OFX_UI_FONT_SMALL));
	gui->addWidgetDown(new ofxUILabel("sdf", "6) Make 15 captures, then clean the highest errors", OFX_UI_FONT_SMALL));

	
	gui->addWidgetDown(new ofxUILabel(" ", OFX_UI_FONT_LARGE)); 
	gui->addWidgetDown(new ofxUILabel("Chessboard settings", OFX_UI_FONT_LARGE)); 
    gui->addSpacer(length-xInit, 2);
	gui->addWidgetDown(new ofxUIBiLabelSlider(length,0,1,&kinectProjectorCalibration.chessboardSize,"boardsize","Small","Large"));
	gui->addWidgetDown(new ofxUIBiLabelSlider(length,0,255,&kinectProjectorCalibration.chessboardColor,"boardColor","dark","light"));
	gui->addWidgetDown(new ofxUIToggle("CV_CALIB_CB_ADAPTIVE_THRESH",&kinectProjectorCalibration.b_CV_CALIB_CB_ADAPTIVE_THRESH, dim, dim));
	gui->addWidgetDown(new ofxUIToggle("CV_CALIB_CB_NORMALIZE_IMAGE",&kinectProjectorCalibration.b_CV_CALIB_CB_NORMALIZE_IMAGE, dim, dim));
	
	gui->addWidgetDown(new ofxUILabel(" ", OFX_UI_FONT_LARGE)); 
	gui->addWidgetDown(new ofxUILabel("Calibration settings", OFX_UI_FONT_LARGE)); 
    gui->addSpacer(length-xInit, 2);
	gui->addWidgetDown(new ofxUIToggle("CV_CALIB_FIX_PRINCIPAL_POINT",&kinectProjectorCalibration.b_CV_CALIB_FIX_PRINCIPAL_POINT, dim, dim));
	gui->addWidgetDown(new ofxUIToggle("CV_CALIB_FIX_ASPECT_RATIO",&kinectProjectorCalibration.b_CV_CALIB_FIX_ASPECT_RATIO, dim, dim));
	gui->addWidgetDown(new ofxUIToggle("CV_CALIB_ZERO_TANGENT_DIST",&kinectProjectorCalibration.b_CV_CALIB_ZERO_TANGENT_DIST, dim, dim));
	gui->addWidgetDown(new ofxUIToggle("CV_CALIB_FIX_K1",&kinectProjectorCalibration.b_CV_CALIB_FIX_K1, dim, dim));
	gui->addWidgetDown(new ofxUIToggle("CV_CALIB_FIX_K2",&kinectProjectorCalibration.b_CV_CALIB_FIX_K2, dim, dim));
	gui->addWidgetDown(new ofxUIToggle("CV_CALIB_FIX_K3",&kinectProjectorCalibration.b_CV_CALIB_FIX_K3, dim, dim));
	gui->addWidgetDown(new ofxUIToggle("CV_CALIB_RATIONAL_MODEL",&kinectProjectorCalibration.b_CV_CALIB_RATIONAL_MODEL, dim, dim));
 
	gui->addWidgetDown(new ofxUILabel(" ", OFX_UI_FONT_LARGE)); 
	gui->addWidgetDown(new ofxUILabel("Calibration", OFX_UI_FONT_LARGE));
    gui->addSpacer(length-xInit, 2);
	gui->addWidgetDown(new ofxUIToggle("Activate calibration mode", &enableCalibration, dim, dim));
	gui->addWidgetDown(new ofxUIButton("Clean dataset (remove > 2 rpr error)", false, dim, dim));
	gui->addWidgetDown(new ofxUIButton("Clean dataset (remove all)", false, dim, dim));
	gui->addWidgetDown(new ofxUILabel(" ", OFX_UI_FONT_LARGE)); 
	gui->addWidgetDown(new ofxUILabel("errorLabel", "Avg Reprojection error: 0.0", OFX_UI_FONT_SMALL));	
	gui->addWidgetDown(new ofxUILabel("capturesLabel", "Number of captures: 0", OFX_UI_FONT_SMALL));
	
	gui->addWidgetDown(new ofxUILabel(" ", OFX_UI_FONT_LARGE)); 
	gui->addWidgetDown(new ofxUILabel("Test", OFX_UI_FONT_LARGE));
    gui->addSpacer(length-xInit, 2);
	gui->addWidgetDown(new ofxUIToggle("Activate test mode", &enableTestmode, dim, dim));
	gui->addWidgetDown(new ofxUIFPS(OFX_UI_FONT_MEDIUM));

	ofAddListener(gui->newGUIEvent,this,&testApp::guiEvent);

}

void testApp::guiUpdateLabels() {
	ofxUILabel* l;
	
	l = (ofxUILabel*) gui->getWidget("errorLabel");
	l->setLabel("Avg Reprojection error: " + ofToString(kinectProjectorCalibration.getReprojectionError(), 2));
	
	l = (ofxUILabel*) gui->getWidget("capturesLabel");
	l->setLabel("Number of captures: " + ofToString(kinectProjectorCalibration.getDatabaseSize()));
}

void testApp::guiEvent(ofxUIEventArgs &e)
{
	string name = e.widget->getName(); 
	int kind = e.widget->getKind(); 
	if (name == "Clean dataset (remove > 2 rpr error)") { 
		ofxUIButton* b = (ofxUIButton*)e.widget;
		if(b->getValue()) kinectProjectorCalibration.clean();
	
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

void testApp::setupSecondWindow()
{
	secondWindow.setup();
	ofxFenster* win = ofxFensterManager::get()->createFenster(projectorWidth,projectorHeight);
	win->addListener(&secondWindow);
	win->setWindowTitle("Projector Window");
	secondWindow.setHandle(win);	
}
