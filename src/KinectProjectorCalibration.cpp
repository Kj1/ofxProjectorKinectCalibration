#include "KinectProjectorCalibration.h"

using namespace ofxCv ;
using namespace cv;

KinectProjectorCalibration::KinectProjectorCalibration() {	
	//load my defaults
	b_CV_CALIB_FIX_PRINCIPAL_POINT = false;
	b_CV_CALIB_FIX_ASPECT_RATIO = false;
	b_CV_CALIB_ZERO_TANGENT_DIST = true; 
	b_CV_CALIB_FIX_K1 = true;
	b_CV_CALIB_FIX_K2 = true;
	b_CV_CALIB_FIX_K3 = true;
	b_CV_CALIB_FIX_K4 = false;
	b_CV_CALIB_FIX_K5 = false;
	b_CV_CALIB_FIX_K6 = false;
	b_CV_CALIB_RATIONAL_MODEL = false;
	b_CV_CALIB_CB_ADAPTIVE_THRESH = true;
	b_CV_CALIB_CB_NORMALIZE_IMAGE = false;
	b_CV_CALIB_CB_FAST_CHECK = true;
	projectorResolutionX = 1280;
	projectorResolutionY = 800;
	chessboardBlocksX = 8;
	chessboardBlocksY = 6;
	chessboardSize = 0.75;
	reprojError = -1;
	//not ready yet, first need to call setup(..)
	isReady = false;
	calibrated = false;

	chessboardFound = false;
	fastCheckResize = 0.5;
	chessboardColor = 125;
	kinectColorImage.allocate(640,480);
	stableFrom = -1;
}


void	KinectProjectorCalibration::setup(RGBDCamCalibWrapper* _kinect, int _projectorResolutionX, int _projectorResolutionY) {
	kinect = _kinect;
	projectorResolutionX = _projectorResolutionX;
	projectorResolutionY = _projectorResolutionY;
	isReady = true;
}


bool	KinectProjectorCalibration::doFastCheck(){
	if (!isReady) return false;
	ofxCvColorImage colorImg = kinect->getColorImageCalibrated();
	colorImg.resize(colorImg.width*fastCheckResize,colorImg.height * fastCheckResize);
	Mat colorImage = toCv(colorImg);
	pointBufFastCheck.clear();
	
	vector<Point2f> pointBuf;
	//setup for finding chessboards
	cv::Size patternSize = cv::Size(chessboardBlocksX, chessboardBlocksY);
	int flags = CV_CALIB_CB_FAST_CHECK;
	if(b_CV_CALIB_CB_ADAPTIVE_THRESH) flags += CV_CALIB_CB_ADAPTIVE_THRESH; 
	if(b_CV_CALIB_CB_NORMALIZE_IMAGE) flags += CV_CALIB_CB_NORMALIZE_IMAGE;  
	
	//find the chessboard
	bool b = findChessboardCorners(colorImage, patternSize, pointBuf, flags);

	if(b) {
		if (stableFrom == -1) 
			stableFrom = ofGetElapsedTimeMillis();
		for (int i = 0; i < pointBuf.size(); i++) {
			pointBufFastCheck.push_back((1.0/fastCheckResize) * toOf(pointBuf[i]));
		}
		//cout << "Stable: " << (ofGetElapsedTimeMillis() - stableFrom) << endl;
		return (ofGetElapsedTimeMillis() - stableFrom > 2000) ;
	} else {
		stableFrom = -1;
		return false;
	}

}

vector<ofVec2f> KinectProjectorCalibration::getFastCheckResults() {
	return pointBufFastCheck;
}

void	KinectProjectorCalibration::addCurrentFrame(){
	if (!isReady) return;
	
	stableFrom = -1;

	//gets the DEPTH/RGB CALIBRATED color image
	kinectColorImage = kinect->getColorImageCalibrated();
	Mat colorImage = toCv(kinectColorImage);
	
	//setup for finding chessboards
	cv::Size patternSize = cv::Size(chessboardBlocksX, chessboardBlocksY);
	vector<Point2f> pointBuf;
	int flags = 0;
	if(b_CV_CALIB_CB_ADAPTIVE_THRESH) flags += CV_CALIB_CB_ADAPTIVE_THRESH; 
	if(b_CV_CALIB_CB_NORMALIZE_IMAGE) flags += CV_CALIB_CB_NORMALIZE_IMAGE;  
	//if(b_CV_CALIB_CB_FAST_CHECK) flags += CV_CALIB_CB_FAST_CHECK; 
	
	//find the chessboard
	chessboardFound = findChessboardCorners(colorImage, patternSize, pointBuf, flags);

	//we found one, lets refine
	if (chessboardFound) {
		//refine				
		Mat gray;
		cvtColor(colorImage, gray, CV_RGB2GRAY);
		cornerSubPix(gray, pointBuf, cv::Size(1, 1), cv::Size(-1, -1),
		TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			
		//make some arrays
		vector<cv::Point3f>		worldCoordinatesChessboard;
		vector<cv::Point2f>		kinectCoordinatesChessboard;
		vector<cv::Point2f>		imageCoordinatesChessboard;
		vector<ofPoint>			imageCoordinatesChessboardInternal = chessboard.getInternalPoints();

		bool valid = true;
		//convert to world	
		for (int i = 0; i < pointBuf.size(); i++) {
			//kinect & world cooords
			ofVec2f kinectCoords = ofVec2f(pointBuf[i].x, pointBuf[i].y);
			ofVec3f worldCoords = kinect->getWorldFromRgbCalibrated(kinectCoords);
			if (worldCoords.z <= 0.01) { 
				valid = false;
				break;
			}
			cv::Point2f kinectCV = toCv(kinectCoords);
			cv::Point3f worldCV = toCv(worldCoords);
			worldCoordinatesChessboard.push_back(worldCV);
			kinectCoordinatesChessboard.push_back(kinectCV);

			//chessboard coords
			imageCoordinatesChessboard.push_back(Point2f(imageCoordinatesChessboardInternal[i].x, imageCoordinatesChessboardInternal[i].y));
		}
		if (valid) {
			//add to our list off valid pints
			worldCoordinatesChessboardBuffer.push_back(worldCoordinatesChessboard);
			kinectCoordinatesChessboardBuffer.push_back(kinectCoordinatesChessboard);
			imageCoordinatesChessboardBuffer.push_back(imageCoordinatesChessboard);

			//update our calibration
			calibrate();
		}
	} 
}


bool	KinectProjectorCalibration::calibrate()
{
	if (!isReady) return false;
	if (worldCoordinatesChessboardBuffer.size() < 1) return false;
	
	//load defaults
	cameraMatrix = (Mat1d(3, 3) << 	projectorResolutionX, 0, projectorResolutionX / 2.,
					                0, projectorResolutionY, projectorResolutionY / 2.,
					                0, 0, 1);
	distCoeffs = Mat::zeros(8, 1, CV_64F);
	
	//load calib settings
	int flags = CV_CALIB_USE_INTRINSIC_GUESS;
	if(b_CV_CALIB_FIX_ASPECT_RATIO) flags += CV_CALIB_FIX_ASPECT_RATIO; 
	if(b_CV_CALIB_ZERO_TANGENT_DIST) flags += CV_CALIB_ZERO_TANGENT_DIST;  
	if(b_CV_CALIB_FIX_K1) flags += CV_CALIB_FIX_K1; 
	if(b_CV_CALIB_FIX_K2) flags += CV_CALIB_FIX_K2; 
	if(b_CV_CALIB_FIX_K3) flags += CV_CALIB_FIX_K3; 
	if(b_CV_CALIB_FIX_K4) flags += CV_CALIB_FIX_K4; 
	if(b_CV_CALIB_FIX_K5) flags += CV_CALIB_FIX_K5; 
	if(b_CV_CALIB_FIX_K6) flags += CV_CALIB_FIX_K6; 
	if(b_CV_CALIB_RATIONAL_MODEL) flags += CV_CALIB_RATIONAL_MODEL; 
		

	//THIS IS NEEDED BECAUSE IT WONT WORK OHTERWISE
	vector<vector<Point3f> > vvo(1); //object points
	vector<vector<Point2f> > vvi(1); //image points
	for (int i=0; i<worldCoordinatesChessboardBuffer.size(); ++i) {
		for (int j = 0; j<worldCoordinatesChessboardBuffer[i].size(); j++) {
			vvo[0].push_back(worldCoordinatesChessboardBuffer[i][j]);
			vvi[0].push_back(imageCoordinatesChessboardBuffer[i][j]);
		}
	}

	//actual calibration wit the hack
	reprojError = calibrateCamera(vvo, vvi, cv::Size(projectorResolutionX, projectorResolutionY), cameraMatrix, distCoeffs, boardRotations, boardTranslations, flags);
	
	calibrated = true;
	//this doesn't work :(
	//reprojError = calibrateCamera(worldCoordinatesChessboardBuffer, imageCoordinatesChessboardBuffer, cv::Size(projectorResolutionX, projectorResolutionY), cameraMatrix, distCoeffs, boardRotations, boardTranslations, flags);
	
	//setup intrinsics
	intrinsics.setup(cameraMatrix, cv::Size(projectorResolutionX, projectorResolutionY));

	//calculate individual vieuws reprojection error
	int totalPoints = 0;
	double totalErr = 0;		
	perViewErrors.clear();	
	
	perViewErrors.resize(worldCoordinatesChessboardBuffer.size());
	for(int i = 0; i < worldCoordinatesChessboardBuffer.size(); i++) {
		vector<ofVec2f> imagePts = project(worldCoordinatesChessboardBuffer[i],i );
		double err = norm(Mat(toCv(imagePts)), Mat(imageCoordinatesChessboardBuffer[i]), CV_L2);
		int n = worldCoordinatesChessboardBuffer[i].size();
		perViewErrors[i] = sqrt(err * err / n);
		totalErr += err * err;
		totalPoints += n;
		ofLog(OF_LOG_NOTICE, "view " + ofToString(i) + " has error of " + ofToString(perViewErrors[i]));
	}
	
	//debug output
	cout << " " << endl;
	cout << " " << endl;
	cout << "RMS: " << reprojError << endl;	
	cout << "Camera matrix" << endl;	
	cout << cameraMatrix << endl;
	cout << "Dist coeffs" << endl;
	cout << distCoeffs << endl;
	cout << "Principal point: " << intrinsics.getPrincipalPoint() << endl;	
	cout << "Reprojection Error " << reprojError << endl;

	//save
	save("kinectProjector.yml",true);
	
	return true;
}

bool KinectProjectorCalibration::clean(float minReprojectionError) {
	if (!calibrated) return false;
	calibrated = false;
	int removed = 0;
	for(int i = worldCoordinatesChessboardBuffer.size() - 1; i >= 0; i--) {
		if(getReprojectionError(i) > minReprojectionError) {
			worldCoordinatesChessboardBuffer.erase(worldCoordinatesChessboardBuffer.begin() + i);
			kinectCoordinatesChessboardBuffer.erase(kinectCoordinatesChessboardBuffer.begin() + i);
			imageCoordinatesChessboardBuffer.erase(imageCoordinatesChessboardBuffer.begin() + i);
			removed++;
		}
	}
	if(worldCoordinatesChessboardBuffer.size() > 0) {
		if(removed > 0) {
			return calibrate();
		} else {
			return true;
		}
	} else {
		ofLog(OF_LOG_ERROR, "KinectProjectorCalibration::clean() removed the last object/image point pair");
		return false;
	}
}

void KinectProjectorCalibration::clearAll() {
	if (!calibrated) return;
	calibrated = false;
	perViewErrors.clear();
	worldCoordinatesChessboardBuffer.clear();
	kinectCoordinatesChessboardBuffer.clear();
	imageCoordinatesChessboardBuffer.clear();
}

vector<ofVec2f> KinectProjectorCalibration::project(vector<Point3f> wrldSrc, int i) const {
	if (!calibrated)  { vector<ofVec2f> v; v.push_back(ofVec2f(0,0)); return v; }
	Mat mt = boardTranslations[0];
	Mat mr = boardRotations[0];	
	vector<Point2f> projected;		
	projectPoints(wrldSrc, mr, mt, cameraMatrix, distCoeffs, projected);	
	vector<ofVec2f> projectedOF;
	for (int i = 0; i < projected.size(); i++) {
		projectedOF.push_back(toOf(projected[i]));
	}
	return projectedOF;
}

float KinectProjectorCalibration::getReprojectionError() const {
	if (!calibrated) return 0.0;
	return reprojError;
}

float KinectProjectorCalibration::getReprojectionError(int i) const {
	if (!calibrated) return 0.0;
	return perViewErrors[i];
}

int KinectProjectorCalibration::getDatabaseSize() {
	return worldCoordinatesChessboardBuffer.size();
}

void KinectProjectorCalibration::save(string filename, bool absolute) const {
	if (worldCoordinatesChessboardBuffer.size() < 1) { 
		return;
	}

	FileStorage fs(ofToDataPath(filename, absolute), FileStorage::WRITE);
	fs << "intrisics" << cameraMatrix;
	fs << "projResX" << projectorResolutionX;
	fs << "projResY" << projectorResolutionY;
	fs << "rotation" << boardRotations[0];
	fs << "translation" << boardTranslations[0];
	fs << "reprojectionError" << reprojError;
}


/*************** DEBUG DRAW METHODS *****************/

void	KinectProjectorCalibration::drawChessboard()
{
	if (!isReady) return;
	drawChessboardDebug(0,0,projectorResolutionX,projectorResolutionY);
}

void	KinectProjectorCalibration::drawChessboardDebug(float x, float y, float width, float height)
{
	if (!isReady) return;

	//set the chessboard
	chessboard.setPatternBlocks(chessboardBlocksX, chessboardBlocksY);
	chessboard.setPatternSize(chessboardSize);
	chessboard.setProjectorResolution(projectorResolutionX, projectorResolutionY);
	
	//draw
	ofPushMatrix();
	ofPushStyle();
	ofTranslate(x,y);
	ofScale(width/projectorResolutionX, height/projectorResolutionY);
	ofSetColor(255);
	chessboard.setBgColor((int)chessboardColor);
	chessboard.draw();
	ofPopStyle();
	ofPopMatrix();
}

void	KinectProjectorCalibration::drawReprojectedPointsDebug(float x, float y, float width, float height){
	if (!isReady) return;

	//draw the chessboard
	drawChessboardDebug(x,y,width,height);

	//draw the reprojected points
	ofPushMatrix();
	ofPushStyle();

	ofTranslate(x,y);
	ofScale(width/projectorResolutionX, height/projectorResolutionY);
	ofSetColor(255);

	//reproject all 3d points stored
	ofSetColor(50,255,50);
	for (int i = 0; i < worldCoordinatesChessboardBuffer.size(); i++) {
		vector<ofVec2f> points = project(worldCoordinatesChessboardBuffer[i],0);
		for (int j = 0; j < points.size(); j++) {
			ofCircle(points[j].x, points[j].y, 8);
			ofDrawBitmapStringHighlight(ofToString(j),points[j].x, points[j].y);
		}
		
	}
	ofPopMatrix();
	ofPopStyle();
}

void	KinectProjectorCalibration::drawProcessedInputDebug(float x, float y, float width, float height)
{
	if (!isReady) return;

	ofPushMatrix();
	ofPushStyle();
	ofTranslate(x,y);
	ofScale(width/kinectColorImage.width, height/kinectColorImage.height);
	ofEnableAlphaBlending();
	ofSetColor(255,255,255,255);
	kinectColorImage.draw(0,0);		
	ofSetColor(255,255,255,255);
	if(chessboardFound && kinectCoordinatesChessboardBuffer.size() > 0) {
		ofSetColor(0,255,0,255);
		ofFill();
		int lastElement = kinectCoordinatesChessboardBuffer.size()-1;
		for (int i = 0; i < kinectCoordinatesChessboardBuffer[lastElement].size(); i++) {
			ofCircle(kinectCoordinatesChessboardBuffer[lastElement][i].x,kinectCoordinatesChessboardBuffer[lastElement][i].y,10);
		}
	}		
	if (stableFrom != -1)
		ofDrawBitmapStringHighlight(ofToString((int)(2000 - ofGetElapsedTimeMillis() + stableFrom)) + " ms to capture...", 20,20);
	ofPopStyle();
	ofPopMatrix();
	ofSetColor(255,255,255,255);
}