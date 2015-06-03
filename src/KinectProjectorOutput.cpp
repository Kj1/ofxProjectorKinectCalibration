/** OFXKINECTPROJECTORCALIBRATION **/
/** work in progress, not even beta! **/
/** Kj1, www.hangaar.net **/

#include "KinectProjectorOutput.h"

using namespace ofxCv ;
using namespace cv;

KinectProjectorOutput::KinectProjectorOutput() {	
	projectorResolutionX = 1280;
	projectorResolutionY = 800;
	reprojError = -1;
	isReady = false;
}


void	KinectProjectorOutput::setup(RGBDCamCalibWrapper* _kinect, int _projectorResolutionX, int _projectorResolutionY) {
	kinect = _kinect;
	projectorResolutionX = _projectorResolutionX;
	projectorResolutionY = _projectorResolutionY;
}

bool KinectProjectorOutput::isCalibrationReady() {
	return isReady;
}

ofPoint KinectProjectorOutput::projectFromDepthXY(const ofPoint o) const {
	if (!isReady) return ofPoint(0,0);
	ofPoint ptWorld = kinect->getWorldFromRgbCalibrated(o);
	vector<cv::Point3f> vo;
	vo.push_back(cv::Point3f(ptWorld.x, ptWorld.y, ptWorld.z));	
	Mat mt = boardTranslations[0];
	Mat mr = boardRotations[0];	
	vector<Point2f> projected;		
	projectPoints(Mat(vo), mr, mt, cameraMatrix, distCoeffs, projected);	
	return toOf(projected[0]);
}

ofVec2f KinectProjectorOutput::project(const Point3f o) const {
	if (!isReady) return ofVec2f(0,0);
	vector<cv::Point3f> vo(1, o);	
	Mat mt = boardTranslations[0];
	Mat mr = boardRotations[0];	
	vector<Point2f> projected;		
	projectPoints(Mat(vo), mr, mt, cameraMatrix, distCoeffs, projected);	
	return toOf(projected[0]);
}

vector<ofVec2f> KinectProjectorOutput::project(vector<Point3f> wrldSrc, int i) const {
	if (!isReady)  { vector<ofVec2f> v; v.push_back(ofVec2f(0,0)); return v; }
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

vector<ofPoint> KinectProjectorOutput::project(vector<ofPoint> wrldSrc, int i) const {
	if (!isReady)  { vector<ofPoint> v; v.push_back(ofPoint(0,0)); return v; }
	Mat mt = boardTranslations[0];
	Mat mr = boardRotations[0];	
	vector<Point2f> projected;		
	vector<Point3f> src = toCv(wrldSrc);		
	projectPoints(src, mr, mt, cameraMatrix, distCoeffs, projected);	
	vector<ofPoint> projectedOF;
	for (int i = 0; i < projected.size(); i++) {
		projectedOF.push_back(toOf(projected[i]));
	}
	return projectedOF;
}

void	KinectProjectorOutput::loadCalibratedView(){
	if (!isReady) return;
	glPushMatrix();
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glMatrixMode(GL_MODELVIEW);

	intrinsics.loadProjectionMatrix(0.001, 2000);
	applyMatrix(modelMatrix);
}

void	KinectProjectorOutput::unloadCalibratedView(){
	if (!isReady) return;
	
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}

float KinectProjectorOutput::getReprojectionError() const {
	if (!isReady) return 0.0;
	return reprojError;
}

bool KinectProjectorOutput::load(string path, bool absolute) {
	FileStorage fs(ofToDataPath(path, absolute), FileStorage::READ);
	cv::Mat	rvec, tvec;
    fs["intrisics"] >> cameraMatrix;
    fs["projResX"] >> projectorResolutionX;
	fs["projResY"] >> projectorResolutionY;
    fs["rotation"] >> rvec;
    fs["translation"] >> tvec;
	fs["reprojectionError"] >>  reprojError;
    intrinsics.setup(cameraMatrix, Size2i(projectorResolutionX, projectorResolutionY));
    modelMatrix = makeMatrix(rvec, tvec);    
	boardRotations.push_back(rvec);
	boardTranslations.push_back(tvec);
	distCoeffs = Mat::zeros(8, 1, CV_64F);

    isReady = true;
	return true;
}