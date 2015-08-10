/** OFXKINECTPROJECTORCALIBRATION **/
/** work in progress, not even beta! **/
/** Kj1, www.hangaar.net **/

#pragma once

#include "ofxCv.h"
#include "ofMain.h"
#include "RGBDCamCalibWrapper.h"

using namespace ofxCv;
using namespace cv;
	
class KinectProjectorOutput {
	public:
		//setup
		KinectProjectorOutput();			
		void setup(RGBDCamCalibWrapper* _kinect, int _projectorResolutionX, int _projectorResolutionY);
		
		//project functions
		ofPoint			projectFromDepthXY(const ofPoint o) const;
		ofVec2f			project(const Point3f o) const;
		vector<ofVec2f> project(vector<Point3f> src, int i) const;
		vector<ofPoint> project(vector<ofPoint> src, int i) const;
		void			loadCalibratedView(); 
		void			unloadCalibratedView();

		//load
		bool	load(string path, bool absolute = false);
		bool	isCalibrationReady();

		//getters & setters		
		float	getReprojectionError() const;
        void    setMirrors(bool horizontal, bool vertical);



	protected:
		//settings
		int					projectorResolutionX;
		int					projectorResolutionY;

		//other
		RGBDCamCalibWrapper* kinect;
		bool				isReady;
        bool                mirrorVert, mirrorHoriz;

		//calibrationResults
		float				reprojError;
		Mat					cameraMatrix, distCoeffs;
		vector<Mat>			boardRotations, boardTranslations;	
		Intrinsics			intrinsics;
		ofMatrix4x4			projectionMatrix;		
		ofMatrix4x4			modelMatrix;		
};
	
