/** OFXKINECTPROJECTORCALIBRATION **/
/** work in progress, not even beta! **/
/** Kj1, www.hangaar.net **/

#pragma once

#include "ofxCv.h"
#include "ofMain.h"
#include "Chessboard.h"
#include "RGBDCamCalibWrapper.h"

using namespace ofxCv;
using namespace cv;
	
class KinectProjectorCalibration {
	public:
		//setup
		KinectProjectorCalibration();			
		void setup(RGBDCamCalibWrapper* _kinect, int _projectorResolutionX, int _projectorResolutionY);
		
		//some public parameters
		bool b_CV_CALIB_FIX_PRINCIPAL_POINT;
		bool b_CV_CALIB_FIX_ASPECT_RATIO;
		bool b_CV_CALIB_ZERO_TANGENT_DIST; 
		bool b_CV_CALIB_FIX_K1;
		bool b_CV_CALIB_FIX_K2;
		bool b_CV_CALIB_FIX_K3;
		bool b_CV_CALIB_FIX_K4;
		bool b_CV_CALIB_FIX_K5;
		bool b_CV_CALIB_FIX_K6;
		bool b_CV_CALIB_RATIONAL_MODEL;
		bool b_CV_CALIB_CB_ADAPTIVE_THRESH;
		bool b_CV_CALIB_CB_NORMALIZE_IMAGE;
		bool b_CV_CALIB_CB_FAST_CHECK;
		float chessboardSize;
		float chessboardColor;

		//fast checking
		bool			doFastCheck();		//does a fast check on resized frame
		vector<ofVec2f> getFastCheckResults();

		//enable/disable calibration and adding images
		void	addCurrentFrame();
		bool	calibrate();
		bool	clean(float maxReprojectionError = 2.f);
		void    clearAll();
				
		//save
		void    save(string filename, bool absolute) const;
		
		//getters & setters		
		float	getReprojectionError() const;
		int		getDatabaseSize();
        void    setStabilityTimeInMs(int stabilityTime);
        void    setChessboardTranslation(float x, float y);

		//Gui stuff
		void	drawChessboard();
		void	drawChessboardDebug(float x, float y, float width, float height);
		void	drawReprojectedPointsDebug(float x, float y, float width, float height);
		void	drawProcessedInputDebug(float x, float y, float width, float height);
	

	protected:
		//settings
		int					projectorResolutionX;
		int					projectorResolutionY;

		//calibration buffers
		vector<vector <cv::Point3f>	>	worldCoordinatesChessboardBuffer;
		vector<vector <cv::Point2f>	>	kinectCoordinatesChessboardBuffer;
		vector<vector <cv::Point2f>	>	imageCoordinatesChessboardBuffer;
		
		//project 
		vector<ofVec2f> project(vector<Point3f> wrldSrc, int i) const;


		//other
		Chessboard			chessboard;
		RGBDCamCalibWrapper* kinect;
		bool				isReady;
		ofxCvColorImage		kinectColorImage;
		int					chessboardBlocksX;
		int					chessboardBlocksY;
		bool				chessboardFound;
		vector<ofVec2f>		pointBufFastCheck;
		float				fastCheckResize;
        int                 hasToBeStableFor;
		long				stableFrom;

		//calibrationResults
		float				reprojError;
		vector<float>		perViewErrors;
		float				getReprojectionError(int i) const;
		bool				calibrated;
		Mat					cameraMatrix, distCoeffs;
		vector<Mat>			boardRotations, boardTranslations;	
		Intrinsics			intrinsics;
		ofMatrix4x4			projectionMatrix;
		ofMatrix4x4			modelViewMatrix;
		cv::Mat				rvec, tvec;
		ofMatrix4x4			modelMatrix;

};
	
