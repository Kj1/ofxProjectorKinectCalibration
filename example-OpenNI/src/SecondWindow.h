/** OFXKINECTPROJECTORCALIBRATION **/
/** work in progress, not even beta! **/
/** Kj1, www.hangaar.net **/

#pragma once

#include "ofMain.h"
#include "ofxFensterManager.h"
#include "ofxOpenCv.h"
#include "Chessboard.h"

class SecondWindow: public ofxFensterListener {
public:

	SecondWindow();
	void draw();	 
	void setHandle(ofxFenster* _win);
	void toggleFullScreen();
	void setFbo(ofFbo* _fbo);
	Chessboard* c;

private:
	ofxFenster* win;
	ofFbo* fbo;
};
