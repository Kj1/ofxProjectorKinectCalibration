/** OFXKINECTPROJECTORCALIBRATION **/
/** work in progress, not even beta! **/
/** Kj1, www.hangaar.net **/

#include "SecondWindow.h"

SecondWindow::SecondWindow() {
	fbo = NULL;
	c = NULL;
}

void SecondWindow::setFbo(ofFbo* _fbo) {
	fbo = _fbo;
}

void SecondWindow::draw(){
	ofPushStyle();  
	ofSetColor(255);
    ofEnableAlphaBlending();
	if (fbo != NULL) {
		fbo->draw(0,0);
	}
	ofPopStyle();
}

void SecondWindow::setHandle(ofxFenster* _win) {
	win = _win;
}

void SecondWindow::toggleFullScreen() {
	win->toggleFullscreen();
}
