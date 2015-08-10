#include "testApp.h"
#include "ofMain.h"
#include "ofAppGlutWindow.h"
#include "ofxFensterManager.h"

//========================================================================
int main( ){
    
    //ofAppGlutWindow window;
    //window.setGlutDisplayString("rgba double samples>=6 depth");
	//ofSetupOpenGL(&window, 1024,768, OF_WINDOW);
	//ofRunApp(new testApp);
    
    ofSetupOpenGL(ofxFensterManager::get(), 1280, 800, OF_WINDOW);
		
	ofRunFensterApp(new testApp());
}
