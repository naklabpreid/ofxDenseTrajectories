#pragma once

#include "ofMain.h"
#include "ofxDenseTrajectories.hpp"
#include "ofVideoPlayer.h"

using namespace cv;

class ofApp : public ofBaseApp{

public:
    void setup();
    void update();
    void draw();

    ofDirectory dir;
    ofxDenseTrajectories ofxDT;
    
    cv::VideoCapture vid;
        
    bool isInit = false;
    int count = 0;

};
