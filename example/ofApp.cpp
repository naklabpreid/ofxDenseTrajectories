#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){

    //please input directry include seaquence image data
    dir.listDir(ofToDataPath("data"));
    dir.sort();

    isInit = false;
    count = 0;
}

//--------------------------------------------------------------
void ofApp::update(){
    
    if(!isInit){
        ofxDT.setup(dir.getPath(count));
        isInit = true;
    }else{
        ofxDT.update();
        if(ofxDT.isFinished()){
            std::string fileName = dir.getName(count);
            string::size_type pos = fileName.find_last_of(".");
            fileName = fileName.substr(0, pos);

            ofxDT.writeCSV(ofToDataPath("result/HOG/" + fileName + "_HOG.csv"), ofxDenseTrajectories::DescType::HOG);
            ofxDT.writeCSV(ofToDataPath("result/HOF/" + fileName + "_HOF.csv"), ofxDenseTrajectories::DescType::HOF);
            ofxDT.writeCSV(ofToDataPath("result/MBHx/" + fileName + "_MBHx.csv"), ofxDenseTrajectories::DescType::MBHx);
            ofxDT.writeCSV(ofToDataPath("result/MBHy/" + fileName + "_MBHy.csv"), ofxDenseTrajectories::DescType::MBHy);
            count++;
            std::cout << "Count : " << count << std::endl;
            isInit = false;
        }
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    if(count == dir.size()){
        ofExit();
    }else{
        ofxDT.drawTrajectories();
    }
}
