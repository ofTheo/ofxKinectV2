#pragma once


#include "ofMain.h"
#include "ofxKinectV2.h"
#include "ofxGui.h"


class ofApp: public ofBaseApp
{
public:
    void setup() override;
    void update() override;
    void draw() override;

    void keyPressed(int key) override;
    
    ofxPanel panel;

    std::vector<std::shared_ptr<ofxKinectV2>> kinects;

    std::vector<ofTexture> texDepth;
    std::vector<ofTexture> texRGB;
    std::vector<ofTexture> texIR;
    
    std::size_t currentKinect = 0;
};
