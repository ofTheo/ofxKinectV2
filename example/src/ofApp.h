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
    
    void drawTextureAtRowAndColumn(const std::string& title,
                                   const ofTexture& tex,
                                   int row,
                                   int column);
    
    ofxPanel panel;

    std::vector<std::shared_ptr<ofxKinectV2>> kinects;

    ofParameter<float> pointSize = {"Point Size", 1, 1, 10};
    
    
    std::vector<ofTexture> texRGB;
    std::vector<ofTexture> texRGBRegistered;
    std::vector<ofTexture> texIR;
    std::vector<ofTexture> texDepth;
    
    std::size_t currentKinect = 0;
    
    int numRows = 2;
    int numColumns = 2;

    ofEasyCam cam;
    ofMesh pointCloud;
    bool showPointCloud = false;
};
