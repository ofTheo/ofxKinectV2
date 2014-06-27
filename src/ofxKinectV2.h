//
//  ofxKinectV2.h
//  kinectExample
//
//  Created by Theodore Watson on 6/23/14.
//
//

#pragma once

#include "ofProtonect.h"
#include "ofMain.h"

class ofxKinectV2 : public ofThread{

    public:
        ofxKinectV2();
        ~ofxKinectV2(); 
        
        bool open();
        void update();
        void close();
    
        bool isFrameNew();
    
        ofPixels getDepthPixels();
        ofPixels getRgbPixels();
        ofFloatPixels getRawDepthPixels();
    
        ofParameter <float> minDistance;
        ofParameter <float> maxDistance;

    protected:
        void threadedFunction();

        ofPixels rgbPix;
        ofPixels depthPix;
        ofFloatPixels rawDepthPixels;
    
        bool bNewBuffer;
        bool bNewFrame;
        bool bOpened;
    
        ofProtonect protonect; 
    
        ofPixels rgbPixelsBack;
        ofPixels rgbPixelsFront;
        ofFloatPixels depthPixelsBack;
        ofFloatPixels depthPixelsFront;
        int lastFrameNo; 
};