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


class ofxKinectV2: public ofThread
{
public:
    struct KinectDeviceInfo
    {
        std::string serial;
        int deviceId;   //if you have the same devices plugged in device 0 will always be the same Kinect
        int freenectId; //don't use this one - this is the index given by freenect2 - but this can change based on order device is plugged in
    };

    ofxKinectV2();
    ~ofxKinectV2();
    
    //for some reason these can't be static - so you need to make a tmp object to query them
    std::vector<KinectDeviceInfo> getDeviceList() const;
    std::size_t getNumDevices() const;

    bool open(const std::string& serial);
    bool open(int deviceId = 0);
    void update();
    void close();

    bool isFrameNew() const;

    ofPixels getDepthPixels();
    ofPixels getRgbPixels();
    ofFloatPixels getRawDepthPixels();
    ofFloatPixels getIrPixels();

    ofParameterGroup params;
    ofParameter <float> minDistance;
    ofParameter <float> maxDistance;

protected:
    void threadedFunction();

    ofPixels rgbPix;
    ofPixels depthPix;
    ofFloatPixels rawDepthPixels;
    ofFloatPixels irPix;

    bool bNewBuffer = false;
    bool bNewFrame = false;
    bool bOpened = false;

    mutable ofProtonect protonect;

    ofPixels rgbPixelsBack;
    ofPixels rgbPixelsFront;
    ofFloatPixels depthPixelsBack;
    ofFloatPixels depthPixelsFront;
    ofFloatPixels irPixelsBack;
    ofFloatPixels irPixelsFront;
    int lastFrameNo = -1;
};
