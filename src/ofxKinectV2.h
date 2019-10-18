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

	class Settings {
	public:
		Settings() {
			config.MinDepth = 0.5f;
			config.MaxDepth = 8.f;
			config.EnableBilateralFilter = true; //these are good defaults - use setConfig if you care to chang
			config.EnableEdgeAwareFilter = true;
		};

		bool enableDepth = true;
		bool enableRGB = true;
		bool enableIR = true;
		bool enableRGBRegistration = true;
		libfreenect2::Freenect2Device::Config config;
		ofProtonect::PacketPipelineType pipeline = ofProtonect::PacketPipelineType::OPENCL;
	};

    ofxKinectV2();
    ~ofxKinectV2();
    
    //for some reason these can't be static - so you need to make a tmp object to query them
    std::vector<KinectDeviceInfo> getDeviceList() const;
    std::size_t getNumDevices() const;
    

    /// \brief Open the device with the given serial number.
    /// \param serial The serial number to open.
    /// \returns true if connected successfully.
    bool open(const std::string& serial, Settings asettings = Settings() );

    /// \brief Open the device with the given serial number.
    /// \param deviceId The device id to open.
    /// \returns true if connected successfully.
    bool open(int deviceId = 0, Settings asettings = Settings() );

    /// \brief Update the Kinect internals.
    void update();
    
    /// \brief Close the connection to the Kinect.
    void close();

    /// \returns true if the frame has been updated.
    bool isFrameNew() const;

    OF_DEPRECATED_MSG("Use getPixels()", ofPixels getRgbPixels());

    /// \returns the RGB pixels.
    const ofPixels& getPixels() const;

    /// \returns pixels registred to the depth image.
    const ofPixels& getRegisteredPixels() const;

    /// \returns the raw depth pixels.
    const ofFloatPixels& getRawDepthPixels() const;

    /// \returns the depth pixels mapped to a visible range.
    const ofPixels& getDepthPixels() const;

    /// \returns the raw IR pixels.
    const ofFloatPixels& getRawIRPixels() const;

    /// \returns the IR pixels mapped to a visible range.
    const ofPixels& getIRPixels() const;

    /// \returns the distance image. Each pixels is the distance in millimeters.
    //const ofFloatImage& getDistancePixels() const;
    
    /// \brief Get the calulated distance for point x, y in the getRegisteredPixels image.
    float getDistanceAt(std::size_t x, std::size_t y) const;
    
    /// \brief Get the world X, Y, Z coordinates in millimeters for x, y in getRegisteredPixels image.
    glm::vec3 getWorldCoordinateAt(std::size_t x, std::size_t y) const;

	/// \brief Get the minimum distance the kinect is tracking in millimeters
	int getMinDistance() const;

	/// \brief Get the maximum distance the kinect is tracking in millimeters
	int getMaxDistance() const;

	bool isDepthEnabled() const;
	bool isIREnabled() const;
	bool isRGBEnabled() const;
	bool isRGBRegistrationEnabled() const;
    
    ofParameterGroup params;
    ofParameter<float> minDistance;
    ofParameter<float> maxDistance;
    ofParameter<float> irExposure;

protected:
    void threadedFunction();

    ofPixels pixels;
    ofPixels registeredPixels;
    ofFloatPixels rawDepthPixels;
    ofPixels depthPixels;
    ofFloatPixels distancePixels;

    ofFloatPixels rawIRPixels;
    ofPixels irPixels;
    
    bool bNewBuffer = false;
    bool bNewFrame = false;
    bool bOpened = false;

    mutable ofProtonect protonect;
    
    ofPixels pixelsBack;
    ofPixels pixelsFront;
    ofPixels registeredPixelsBack;
    ofPixels registeredPixelsFront;
    ofFloatPixels rawDepthPixelsBack;
    ofFloatPixels rawDepthPixelsFront;
    ofFloatPixels rawIRPixelsBack;
    ofFloatPixels rawIRPixelsFront;
    ofFloatPixels distancePixelsFront;
    ofFloatPixels distancePixelsBack;

	Settings mSettings = Settings();

    int lastFrameNo = -1;
};
