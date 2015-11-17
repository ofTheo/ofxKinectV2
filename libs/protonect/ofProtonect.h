//  ofProtonect.cpp
//
//  Created by Theodore Watson on 11/16/15


#include "ofMain.h"

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <transfer_pool.h>
#include <event_loop.h>
#include "ofRGBPacketProcessor.h"

#include "ofAppGLFWWindow.h"
#include "ofAppRunner.h"

class ofProtonect{

    public:


    
        ofProtonect();
    
        int openKinect(std::string serialNo);
        void updateKinect(ofPixels & rgbPixels, ofFloatPixels & depthPixels);
        int closeKinect();
    
        libfreenect2::Freenect2 & getFreenect2Instance(){
            return freenect2;
        }
  
    protected:
  
        bool bOpened;
        
        libfreenect2::Freenect2 freenect2;

        libfreenect2::Freenect2Device *dev = 0;
        libfreenect2::PacketPipeline *pipeline = 0;

        libfreenect2::FrameMap frames;

        libfreenect2::Registration* registration;
        libfreenect2::SyncMultiFrameListener * listener;
        libfreenect2::Frame  * undistorted = NULL;
        libfreenect2::Frame  * registered = NULL;
        libfreenect2::Frame  * bigFrame = NULL;

};