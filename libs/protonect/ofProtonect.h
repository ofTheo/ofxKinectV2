//  ofProtonect.cpp
//
//  Created by Theodore Watson on 11/16/15


#include "ofMain.h"

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
//#include <transfer_pool.h>
//#include <event_loop.h>
#include "ofRGBPacketProcessor.h"

#include "ofAppGLFWWindow.h"
#include "ofAppRunner.h"


class ofProtonect
{
public:
    enum class PacketPipelineType
    {
        DEFAULT,
        CPU,
        OPENGL,
        OPENCL,
        OPENCLKDE
#if defined(LIBFREENECT2_WITH_CUDA_SUPPORT)
        CUDA
        CUDAKDE
#endif
    };

    ofProtonect();
    
    int open(const std::string& serial,
             PacketPipelineType packetPipelineType = PacketPipelineType::DEFAULT);
    
    void updateKinect(ofPixels& rgbPixels,
                      ofFloatPixels& depthPixels,
                      ofFloatPixels& irPixels);

    int closeKinect();

    libfreenect2::Freenect2& getFreenect2Instance()
    {
        return freenect2;
    }
  
protected:
    bool enableRGB = true;
    bool enableDepth = true;
    int deviceId = -1;

    bool bOpened = false;
    
    libfreenect2::Freenect2 freenect2;

    libfreenect2::Freenect2Device* dev;
    libfreenect2::PacketPipeline* pipeline;

    libfreenect2::FrameMap frames;

    libfreenect2::Registration* registration;
    libfreenect2::SyncMultiFrameListener* listener;
    libfreenect2::Frame* undistorted;
    libfreenect2::Frame* registered;
    libfreenect2::Frame* bigFrame;

};
