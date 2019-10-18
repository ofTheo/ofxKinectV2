//  ofProtonect.cpp
//
//  Created by Theodore Watson on 11/16/15


#include "ofMain.h"

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>


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
        , CUDA,
        CUDAKDE
#endif
    };

    int open(const std::string& serial,
             PacketPipelineType packetPipelineType, libfreenect2::Freenect2Device::Config aConfig );
    
    void updateKinect(ofPixels& rgbPixels,
                      ofPixels& rgbRegisteredPixels,
                      ofFloatPixels& depthPixels,
                      ofFloatPixels& irPixels,
                      ofFloatPixels& distancePixels);

    int closeKinect();

    libfreenect2::Freenect2& getFreenect2Instance()
    {
        return freenect2;
    }

	void setRGBEnabled(bool ab) {
		enableRGB = ab;
	};
	void setDepthEnabled(bool ab) {
		enableDepth = ab;
	};
	void setIREnabled(bool ab) {
		enableIR = ab;
	}
	void setEnableRGBRegistration(bool ab) {
		enableRGBRegistration = ab;
	}
  
protected:
    bool enableRGB = true;
    bool enableDepth = true;
	bool enableIR = true;
	bool enableRGBRegistration = true;
    int deviceId = -1;

    bool bOpened = false;
    
    libfreenect2::Freenect2 freenect2;

    libfreenect2::Freenect2Device* dev = nullptr;
    libfreenect2::FrameMap frames;

    libfreenect2::Registration* registration = nullptr;
    libfreenect2::SyncMultiFrameListener* listener = nullptr;
    libfreenect2::Frame* undistorted = nullptr;
    libfreenect2::Frame* registered = nullptr;

    friend class ofxKinectV2;
};
