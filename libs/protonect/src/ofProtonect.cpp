//  ofProtonect.cpp
//
//  Modified by Theodore Watson on 11/16/15
//  from the Protonect example in https://github.com/OpenKinect/libfreenect2


/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2011 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */


#include "ofProtonect.h"
//#include <iostream>
//#include <signal.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

int ofProtonect::open(const std::string& serial, PacketPipelineType packetPipelineType, libfreenect2::Freenect2Device::Config aConfig) {

    if (ofGetLogLevel() == OF_LOG_VERBOSE){
        libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug));
    }
    else{
        libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Warning));
    }

	if (!enableRGB && !enableDepth && !enableIR) {
		ofLogError("ofProtonect::openKinect") << " neither color, depth or IR is enabled!";
		return -1;
	}
    
    //this is deleted by openDevice
    libfreenect2::PacketPipeline* pipeline = nullptr;

	switch (packetPipelineType) {
	case PacketPipelineType::CPU:
		pipeline = new libfreenect2::CpuPacketPipeline();
		break;
	case PacketPipelineType::OPENGL:
		pipeline = new libfreenect2::OpenGLPacketPipeline();
		break;
	case PacketPipelineType::OPENCL:
		pipeline = new libfreenect2::OpenCLPacketPipeline();
		break;
	case PacketPipelineType::OPENCLKDE:
		pipeline = new libfreenect2::OpenCLKdePacketPipeline();
		break;
#if defined(LIBFREENECT2_WITH_CUDA_SUPPORT)
	case PacketPipelineType::CUDA:
		pipeline = new libfreenect2::CudaPacketPipeline(deviceId);
		break;
	case PacketPipelineType::CUDAKDE:
		pipeline = new libfreenect2::CudaKdePacketPipeline(deviceId);
		break;
#endif
	case PacketPipelineType::DEFAULT:
		break;
	}

	if (pipeline) {
		dev = freenect2.openDevice(serial, pipeline);
	} else {
		dev = freenect2.openDevice(serial);
	}

	if (!dev) {
		ofLogError("ofProtonect::openKinect") << "failure opening device with serial " << serial;
		return -1;
	}

	int types = 0;

	if (enableRGB)
		types |= libfreenect2::Frame::Color;
	if (enableDepth || enableIR)
		types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;

	listener = new libfreenect2::SyncMultiFrameListener(types);

	if (enableRGB) dev->setColorFrameListener(listener);
	if (enableDepth || enableIR) dev->setIrAndDepthFrameListener(listener);

	dev->setConfiguration(aConfig);

	/// [start]
	if (enableRGB && (enableDepth || enableIR) ) {
		if (!dev->start()) {
			ofLogError("ofProtonect::openKinect") << "Error starting default stream for: " << serial;
			return -1;
		}
	} else {
		if (!dev->startStreams(enableRGB, (enableDepth || enableIR) )) {
			ofLogError("ofProtonect::openKinect") << "Error starting selected streams for: " << serial;
			return -1;
		}
	}

	ofLogVerbose("ofProtonect::openKinect") << "device serial: " << dev->getSerialNumber();
	ofLogVerbose("ofProtonect::openKinect") << "device firmware: " << dev->getFirmwareVersion();

	if (enableRGBRegistration && enableRGB && (enableDepth || enableIR) ) {
		registration = new libfreenect2::Registration(dev->getIrCameraParams(),
			dev->getColorCameraParams());

		undistorted = new libfreenect2::Frame(512, 424, 4);
		registered = new libfreenect2::Frame(512, 424, 4);
	}
    
    bOpened = true;
    
    return 0;
}

void ofProtonect::updateKinect(ofPixels& rgbPixels,
                               ofPixels& rgbRegisteredPixels,
                               ofFloatPixels& depthPixels,
                               ofFloatPixels& irPixels,
                               ofFloatPixels& distancePixels)
{
    if (bOpened)
    {
        if (!listener->waitForNewFrame(frames, 10 * 1000))
        {
            ofLogError("ofProtonect::updateKinect") << "Timeout serial: " << dev->getSerialNumber();
            return;
        }

		libfreenect2::Frame* rgb;// = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame* ir;// = frames[libfreenect2::Frame::Ir];
		libfreenect2::Frame* depth;// = frames[libfreenect2::Frame::Depth];
		if (enableRGB) {
			rgb = frames[libfreenect2::Frame::Color];
		}
		if (enableDepth) {
			depth = frames[libfreenect2::Frame::Depth];
		}
		if (enableIR) {
			ir = frames[libfreenect2::Frame::Ir];
		}

        if (enableRGB && enableDepth && enableRGBRegistration)
        {
            registration->apply(rgb,
                                depth,
                                undistorted,
                                registered);
        }

		if (enableRGB) {
			ofPixelFormat rgbFormat;
			if (rgb->format == libfreenect2::Frame::BGRX) {
				rgbFormat = OF_PIXELS_BGRA;
			} else {
				rgbFormat = OF_PIXELS_RGBA;
			}

			rgbPixels.setFromPixels(rgb->data, rgb->width, rgb->height, rgbFormat);
			if (enableDepth && enableRGBRegistration) {
				rgbRegisteredPixels.setFromPixels(registered->data, registered->width, registered->height, rgbFormat);
			}
		}

		if (enableDepth) {
			depthPixels.setFromPixels(reinterpret_cast<float*>(depth->data), depth->width, depth->height, 1);
		}
		if (enableIR) {
			irPixels.setFromPixels(reinterpret_cast<float*>(ir->data), ir->width, ir->height, 1);
		}

        listener->release(frames);
    }
}

int ofProtonect::closeKinect()
{
  if (bOpened)
  {
      listener->release(frames);
      
      dev->stop();
      dev->close();

      delete listener;
      listener = nullptr;
      
      //Have to delete dev after listener otherwise you will get a crash.
      delete dev;
      dev = nullptr;
      
	  if (undistorted != nullptr) {
		  delete undistorted;
		  undistorted = nullptr;
	  }
	  if (registered != nullptr) {
		  delete registered;
		  registered = nullptr;
	  }
	  if (registration != nullptr) {
		  delete registration;
		  registration = nullptr;
	  }
      bOpened = false;
  }

  return 0;
}

