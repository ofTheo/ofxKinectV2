meta:
	ADDON_NAME = ofxKinectV2
	ADDON_DESCRIPTION = An addon for the new Kinect For Windows V2 sensor
	ADDON_AUTHOR = Theo Watson
	ADDON_TAGS = "kinect" "kinectv2" "libfreenect" "libfreenect2"
	ADDON_URL = https://github.com/ofTheo/ofxKinectV2


common:
	# derines that will be passed to the compiler when including this addon
	ADDON_DEFINES = OF_ADDON_HAS_OFXKINECT_V2
osx:
	ADDON_FRAMEWORKS = OpenCL VideoToolbox
    ADDON_INCLUDES_EXCLUDE  = "libs/libturbojpeg"
    ADDON_INCLUDES_EXCLUDE  += "libs/libturbojpeg/%"
    ADDON_INCLUDES_EXCLUDE  += "libs/opencl"
    ADDON_INCLUDES_EXCLUDE  += "libs/opencl/%"

vs:
	#ADDON_LIBS_EXCLUDE = 
	#ADDON_INCLUDES_EXCLUDE = "libs\libfreenect2\src\vt_rgb_packet_processor.cpp"
	#ADDON_INCLUDES = "C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v8.0\include"

linux64:
	# linux only, any library that should be included in the project using
	# pkg-config
	ADDON_PKG_CONFIG_LIBRARIES = libusb-1.0 OpenCL
	ADDON_LDFLAGS = -L/usr/local/lib/ -lfreenect2 -L/usr/local/cuda/lib64 -lcuda -lcudart
	# when parsing the file system looking for include paths exclude this for all or
	# a specific platform
	#ADDON_INCLUDES_EXCLUDE = libs/libusb/%

linux:
	# linux only, any library that should be included in the project using
	# pkg-config
	ADDON_PKG_CONFIG_LIBRARIES = libusb-1.0 OpenCL

	# when parsing the file system looking for include paths exclude this for all or
	# a specific platform
	ADDON_INCLUDES_EXCLUDE = libs/libusb/%

linuxarmv7l:
	# linux only, any library that should be included in the project using
	# pkg-config
	ADDON_PKG_CONFIG_LIBRARIES = freenect2 cuda-10.0 cudart-10.0 #libusb-1.0 OpenCL
	#ADDON_LDFLAGS = -L/usr/local/lib/ -lfreenect2 -L/usr/local/cuda/lib64 -lcuda -lcudart
	# when parsing the file system looking for include paths exclude this for all or
	# a specific platform
	#ADDON_INCLUDES_EXCLUDE = libs/libusb/%
