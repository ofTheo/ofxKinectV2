meta:
	ADDON_NAME = ofxKinectV2
	ADDON_DESCRIPTION = An addon for the new Kinect For Windows V2 sensor
	ADDON_AUTHOR = Theo Watson
	ADDON_TAGS = "kinect" "kinectv2" "libfreenect" "libfreenect2"
	ADDON_URL = https://github.com/ofTheo/ofxKinectV2

osx:
	ADDON_FRAMEWORKS = OpenCL VideoToolbox

linux64:
	# linux only, any library that should be included in the project using
	# pkg-config
	ADDON_PKG_CONFIG_LIBRARIES = libusb-1.0 OpenCL

	# when parsing the file system looking for include paths exclude this for all or
	# a specific platform
	ADDON_INCLUDES_EXCLUDE = libs/libusb/%

linux:
	# linux only, any library that should be included in the project using
	# pkg-config
	ADDON_PKG_CONFIG_LIBRARIES = libusb-1.0 OpenCL

	# when parsing the file system looking for include paths exclude this for all or
	# a specific platform
	ADDON_INCLUDES_EXCLUDE = libs/libusb/%
