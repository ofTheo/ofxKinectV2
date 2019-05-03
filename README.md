ofxKinectV2
===========

An addon for the new Kinect For Windows V2 sensor. 
Based on the excellent work by the https://github.com/OpenKinect/libfreenect2 team ( @JoshBlake @floe and @christiankerl plus others ) 

New: 
- Supports multiple kinects. 
- Optimized for openFrameworks release 0.9.0 
- Uses OpenCL based decoding by default = much faster decoding 
- Less crashes on startup and shutdown 
- Supports both the early beta device and the new retail device. 
- Supports linux64, see detailed installation instructions [here](https://gist.github.com/madelinegannon/10f62caba7184b90ea43a734768e5147).
- Supports linuxarmv7l, see detailed installation instructions [here](https://gist.github.com/madelinegannon/237733e6c114f156b31366f47c1f3d32).


Notes:
- Kinect needs to have flashed firmware ( this currently needs to be done on Windows 8 ) 
- Requires a USB 3 port on machine. 
- For OS X if you have issues connecting to the device, check in the System Profiler -> USB.  If the Nui Sensor is not listed under SuperSpeed, unplug the power to the device and replug it in, without disconnecting the USB cable. 
- Only tested on OS X though Win / Nix should be possible too with patched libusb ( see: https://github.com/OpenKinect/libfreenect2/blob/master/depends/README.depends.txt ) 
- If you have the ofxKinect ( v1 ) addon in your project remove the ofxKinect libusb lib and use the one that comes with this repo instead. 
- //On OS X if you are not using the example project. Make sure to add OpenCL.framework to the Link Binary With Library Build Phase and also change the line in Project.xcconfig to OTHER_LDFLAGS = $(OF_CORE_LIBS) $(OF_CORE_FRAMEWORKS) -framework OpenCL



Huge thanks to @christiankerl for a lot of the recent changes that made this work well on OS X. 
