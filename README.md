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


Notes:
- Kinect needs to have flashed firmware ( this currently needs to be done on Windows 8 ) 
- Requires a USB 3 port on machine. 
- For OS X if you have issues connecting to the device, check in the System Profiler -> USB.  If the Nui Sensor is not listed under SuperSpeed, unplug the power to the device and replug it in, without disconnecting the USB cable. 
- Only tested on OS X though Win / Nix should be possible too with patched libusb ( see: https://github.com/OpenKinect/libfreenect2/blob/master/depends/README.depends.txt ) 
- If you have the ofxKinect ( v1 ) addon in your project remove the ofxKinect libusb lib and use the one that comes with this repo instead. 
- //On OS X if you are not using the example project. Make sure to add OpenCL.framework to the Link Binary With Library Build Phase and also change the line in Project.xcconfig to OTHER_LDFLAGS = $(OF_CORE_LIBS) $(OF_CORE_FRAMEWORKS) -framework OpenCL



Huge thanks to @christiankerl for a lot of the recent changes that made this work well on OS X. 


## Setup

### Windows / Visual Studio

#### KinectV2 drivers

On Windows, to run multiple Kinects at the same time, you need to install custom drivers : libusbK driver

You don't need the Kinect for Windows v2 SDK to build and install libfreenect2, though it doesn't hurt to have it too. You don't need to uninstall the SDK or the driver before doing this procedure.

Install the libusbK backend driver for libusb. Please follow the steps exactly:

- Download Zadig from http://zadig.akeo.ie/.
- Run Zadig and in options, check "List All Devices" and uncheck "Ignore Hubs or Composite Parents"
- Select the "Xbox NUI Sensor (composite parent)" from the drop-down box. (Important: Ignore the "NuiSensor Adaptor" varieties, which are the adapter, NOT the Kinect) The current driver will list usbccgp. USB ID is VID 045E, PID 02C4 or 02D8.
- Select libusbK (v3.0.7.0 or newer) from the replacement driver list.
- Click the "Replace Driver" button. Click yes on the warning about replacing a system driver. (This is because it is a composite parent.)

To uninstall the libusbK driver (and get back the official SDK driver, if installed):

- Open "Device Manager"
- Under "libusbK USB Devices" tree, right click the "Xbox NUI Sensor (Composite Parent)" device and select uninstall.
- Important: Check the "Delete the driver software for this device." checkbox, then click OK.

If you already had the official SDK driver installed and you want to use it:

- In Device Manager, in the Action menu, click "Scan for hardware changes."

This will enumerate the Kinect sensor again and it will pick up the K4W2 SDK driver, and you should be ready to run KinectService.exe again immediately.

You can go back and forth between the SDK driver and the libusbK driver very quickly and easily with these steps.

#### OpenCL

Refer to [this tutorial](https://streamcomputing.eu/blog/2015-03-16/how-to-install-opencl-on-windows/) to install the right OpenCL SDK depending on your GPU (Intel, NVIDIA or AMD)

Don't forget to add following environment variables in Windows system :
- CUDA_INC_PATH with value `C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v7.5\include`
- CUDA_LIB_PATH with value `C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v7.5\lib`