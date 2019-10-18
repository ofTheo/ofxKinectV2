ofxKinectV2
===========

An addon for the new Kinect For Windows V2 sensor. 
Based on the excellent work by the https://github.com/OpenKinect/libfreenect2 team ( @JoshBlake @floe and @christiankerl plus others ) 

New: 
- Adds support for > 8m if needed with ofxKinectV2::Settings 
- Adds ofParameter for irExposure ( brightness for IR image ). 
- Supports multiple kinects. 
- Optimized for openFrameworks release 0.10.1 
- Uses OpenCL based decoding by default = much faster decoding 
- Less crashes on startup and shutdown 
- Supports both the early beta device and the new retail device. 
- Supports `linux64`, see detailed installation instructions [here](https://gist.github.com/madelinegannon/10f62caba7184b90ea43a734768e5147).
- Supports `linuxarmv7l`, see detailed installation instructions [here](https://gist.github.com/madelinegannon/237733e6c114f156b31366f47c1f3d32).

Known Issues:
- There is a bug with the OpenCLFrame destructor which casues a crash if the listener is deleted in ofProtonect::closeKinect. So we are not deleting the listener currently. This might cause a very small memory leak if you are opening and closing the Kinect many times. More info here: https://github.com/OpenKinect/libfreenect2/issues/867 

Notes:
- Requires a USB 3 port on machine. 
- For OS X if you have issues connecting to the device, check in the System Profiler -> USB.  If the Nui Sensor is not listed under SuperSpeed, unplug the power to the device and replug it in, without disconnecting the USB cable. 
- Only tested on OS X / Windows10 / Nix 
- If you have the ofxKinect ( v1 ) addon in your project remove the ofxKinect libusb lib and use the one that comes with this repo instead. 
- //On OS X if you are not using the example project. Make sure to add OpenCL.framework and VideoToolbox.framework to the Link Binary With Library Build Phase and also change the line in Project.xcconfig to OTHER_LDFLAGS = $(OF_CORE_LIBS) $(OF_CORE_FRAMEWORKS) -framework OpenCL -framework VideoToolbox

### Windows 10 USB Setup.
- Use zadig to install libusbK drivers onto the Kinect V2 device
- Install the libusbK backend driver for libusb. 

Please follow the steps exactly:

    1. Download Zadig from http://zadig.akeo.ie/.
    2. Run Zadig and in options, check "List All Devices" and uncheck "Ignore Hubs or Composite Parents"
    3. Select the "Xbox NUI Sensor (composite parent)" from the drop-down box. (Important: Ignore the "NuiSensor Adaptor" varieties, which are the adapter, NOT the Kinect) The current driver will list usbccgp. USB ID is VID 045E, PID 02C4 or 02D8.
    4. Select libusbK (v3.0.7.0 or newer) from the replacement driver list.
    5. Click the "Replace Driver" button. Click yes on the warning about replacing a system driver. (This is because it is a composite parent.)

    To uninstall the libusbK driver (and get back the official SDK driver, if installed):

    1. Open "Device Manager"
    2. Under "libusbK USB Devices" tree, right click the "Xbox NUI Sensor (Composite Parent)" device and select uninstall.
    3. Important: Check the "Delete the driver software for this device." checkbox, then click OK.

    If you already had the official SDK driver installed and you want to use it:

    4. In Device Manager, in the Action menu, click "Scan for hardware changes."

    This will enumerate the Kinect sensor again and it will pick up the K4W2 SDK driver, and you should be ready to run KinectService.exe again immediately.

    You can go back and forth between the SDK driver and the libusbK driver very quickly and easily with these steps.


Huge thanks to @christiankerl for a lot of the recent changes that made this work well on OS X. 
