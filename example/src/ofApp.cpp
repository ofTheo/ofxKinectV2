#include "ofApp.h"

//NOTE: if you are unable to connect to your device on OS X, try unplugging and replugging in the power, while leaving the USB connected.
//ofxKinectV2 will only work if the NUI Sensor shows up in the Superspeed category of the System Profiler in the USB section.

//On OS X if you are not using the example project. Make sure to add OpenCL.framework to the Link Binary With Library Build Phase 
//and change the line in Project.xcconfig to OTHER_LDFLAGS = $(OF_CORE_LIBS) $(OF_CORE_FRAMEWORKS) -framework OpenCL


void ofApp::setup()
{
    // Uncomment for verbose info from libfreenect2
    ofSetLogLevel(OF_LOG_VERBOSE);

    ofBackground(0);

    //see how many devices we have.
    ofxKinectV2 tmp;
    std::vector <ofxKinectV2::KinectDeviceInfo> deviceList = tmp.getDeviceList();
    
    //allocate for this many devices
    kinects.resize(deviceList.size());
    texDepth.resize(kinects.size());
    texRGB.resize(kinects.size());
    texRGBRegistered.resize(kinects.size());
    texIR.resize(kinects.size());

    panel.setup("", "settings.xml", 10, 100);
    
	ofxKinectV2::Settings ksettings;
	ksettings.enableRGB = true;
	ksettings.enableIR = true;
	ksettings.enableDepth = true;
	ksettings.enableRGBRegistration = true;
	ksettings.config.MinDepth = 0.5;
	ksettings.config.MaxDepth = 8.0;
    
    //****************************************//
    // example settings for just getting the depth stream //
//    ksettings.enableRGB = false;
//    ksettings.enableRGBRegistration = false;
//    ksettings.enableIR = false;
//    // use enableDepthRegistration if you would like to use getWorldCoordinateAt
//    ksettings.enableDepthRegistration = true;
    //****************************************//
    
    // Note you don't have to use ofxKinectV2 as a shared pointer, but if you
    // want to have it in a vector ( ie: for multuple ) it needs to be.
    for(int d = 0; d < kinects.size(); d++) {
        kinects[d] = std::make_shared<ofxKinectV2>();
        kinects[d]->open(deviceList[d].serial, ksettings);
        panel.add(kinects[d]->params);
    }
    panel.add(pointSize);

    panel.loadFromFile("settings.xml");

}


void ofApp::update()
{
    for (int d = 0; d < kinects.size(); d++)
    {
        kinects[d]->update();
        
        if (kinects[d]->isFrameNew())
        {
            if( kinects[d]->isRGBEnabled()) texRGB[d].loadData(kinects[d]->getPixels());
            if(kinects[d]->getRegisteredPixels().getWidth() > 10) texRGBRegistered[d].loadData(kinects[d]->getRegisteredPixels());
            if(kinects[d]->isIREnabled()) texIR[d].loadData(kinects[d]->getIRPixels());
            if(kinects[d]->isDepthEnabled() ) texDepth[d].loadData(kinects[d]->getDepthPixels());

            if (showPointCloud)
            {
                pointCloud.clear();
                pointCloud.setMode(OF_PRIMITIVE_POINTS);
                ofSetPointSize(pointSize.get());
                for (std::size_t x = 0; x < texRGBRegistered[d].getWidth(); x++)
                {
                    for (std::size_t y = 0; y < texRGBRegistered[d].getHeight(); y++)
                    {
                        pointCloud.addVertex(kinects[d]->getWorldCoordinateAt(x, y));
                        pointCloud.addColor(kinects[d]->getRegisteredPixels().getColor(x, y));
                    }
                }
            }
        }
    }
}


void ofApp::draw()
{
    if (!showPointCloud && currentKinect < texRGB.size())
    {
        drawTextureAtRowAndColumn("RGB Pixels",
                                  texRGB[currentKinect],
                                  0, 0);

        drawTextureAtRowAndColumn("RGB Pixels, Registered",
                                  texRGBRegistered[currentKinect],
                                  1, 0);

        drawTextureAtRowAndColumn("Depth Pixels, Mapped",
                                  texDepth[currentKinect],
                                  1, 1);

        drawTextureAtRowAndColumn("IR Pixels, Mapped",
                                  texIR[currentKinect],
                                  0, 1);
    }
    else
    {
        cam.begin();
        ofPushMatrix();
        ofScale(100, -100, -100);
        pointCloud.draw();
        ofPopMatrix();
        cam.end();
    }
    
    if( kinects.size() < 1 ) {
        ofDrawBitmapStringHighlight( "No Kinects Detected", 40, 40 );
    }
        
    panel.draw();
}


void ofApp::keyPressed(int key)
{
    if (key == ' ')
    {
        currentKinect = (currentKinect + 1) % kinects.size();
    }
    else if (key == 'p')
    {
        showPointCloud = !showPointCloud;
    }
}


void ofApp::drawTextureAtRowAndColumn(const std::string& title,
                                      const ofTexture& tex,
                                      int row,
                                      int column)
{
    float displayWidth = ofGetWidth() / numColumns;
    float displayHeight = ofGetHeight() / numRows;
    
    ofRectangle targetRectangle(row * displayWidth,
                                column * displayHeight,
                                displayWidth,
                                displayHeight);
    
    ofNoFill();
    ofSetColor(ofColor::gray);
    ofDrawRectangle(targetRectangle);
    
    ofFill();
    ofSetColor(255);
    if (tex.isAllocated())
    {
        ofRectangle textureRectangle(0, 0, tex.getWidth(), tex.getHeight());
        
        // Scale the texture rectangle to its new location and size.
        textureRectangle.scaleTo(targetRectangle);
        tex.draw(textureRectangle);
    }
    else
    {
        ofDrawBitmapStringHighlight("...",
                                    targetRectangle.getCenter().x,
                                    targetRectangle.getCenter().y);
    }
    
    ofDrawBitmapStringHighlight(title,
                                targetRectangle.getPosition() + glm::vec3(14, 20, 0));
}
