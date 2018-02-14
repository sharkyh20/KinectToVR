#pragma once
#include "stdafx.h"
#include "IKinectHandler.h"
// Kinect V2 - directory local due to my win 7 machine being unsupported for actual install

#include <Kinect.h>
class KinectV2Handler : public IKinectHandler {
public:
    KinectV2Handler() {}
    virtual ~KinectV2Handler() {}

    IKinectSensor* kinectSensor = nullptr;
    IMultiSourceFrameReader* frameReader = nullptr;
    IMultiSourceFrame* multiFrame = nullptr;
    ICoordinateMapper* coordMapper = nullptr;
    BOOLEAN isTracking;

    Joint joints[JointType_Count];

    // TODO: Convert the current opengl stuff into buffer objects
    GLuint colourBufferObjectId;   // color buffer object for the K
                                   //GLuint vertexBufferObjectId;    // unused, as depth data is not to be displayed

    virtual void initialise();
    virtual void drawKinectData();
    virtual void drawKinectImageData();
    virtual void drawTrackedSkeletons();

    virtual void updateSkeletalData();
private:
    bool initKinect();
    void getKinectData();
    void getRGBImageData(IMultiSourceFrame* multiFrame, GLubyte* dest);
};
