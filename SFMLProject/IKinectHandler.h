#pragma once
#include "stdafx.h"

#include "KinectTrackedDevice.h"

#include <string>
#include <vector>
#include <vrinputemulator.h>
#include <SFML/Graphics/RenderWindow.hpp>
#include <glew.h>

// Kinect V1
#include <Windows.h> //MUST BE BEFORE NUI
#include <ole2.h>

#include <NuiApi.h>
#include <NuiImageCamera.h>
#include <NuiSensor.h>
#include <NuiSkeleton.h>

// Kinect V2 - directory local due to my win 7 machine being unsupported for actual install
#include "Kinect2\inc\Kinect.h"

class IKinectHandler {
    // Interface base for Kinectv1 and v2
public:
    virtual ~IKinectHandler() {}

    virtual void initOpenGL() = 0;
    virtual void initialise() = 0;

    virtual std::string statusString(HRESULT stat) = 0;

    virtual void update() = 0;

    virtual void drawKinectData() = 0;  // Houses the below draw functions with a check
    virtual void drawKinectImageData() = 0;
    virtual void drawTrackedSkeletons() = 0;

    virtual void zeroAllTracking(vr::IVRSystem* &m_sys) = 0;
    virtual void updateTrackersWithSkeletonPosition(
        vrinputemulator::VRInputEmulator &emulator,
        std::vector<KinectTrackedDevice> trackers
    ) = 0;

    bool isInitialised() { return initialised; }
    bool isZeroed() { return zeroed; }

    GLuint kinectTextureId; //TEMPORARY!!
    sf::RenderWindow* drawingWindow;    //TEMPORARY!!!
    BOOLEAN isTracking; // Consider seperating into solely  v2
    KinectVersion kVersion;
    std::unique_ptr<GLubyte[]> kinectImageData; // array containing the texture data
    Vector4 kinectZero{ 0,0,0 };
    bool zeroed = false;
    vr::HmdVector3_t trackedPositionVROffset{ 0,0,0 };
protected:
    bool initialised;

    class FailedKinectInitialisation : public std::exception
    {
        virtual const char* what() const throw()
        {
            return "Failure to initialise the kinect sensor. Is it plugged in and supplied with power?";
        }
    } FailedKinectInitialisation;
private:
};