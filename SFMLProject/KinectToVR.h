#pragma once
#include "stdafx.h"
#include "sfLine.h"

#include <vector>
#include <iostream>
#include <string>
#include <exception>

#include <SFML/Graphics.hpp>
#include <SFML/Window/Mouse.hpp>
#include <SFML/OpenGL.hpp>

// Kinect Includes
#include <Windows.h> //MUST BE BEFORE NUI
#include <ole2.h>

#include <NuiApi.h>
#include <NuiImageCamera.h>
#include <NuiSensor.h>
#include <NuiSkeleton.h>

#include <openvr.h>
#include <openvr_math.h>
#include <vrinputemulator.h>

namespace KinectSettings {
    extern bool isKinectDrawn;
    extern bool isSkeletonDrawn;
    extern bool ignoreInferredPositions;

    extern double trackedPositionOffset[3];
    extern bool userChangingZero;

    extern float g_TrackedBoneThickness;
    extern float g_InferredBoneThickness;
    extern float g_JointThickness;

    extern const int kinectHeight;
    extern const int kinectWidth;
}

namespace SFMLsettings {
    extern int m_window_width;
    extern int m_window_height;
}
# define M_PI           3.14159265358979323846

//Skeleton Tracking
extern sf::Vector2f m_points[NUI_SKELETON_POSITION_COUNT]; // Converted to screen space

extern bool zeroed;
extern vr::HmdVector3_t hmdZero; //TEMP GLOBAL
extern Vector4 kinectZero; //TEMP GLOBAL
extern vr::HmdVector3_t m_HMDposition;
extern vr::HmdQuaternion_t m_HMDquaternion;
extern double kinectToVRScale;


class KinectTrackedDevice {
public:
    KinectTrackedDevice(
        vrinputemulator::VRInputEmulator& inputEmulator,
        NUI_SKELETON_POSITION_INDEX j0,
        NUI_SKELETON_POSITION_INDEX j1,
        bool isKinect);
    ~KinectTrackedDevice() {}
    uint32_t deviceId;
    NUI_SKELETON_POSITION_INDEX joint0;
    NUI_SKELETON_POSITION_INDEX joint1;
    sf::Vector3f hmdRelativePosition;
    bool isKinectDevice;
};
class KinectHandler {
    // A representation of the Kinect elements, it is initialised in its constructor
public:
    HANDLE kinectRGBStream = nullptr;
    INuiSensor* kinectSensor = nullptr;
    GLuint kinectTextureId;    // ID of the texture to contain Kinect RGB Data
                               // BGRA array containing the texture data
    bool initStatus() { return initialised; }

    std::string status_str(HRESULT stat) {
        switch (stat) {
            case S_NUI_INITIALIZING:	return "S_NUI_INITIALIZING The device is connected, but still initializing.";
            case E_NUI_NOTCONNECTED:	return "E_NUI_NOTCONNECTED The device is not connected.";
            case E_NUI_NOTGENUINE:	return "E_NUI_NOTGENUINE The device is not a valid Kinect.";
            case E_NUI_NOTSUPPORTED:	return "E_NUI_NOTSUPPORTED The device is an unsupported model.";
            case E_NUI_INSUFFICIENTBANDWIDTH:	return "E_NUI_INSUFFICIENTBANDWIDTH The device is connected to a hub without the necessary bandwidth requirements.";
            case E_NUI_NOTPOWERED:	return "E_NUI_NOTPOWERED The device is connected, but unpowered.";
            case E_NUI_NOTREADY:	return "E_NUI_NOTREADY There was some other unspecified error.";
        }
    }

    std::unique_ptr<GLubyte[]> kinectImageData
        = std::make_unique<GLubyte[]>(KinectSettings::kinectWidth * KinectSettings::kinectHeight * 4);

    void initialise() {
        try {
            initialised = initKinect(kinectRGBStream, kinectSensor);
            if (!initialised) throw FailedKinectInitialisation;
        }
        catch (std::exception&  e) {
            std::cerr << e.what() << std::endl;
        }
    }

    KinectHandler() {
        initialise();
    }
    ~KinectHandler() {}

private:
    bool initialised;
    bool initKinect(HANDLE& rgbStream, INuiSensor* &sensor) {
        //Get a working Kinect Sensor
        int numSensors = 0;
        if (NuiGetSensorCount(&numSensors) < 0 || numSensors < 1)
            return false;
        if (NuiCreateSensorByIndex(0, &sensor) < 0)
            return false;
        //Initialise Sensor
        sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX
            | NUI_INITIALIZE_FLAG_USES_COLOR
            | NUI_INITIALIZE_FLAG_USES_SKELETON);

        sensor->NuiImageStreamOpen(
            NUI_IMAGE_TYPE_COLOR,               //Depth Camera or RGB Camera?
            NUI_IMAGE_RESOLUTION_640x480,       //Image Resolution
            0,                                  //Image stream flags, e.g. near mode
            2,                                  //Number of frames to buffer
            NULL,                               //Event handle
            &rgbStream);

        sensor->NuiSkeletonTrackingEnable(
            NULL,
            0       // Enable seat support (Upper body only)
        );
        return sensor;
    }
    class FailedKinectInitialisation : public std::exception
    {
        virtual const char* what() const throw()
        {
            return "Failure to initialise the kinect sensor. Is it plugged in and supplied with power?";
        }
    } FailedKinectInitialisation;
};

void drawKinectImageData(KinectHandler& kinect);
void getKinectData(GLubyte* dest, HANDLE& rgbStream, INuiSensor* &sensor);
bool acquireKinectFrame(NUI_IMAGE_FRAME &imageFrame, HANDLE & rgbStream, INuiSensor* &sensor);
INuiFrameTexture* lockKinectPixelData(NUI_IMAGE_FRAME &imageFrame, NUI_LOCKED_RECT &LockedRect);
void copyKinectPixelData(NUI_LOCKED_RECT &LockedRect, GLubyte* dest);
void unlockKinectPixelData(INuiFrameTexture* texture);
void releaseKinectFrame(NUI_IMAGE_FRAME &imageFrame, HANDLE& rgbStream, INuiSensor* &sensor);
void updateTrackersWithSkeletonPosition(vrinputemulator::VRInputEmulator &emulator, std::vector<KinectTrackedDevice> trackers, NUI_SKELETON_FRAME &skeletonFrame);
void updateKinectTrackedDevice(int i, vrinputemulator::VRInputEmulator &emulator,
    KinectTrackedDevice device, const NUI_SKELETON_FRAME & skel,
    vr::HmdVector3_t zeroPos);

void updateKinectTracker(vrinputemulator::VRInputEmulator &emulator, KinectTrackedDevice device);
void toEulerAngle(vr::HmdQuaternion_t q, double& roll, double& pitch, double& yaw);

void zeroAllTracking(NUI_SKELETON_FRAME& skeletonFrame, vr::IVRSystem* &m_sys);
vr::HmdVector3_t getHMDPosition(vr::IVRSystem* &m_sys);
Vector4 zeroKinectPosition(NUI_SKELETON_FRAME &skeletonFrame, int i);
void setKinectToVRMultiplier(NUI_SKELETON_FRAME & skel, int i);

void updateSkeletalData(NUI_SKELETON_FRAME &skeletonFrame, INuiSensor* sensor);
void drawTrackedSkeletons(NUI_SKELETON_FRAME& skeletonFrame, sf::RenderWindow &window);
void DrawSkeleton(const NUI_SKELETON_DATA & skel, sf::RenderWindow &window);
sf::Vector2f SkeletonToScreen(Vector4 skeletonPoint, int _width, int _height);
void DrawBone(const NUI_SKELETON_DATA & skel, NUI_SKELETON_POSITION_INDEX joint0,
    NUI_SKELETON_POSITION_INDEX joint1, sf::RenderWindow &window);
void DrawLine(sf::Vector2f start, sf::Vector2f end, sf::Color colour, float lineThickness, sf::RenderWindow &window);

// Get the quaternion representing the rotation
vr::HmdQuaternion_t GetRotation(vr::HmdMatrix34_t matrix);
// Get the vector representing the position
vr::HmdVector3_t GetPosition(vr::HmdMatrix34_t matrix);

//VR Tracking
uint32_t initTracker(vrinputemulator::VRInputEmulator &inputEmulator, bool connected);
void setTrackerDefaultProperties(uint32_t &deviceId);
void setDeviceProperty(uint32_t deviceId, int dProp, std::string type, std::string value);
void removeAllTrackerProperties(uint32_t &deviceId);
void removeDeviceProperty(uint32_t deviceId, int dProp, std::string type, std::string value);
void destroyTrackers(vrinputemulator::VRInputEmulator& inputEmulator, std::vector<KinectTrackedDevice> trackers);
void setKinectTrackerProperties(uint32_t deviceId);

void processKeyEvents(sf::Event event);
void toggle(bool &b);

void initOpenGL(GLuint &textureId, GLubyte* data);