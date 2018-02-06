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

// Kinect V2
#include "Kinect2\inc\Kinect.h"

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

    extern const int kinectV2Height;
    extern const int kinectV2Width;
}

namespace SFMLsettings {
    extern int m_window_width;
    extern int m_window_height;
}
# define M_PI           3.14159265358979323846

//Skeleton Tracking Globals----------------
extern sf::Vector2f m_points[NUI_SKELETON_POSITION_COUNT]; // Converted to screen space

extern bool zeroed;
extern vr::HmdVector3_t hmdZero; //TEMP GLOBAL
extern Vector4 kinectZero; //TEMP GLOBAL
extern vr::HmdVector3_t m_HMDposition;
extern vr::HmdQuaternion_t m_HMDquaternion;
extern double kinectToVRScale;
// ----------------------------------------
enum class KinectVersion {
    Version1 = 0,   //AKA Xbox 360/ Windows v1
    Version2    //AKA Xbox One/ Windows v2
};
enum class KinectJointType {
    SpineBase = 0,
    SpineMid = 1,
    Neck = 2,   //Not in v1
    Head = 3,
    ShoulderLeft = 4,
    ElbowLeft = 5,
    WristLeft = 6,
    HandLeft = 7,
    ShoulderRight = 8,
    ElbowRight = 9,
    WristRight = 10,
    HandRight = 11,
    HipLeft = 12,
    KneeLeft = 13,
    AnkleLeft = 14,
    FootLeft = 15,
    HipRight = 16,
    KneeRight = 17,
    AnkleRight = 18,
    FootRight = 19,
    SpineShoulder = 20,
    HandTipLeft = 21, //Not in v1
    ThumbLeft = 22, //Not in v1
    HandTipRight = 23, //Not in v1
    ThumbRight = 24, //Not in v1
};
class KinectJoint {
public:
    // Allows for conversion between v1 and v2 joints without multiple variables
    // Enums for v1 are 'NUI_SKELETON_POSITION_INDEX' in NuiSensor.h
    // Enums for v2 are 'JointType' in Kinect.h

    KinectJoint(KinectJointType type) :
        joint(type)
    {
    }
    ~KinectJoint() {}
    void setJoint(KinectJointType type) {
        joint = type;
    }

    NUI_SKELETON_POSITION_INDEX getV1Representation() {
        //Unfortunately I believe this is required because there are mismatches between v1 and v2 joint IDs
        //Might consider investigating to see if there's a way to shorten this
        switch (joint) {
        case KinectJointType::SpineBase:
            return NUI_SKELETON_POSITION_HIP_CENTER;
        case KinectJointType::SpineMid:
            return NUI_SKELETON_POSITION_SPINE;

        case KinectJointType::Head:
            return NUI_SKELETON_POSITION_HEAD;
        case KinectJointType::ShoulderLeft:
            return NUI_SKELETON_POSITION_SHOULDER_LEFT;
        case KinectJointType::ShoulderRight:
            return NUI_SKELETON_POSITION_SHOULDER_RIGHT;
        case KinectJointType::SpineShoulder:
            return NUI_SKELETON_POSITION_SHOULDER_CENTER;

        case KinectJointType::ElbowLeft:
            return NUI_SKELETON_POSITION_ELBOW_LEFT;
        case KinectJointType::WristLeft:
            return NUI_SKELETON_POSITION_WRIST_LEFT;
        case KinectJointType::HandLeft:
            return NUI_SKELETON_POSITION_HAND_LEFT;

        case KinectJointType::ElbowRight:
            return NUI_SKELETON_POSITION_ELBOW_RIGHT;
        case KinectJointType::WristRight:
            return NUI_SKELETON_POSITION_WRIST_RIGHT;
        case KinectJointType::HandRight:
            return NUI_SKELETON_POSITION_HAND_RIGHT;

        case KinectJointType::HipLeft:
            return NUI_SKELETON_POSITION_HIP_LEFT;
        case KinectJointType::HipRight:
            return NUI_SKELETON_POSITION_HIP_RIGHT;

        case KinectJointType::KneeLeft:
            return NUI_SKELETON_POSITION_KNEE_LEFT;
        case KinectJointType::KneeRight:
            return NUI_SKELETON_POSITION_KNEE_RIGHT;

        case KinectJointType::AnkleLeft:
            return NUI_SKELETON_POSITION_ANKLE_LEFT;
        case KinectJointType::AnkleRight:
            return NUI_SKELETON_POSITION_ANKLE_RIGHT;

        case KinectJointType::FootLeft:
            return NUI_SKELETON_POSITION_FOOT_LEFT;
        case KinectJointType::FootRight:
            return NUI_SKELETON_POSITION_FOOT_RIGHT;

            /*BELOW DO NOT HAVE A 1:1 V1 REPRESENTATION*/
            //refer to the skeleton images from Microsoft for diffs between v1 and 2
            
        case KinectJointType::Neck:
            return NUI_SKELETON_POSITION_SHOULDER_CENTER;
        case KinectJointType::HandTipLeft:
            return NUI_SKELETON_POSITION_HAND_LEFT;
        case KinectJointType::HandTipRight:
            return NUI_SKELETON_POSITION_HAND_RIGHT;
        case KinectJointType::ThumbLeft:
            return NUI_SKELETON_POSITION_HAND_LEFT;
        case KinectJointType::ThumbRight:
            return NUI_SKELETON_POSITION_HAND_RIGHT;

        default:
            std::cerr << "INVALID KinectJointType!!!\n";
            break;
            
        }
    }
    JointType getV2Representation() {
        // Currently, the v2 SDK id's match my jointtype class 1:1
        return static_cast<JointType>(joint);
    }
private:
    KinectJointType joint;
};
class KinectTrackedDevice {
public:
    KinectTrackedDevice(
        vrinputemulator::VRInputEmulator& inputEmulator,
        KinectJointType j0,
        KinectJointType j1,
        bool isKinect, 
        KinectVersion version);
    ~KinectTrackedDevice() {}
    uint32_t deviceId;

    KinectJoint joint0;
    KinectJoint joint1;

    sf::Vector3f hmdRelativePosition;
    bool isKinectRepresentation;

    KinectVersion deviceKinectVersion;
};

class IKinectHandler {
    // Interface base for Kinectv1 and v2
public:
    virtual ~IKinectHandler() {}

    virtual void initialise() = 0;
    virtual bool initStatus() = 0;
    virtual std::string statusString() = 0;

    virtual void drawKinectData() = 0;  // Houses the below draw functions with a check
    virtual void drawKinectImageData() = 0;
    virtual void drawTrackedSkeletons() = 0;

    virtual void updateSkeletalData() = 0;

    KinectVersion kVersion;
    std::unique_ptr<GLubyte[]> kinectImageData; // BGRA array containing the texture data
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

class KinectV1Handler : IKinectHandler{
    // A representation of the Kinect elements, and supports both Kinectv1 and v2
public:
    KinectV1Handler()
    {
        initialise();
    }
    virtual ~KinectV1Handler() {}
    HANDLE kinectRGBStream = nullptr;
    INuiSensor* kinectSensor = nullptr;
    GLuint kinectTextureId;    // ID of the texture to contain Kinect RGB Data

    virtual bool initStatus() { return initialised; }

    virtual std::string statusString(HRESULT stat) {
        switch (stat) {
        case S_OK: return "S_OK";
            case S_NUI_INITIALIZING:	return "S_NUI_INITIALIZING The device is connected, but still initializing.";
            case E_NUI_NOTCONNECTED:	return "E_NUI_NOTCONNECTED The device is not connected.";
            case E_NUI_NOTGENUINE:	return "E_NUI_NOTGENUINE The device is not a valid Kinect.";
            case E_NUI_NOTSUPPORTED:	return "E_NUI_NOTSUPPORTED The device is an unsupported model.";
            case E_NUI_INSUFFICIENTBANDWIDTH:	return "E_NUI_INSUFFICIENTBANDWIDTH The device is connected to a hub without the necessary bandwidth requirements.";
            case E_NUI_NOTPOWERED:	return "E_NUI_NOTPOWERED The device is connected, but unpowered.";
            case E_NUI_NOTREADY:	return "E_NUI_NOTREADY There was some other unspecified error.";
            default: return "Uh Oh undefined kinect error!";
        }
    }

    virtual void initialise() {
        try {
            kVersion = KinectVersion::Version1;
            kinectImageData
                = std::make_unique<GLubyte[]>(KinectSettings::kinectWidth * KinectSettings::kinectHeight * 4);
            initialised = initKinect();
            if (!initialised) throw FailedKinectInitialisation;
        }
        catch (std::exception&  e) {
            std::cerr << e.what() << std::endl;
        }
    }


private:
    bool initialised;
    bool initKinect() {
        //Get a working Kinect Sensor
        int numSensors = 0;
        if (NuiGetSensorCount(&numSensors) < 0 || numSensors < 1)
            return false;
        if (NuiCreateSensorByIndex(0, &kinectSensor) < 0)
            return false;
        //Initialise Sensor
        kinectSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX
            | NUI_INITIALIZE_FLAG_USES_COLOR
            | NUI_INITIALIZE_FLAG_USES_SKELETON);

        kinectSensor->NuiImageStreamOpen(
            NUI_IMAGE_TYPE_COLOR,               //Depth Camera or RGB Camera?
            NUI_IMAGE_RESOLUTION_640x480,       //Image Resolution
            0,                                  //Image stream flags, e.g. near mode
            2,                                  //Number of frames to buffer
            NULL,                               //Event handle
            &kinectRGBStream);

        kinectSensor->NuiSkeletonTrackingEnable(
            NULL,
            0       // Enable seat support (Upper body only)
        );
        return kinectSensor;
    }
    
};
class KinectV2Handler :  IKinectHandler {
public:
    KinectV2Handler() {}
    virtual ~KinectV2Handler() {}

    IKinectSensor* kinectSensor = nullptr;
    IMultiSourceFrameReader* frameReader = nullptr;
    ICoordinateMapper* coordMapper = nullptr;
    
    
    virtual void initialise() {
        try {
            kVersion = KinectVersion::Version2;
            kinectImageData = std::make_unique<GLubyte[]>(KinectSettings::kinectV2Width * KinectSettings::kinectV2Height * 4);
            initialised = initKinect();
            if (!initialised) throw FailedKinectInitialisation;
        }
        catch (std::exception& e) {
            std::cerr << e.what() << std::endl;
        }
    }
    virtual bool initStatus() { return initialised; }


    virtual void updateSkeletalData() {
        IBodyFrame* bodyFrame = nullptr;
        IBodyFrameReference* frameRef = nullptr;
        frameReader->get_Body
    }
private:
    bool initKinect() {
        if (FAILED(GetDefaultKinectSensor(&kinectSensor))) {
            return false;
        }
        if (kinectSensor) {
            kinectSensor->get_CoordinateMapper(&coordMapper);

            kinectSensor->Open();
            kinectSensor->OpenMultiSourceFrameReader(
                FrameSourceTypes::FrameSourceTypes_Depth
                | FrameSourceTypes::FrameSourceTypes_Color
                | FrameSourceTypes::FrameSourceTypes_Body,
                &frameReader);
            return frameReader;
        }
        else {
            return false;
        }
    }
    void getKinectData() {
        IMultiSourceFrame* multiFrame = nullptr;
        if (SUCCEEDED(frameReader->AcquireLatestFrame(&multiFrame))) {
            GLubyte* ptr;
            //TODO Follow on from getKinectData https://github.com/kyzyx/Tutorials/blob/master/Kinect2SDK/4_SkeletalTracking/main.cpp
            //NEED TO GET A BETTER OPEN GL THAT SUPPORTS VBOs!!!!
        }
    }
    void getKinectImageData() {
        IColorFrame* frame = nullptr;
        if (SUCCEEDED(colorFrameReader->AcquireLatestFrame(&frame))) {
            frame->CopyConvertedFrameDataToArray(KinectSettings::kinectV2Width*KinectSettings::kinectV2Height * 4, kinectImageData.get(), ColorImageFormat_Bgra);
        }
        if (frame) frame->Release();
    }
};

void drawKinectImageData(KinectHandler& kinect);
void getKinectData(GLubyte* dest, KinectHandler& kinect);
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

void initOpenGL(KinectHandler& kinect);