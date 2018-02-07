#pragma once
#include "stdafx.h"
#include "sfLine.h"

#include <vector>
#include <iostream>
#include <string>
#include <exception>

//#include <SFML/Graphics.hpp>
#include <glew.h>
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

    virtual void initOpenGL() = 0;
    virtual void initialise() = 0;
    virtual bool initStatus() = 0;
    virtual std::string statusString(HRESULT stat) = 0;

    virtual void update() = 0;

    virtual void drawKinectData() = 0;  // Houses the below draw functions with a check
    virtual void drawKinectImageData() = 0;
    virtual void drawTrackedSkeletons() = 0;

    GLuint kinectTextureId; //TEMPORARY!!
    sf::RenderWindow* drawingWindow;    //TEMPORARY!!!
    BOOLEAN isTracking;
    KinectVersion kVersion;
    std::unique_ptr<GLubyte[]> kinectImageData; // array containing the texture data
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
    KinectV1Handler(sf::RenderWindow &win)
    {
        drawingWindow = &win;
        initialise();
        initOpenGL();
    }
    virtual ~KinectV1Handler() {}
    HANDLE kinectRGBStream = nullptr;
    INuiSensor* kinectSensor = nullptr;
    GLuint kinectTextureId;    // ID of the texture to contain Kinect RGB Data
    
    virtual void initOpenGL() {
        int width = 0, height = 0;
        if (kVersion == KinectVersion::Version1) {
            width = KinectSettings::kinectWidth;
            height = KinectSettings::kinectHeight;
        }
        else if (kVersion == KinectVersion::Version2) {
            width = KinectSettings::kinectV2Width;
            height = KinectSettings::kinectV2Height;
        }
        // Initialize textures
        glGenTextures(1, &kinectTextureId);
        glBindTexture(GL_TEXTURE_2D, kinectTextureId);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height,
            0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, (GLvoid*)kinectImageData.get());
        glBindTexture(GL_TEXTURE_2D, 0);

        // OpenGL setup
        glClearColor(1, 0, 0, 0);
        glClearDepth(1.0f);
        glEnable(GL_TEXTURE_2D);

        // Camera setup
        glViewport(0, 0, SFMLsettings::m_window_width, SFMLsettings::m_window_height);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(0, SFMLsettings::m_window_width, SFMLsettings::m_window_height, 0, 1, -1);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
    }

    NUI_SKELETON_FRAME skeletonFrame = { 0 };

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
                = std::make_unique<GLubyte[]>(KinectSettings::kinectWidth * KinectSettings::kinectHeight * 4);  // BGRA
            initialised = initKinect();
            if (!initialised) throw FailedKinectInitialisation;
        }
        catch (std::exception&  e) {
            std::cerr << e.what() << std::endl;
        }
    }

    virtual void update() {
        getKinectRGBData();
        updateSkeletalData();
    }

    virtual void drawKinectData() {
        if (KinectSettings::isKinectDrawn) {
            drawKinectImageData();
        }
        if (KinectSettings::isSkeletonDrawn) {
            drawTrackedSkeletons();
        }
    };
    virtual void drawKinectImageData() {

        glBindTexture(GL_TEXTURE_2D, kinectTextureId);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, SFMLsettings::m_window_width, SFMLsettings::m_window_height, GL_BGRA_EXT, GL_UNSIGNED_BYTE, (GLvoid*)kinectImageData.get());
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glBegin(GL_QUADS);
        glTexCoord2f(0.0f, 0.0f);
        glVertex3f(0, 0, 0);
        glTexCoord2f(1.0f, 0.0f);
        glVertex3f(SFMLsettings::m_window_width, 0, 0);
        glTexCoord2f(1.0f, 1.0f);
        glVertex3f(SFMLsettings::m_window_width, SFMLsettings::m_window_height, 0.0f);
        glTexCoord2f(0.0f, 1.0f);
        glVertex3f(0, SFMLsettings::m_window_height, 0.0f);

        glEnd();
    };
    virtual void drawTrackedSkeletons() {
        for (int i = 0; i < NUI_SKELETON_POSITION_COUNT; ++i) {
            m_points[i] = sf::Vector2f(0.0f, 0.0f);
        }
        for (int i = 0; i < NUI_SKELETON_COUNT; ++i) {
            NUI_SKELETON_TRACKING_STATE trackingState = skeletonFrame.SkeletonData[i].eTrackingState;

            if (NUI_SKELETON_TRACKED == trackingState)
            {
                if (KinectSettings::isSkeletonDrawn) {
                    drawingWindow->pushGLStates();
                    drawingWindow->resetGLStates();

                    DrawSkeleton(skeletonFrame.SkeletonData[i], *drawingWindow);

                    drawingWindow->popGLStates();
                }

            }
            else if (NUI_SKELETON_POSITION_ONLY == trackingState) {
                //ONLY CENTER POINT TO DRAW
                if (KinectSettings::isSkeletonDrawn) {
                    sf::CircleShape circle(KinectSettings::g_JointThickness, 30);
                    circle.setRadius(KinectSettings::g_JointThickness);
                    circle.setPosition(SkeletonToScreen(skeletonFrame.SkeletonData[i].Position, SFMLsettings::m_window_width, SFMLsettings::m_window_height));
                    circle.setFillColor(sf::Color::Yellow);

                    drawingWindow->pushGLStates();
                    drawingWindow->resetGLStates();

                    drawingWindow->draw(circle);

                    drawingWindow->popGLStates();
                }
            }
        }
    };

    

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
    void getKinectRGBData() {
        NUI_IMAGE_FRAME imageFrame{};
        NUI_LOCKED_RECT LockedRect{};
        if (acquireKinectFrame(imageFrame, kinectRGBStream, kinectSensor)) {
            return;
        }
        INuiFrameTexture* texture = lockKinectPixelData(imageFrame, LockedRect);
        copyKinectPixelData(LockedRect, kinectImageData.get());
        unlockKinectPixelData(texture);

        releaseKinectFrame(imageFrame, kinectRGBStream, kinectSensor);
    }
    bool acquireKinectFrame(NUI_IMAGE_FRAME &imageFrame, HANDLE & rgbStream, INuiSensor* &sensor)
    {
        return (sensor->NuiImageStreamGetNextFrame(rgbStream, 1, &imageFrame) < 0);
    }
    INuiFrameTexture* lockKinectPixelData(NUI_IMAGE_FRAME &imageFrame, NUI_LOCKED_RECT &LockedRect)
    {
        INuiFrameTexture* texture = imageFrame.pFrameTexture;
        texture->LockRect(0, &LockedRect, NULL, 0);
        return imageFrame.pFrameTexture;
    }
    void copyKinectPixelData(NUI_LOCKED_RECT &LockedRect, GLubyte* dest)
    {
        int bytesInFrameRow = LockedRect.Pitch;
        if (bytesInFrameRow != 0) {
            const BYTE* curr = (const BYTE*)LockedRect.pBits;
            const BYTE* dataEnd = curr + (KinectSettings::kinectWidth*KinectSettings::kinectHeight) * 4;

            while (curr < dataEnd) {
                *dest++ = *curr++;
            }
        }
    }
    void unlockKinectPixelData(INuiFrameTexture* texture)
    {
        texture->UnlockRect(0);
    }
    void releaseKinectFrame(NUI_IMAGE_FRAME &imageFrame, HANDLE& rgbStream, INuiSensor* &sensor)
    {
        sensor->NuiImageStreamReleaseFrame(rgbStream, &imageFrame);
    }

    void updateSkeletalData() {
        if (kinectSensor->NuiSkeletonGetNextFrame(0, &skeletonFrame) >= 0) {
            kinectSensor->NuiTransformSmooth(&skeletonFrame, NULL);   //Smooths jittery tracking
        }
        return;
    };
    void DrawSkeleton(const NUI_SKELETON_DATA & skel, sf::RenderWindow &window) {
        for (int i = 0; i < NUI_SKELETON_POSITION_COUNT; ++i) {
            m_points[i] = SkeletonToScreen(skel.SkeletonPositions[i], SFMLsettings::m_window_width, SFMLsettings::m_window_height);
            std::cerr << "m_points[" << i << "] = " << m_points[i].x << ", " << m_points[i].y << std::endl;
            // Same with the other cerr, without this, the skeleton flickers
        }
        // Render Torso
        DrawBone(skel, NUI_SKELETON_POSITION_HEAD, NUI_SKELETON_POSITION_SHOULDER_CENTER, window);
        DrawBone(skel, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_LEFT, window);
        DrawBone(skel, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_RIGHT, window);
        DrawBone(skel, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SPINE, window);
        DrawBone(skel, NUI_SKELETON_POSITION_SPINE, NUI_SKELETON_POSITION_HIP_CENTER, window);
        DrawBone(skel, NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_HIP_LEFT, window);
        DrawBone(skel, NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_HIP_RIGHT, window);

        // Left Arm
        DrawBone(skel, NUI_SKELETON_POSITION_SHOULDER_LEFT, NUI_SKELETON_POSITION_ELBOW_LEFT, window);
        DrawBone(skel, NUI_SKELETON_POSITION_ELBOW_LEFT, NUI_SKELETON_POSITION_WRIST_LEFT, window);
        DrawBone(skel, NUI_SKELETON_POSITION_WRIST_LEFT, NUI_SKELETON_POSITION_HAND_LEFT, window);

        // Right Arm
        DrawBone(skel, NUI_SKELETON_POSITION_SHOULDER_RIGHT, NUI_SKELETON_POSITION_ELBOW_RIGHT, window);
        DrawBone(skel, NUI_SKELETON_POSITION_ELBOW_RIGHT, NUI_SKELETON_POSITION_WRIST_RIGHT, window);
        DrawBone(skel, NUI_SKELETON_POSITION_WRIST_RIGHT, NUI_SKELETON_POSITION_HAND_RIGHT, window);

        // Left Leg
        DrawBone(skel, NUI_SKELETON_POSITION_HIP_LEFT, NUI_SKELETON_POSITION_KNEE_LEFT, window);
        DrawBone(skel, NUI_SKELETON_POSITION_KNEE_LEFT, NUI_SKELETON_POSITION_ANKLE_LEFT, window);
        DrawBone(skel, NUI_SKELETON_POSITION_ANKLE_LEFT, NUI_SKELETON_POSITION_FOOT_LEFT, window);

        // Right Leg
        DrawBone(skel, NUI_SKELETON_POSITION_HIP_RIGHT, NUI_SKELETON_POSITION_KNEE_RIGHT, window);
        DrawBone(skel, NUI_SKELETON_POSITION_KNEE_RIGHT, NUI_SKELETON_POSITION_ANKLE_RIGHT, window);
        DrawBone(skel, NUI_SKELETON_POSITION_ANKLE_RIGHT, NUI_SKELETON_POSITION_FOOT_RIGHT, window);


        // Draw the joints in a different color
        for (int i = 0; i < NUI_SKELETON_POSITION_COUNT; ++i)
        {
            sf::CircleShape circle{};
            circle.setRadius(KinectSettings::g_JointThickness);
            circle.setPosition(m_points[i]);

            if (skel.eSkeletonPositionTrackingState[i] == NUI_SKELETON_POSITION_INFERRED)
            {
                circle.setFillColor(sf::Color::Red);
                window.draw(circle);
            }
            else if (skel.eSkeletonPositionTrackingState[i] == NUI_SKELETON_POSITION_TRACKED)
            {
                circle.setFillColor(sf::Color::Yellow);
                window.draw(circle);
            }
        }

    }
    sf::Vector2f SkeletonToScreen(Vector4 skeletonPoint, int _width, int _height) {
        LONG x = 0, y = 0;
        USHORT depth = 0;

        // Calculate the skeleton's position on the screen
        // NuiTransformSkeletonToDepthImage returns coordinates in NUI_IMAGE_RESOLUTION_320x240 space
        NuiTransformSkeletonToDepthImage(skeletonPoint, &x, &y, &depth);

        float screenPointX = static_cast<float>(x * _width) / 320;
        float screenPointY = static_cast<float>(y * _height) / 240;
        std::cerr << "x = " << x << " ScreenX = " << screenPointX << " y = " << y << " ScreenY = " << screenPointY << std::endl;

        // The skeleton constantly flickers and drops out without the cerr command...
        return sf::Vector2f(screenPointX, screenPointY);
    }
    void DrawBone(const NUI_SKELETON_DATA & skel, NUI_SKELETON_POSITION_INDEX joint0,
        NUI_SKELETON_POSITION_INDEX joint1, sf::RenderWindow &window)
    {
        NUI_SKELETON_POSITION_TRACKING_STATE joint0State = skel.eSkeletonPositionTrackingState[joint0];
        NUI_SKELETON_POSITION_TRACKING_STATE joint1State = skel.eSkeletonPositionTrackingState[joint1];

        // If we can't find either of these joints, exit
        if (joint0State == NUI_SKELETON_POSITION_NOT_TRACKED || joint1State == NUI_SKELETON_POSITION_NOT_TRACKED)
        {
            return;
        }

        // Don't draw if both points are inferred
        if (joint0State == NUI_SKELETON_POSITION_INFERRED && joint1State == NUI_SKELETON_POSITION_INFERRED)
        {
            return;
        }
        // Assume all bones are inferred unless BOTH joints are tracked
        if (joint0State == NUI_SKELETON_POSITION_TRACKED && joint1State == NUI_SKELETON_POSITION_TRACKED)
        {
            DrawLine(m_points[joint0], m_points[joint1], sf::Color::Green, KinectSettings::g_TrackedBoneThickness, window);
        }
        else
        {
            DrawLine(m_points[joint0], m_points[joint1], sf::Color::Red, KinectSettings::g_InferredBoneThickness, window);
        }
    }
    void DrawLine(sf::Vector2f start, sf::Vector2f end, sf::Color colour, float lineThickness, sf::RenderWindow &window) {
        sfLine line(start, end);
        line.setColor(colour);
        line.setThickness(lineThickness);
        window.draw(line);
    }
};
class KinectV2Handler : IKinectHandler {
public:
    KinectV2Handler() {}
    virtual ~KinectV2Handler() {}

    IKinectSensor* kinectSensor = nullptr;
    IMultiSourceFrameReader* frameReader = nullptr;
    IMultiSourceFrame* multiFrame = nullptr;
    ICoordinateMapper* coordMapper = nullptr;

    Joint joints[JointType_Count];
    
    // TODO: Convert the current opengl stuff into buffer objects
    GLuint colourBufferObjectId;   // color buffer object for the K
    //GLuint vertexBufferObjectId;    // unused, as depth data is not to be displayed

    virtual void initialise() {
        try {
            kVersion = KinectVersion::Version2;
            kinectImageData = std::make_unique<GLubyte[]>(KinectSettings::kinectV2Width * KinectSettings::kinectV2Height * 4);  //RGBA
            initialised = initKinect();
            if (!initialised) throw FailedKinectInitialisation;
        }
        catch (std::exception& e) {
            std::cerr << e.what() << std::endl;
        }
    }
    virtual bool initStatus() { return initialised; }

    virtual void drawKinectData() {
        if (KinectSettings::isKinectDrawn) {
            drawKinectImageData();
        }
        if (KinectSettings::isSkeletonDrawn) {
            drawTrackedSkeletons();
        }
    }
    virtual void drawKinectImageData() {

    }
    virtual void drawTrackedSkeletons() {

    }

    virtual void updateSkeletalData() {
        IBodyFrame* bodyFrame = nullptr;
        IBodyFrameReference* frameRef = nullptr;
        multiFrame->get_BodyFrameReference(&frameRef);
        frameRef->AcquireFrame(&bodyFrame);
        if (frameRef) frameRef->Release();

        if (!bodyFrame) return;

        IBody* bodies[BODY_COUNT];
        bodyFrame->GetAndRefreshBodyData(BODY_COUNT, bodies);
        for (int i = 0; i < BODY_COUNT; i++) {
            bodies[i]->get_IsTracked(&isTracking);
            if (isTracking) {
                bodies[i]->GetJoints(JointType_Count, joints);
                break;
            }
        }
        if (bodyFrame) bodyFrame->Release();
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
        if (SUCCEEDED(frameReader->AcquireLatestFrame(&multiFrame))) {
            GLubyte* ptr;
            glBindBuffer(GL_ARRAY_BUFFER, colourBufferObjectId);
            ptr = (GLubyte*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);

            if (ptr) {
                getRGBImageData(multiFrame, ptr);
            }
            glUnmapBuffer(GL_ARRAY_BUFFER);

            updateSkeletalData();
        }
        if (multiFrame) multiFrame->Release();
    }
    void getRGBImageData(IMultiSourceFrame* multiFrame, GLubyte* dest) {
        IColorFrame* colorFrame = nullptr;
        IColorFrameReference* frameRef = nullptr;

        multiFrame->get_ColorFrameReference(&frameRef);
        frameRef->AcquireFrame(&colorFrame);
        if (frameRef) frameRef->Release();

        if (!colorFrame) return;

        //Get data from current frame
        colorFrame->CopyConvertedFrameDataToArray(KinectSettings::kinectV2Width*KinectSettings::kinectV2Height * 4, kinectImageData.get(), ColorImageFormat_Rgba);

        if (colorFrame) colorFrame->Release();
    }
};
/*/
void drawKinectImageData(KinectHandler& kinect);
void getKinectData(GLubyte* dest, KinectHandler& kinect);
bool acquireKinectFrame(NUI_IMAGE_FRAME &imageFrame, HANDLE & rgbStream, INuiSensor* &sensor);
INuiFrameTexture* lockKinectPixelData(NUI_IMAGE_FRAME &imageFrame, NUI_LOCKED_RECT &LockedRect);
void copyKinectPixelData(NUI_LOCKED_RECT &LockedRect, GLubyte* dest);
void unlockKinectPixelData(INuiFrameTexture* texture);
void releaseKinectFrame(NUI_IMAGE_FRAME &imageFrame, HANDLE& rgbStream, INuiSensor* &sensor);
*/
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

//void updateSkeletalData(NUI_SKELETON_FRAME &skeletonFrame, INuiSensor* sensor);
//void drawTrackedSkeletons(NUI_SKELETON_FRAME& skeletonFrame, sf::RenderWindow &window);
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

void initOpenGL(IKinectHandler& kinect);