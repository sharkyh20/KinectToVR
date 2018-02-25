#include "stdafx.h"
#include "KinectV1Handler.h"

#include "glew.h"
#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <KinectSettings.h>
#include <VRHelper.h>
#include <sfLine.h>
#include <iostream>
#include <KinectJoint.h>


 void KinectV1Handler::initOpenGL() {
    int width = 0, height = 0;
    if (kVersion == KinectVersion::Version1) {
        width = KinectSettings::kinectWidth;
        height = KinectSettings::kinectHeight;
    }
    else if (kVersion == KinectVersion::Version2) {
        width = KinectSettings::kinectV2Width;
        height = KinectSettings::kinectV2Height;
    }   // REMOVE THIS INTO KINECT V2 IMPL
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


 HRESULT KinectV1Handler::getStatusResult()
 {
     return kinectSensor->NuiStatus();
 }

 std::string KinectV1Handler::statusResultString(HRESULT stat) {
    switch (stat) {
    case S_OK: return "S_OK";
    case S_NUI_INITIALIZING:	return "S_NUI_INITIALIZING The device is connected, but still initializing.";
    case E_NUI_NOTCONNECTED:	return "E_NUI_NOTCONNECTED The device is not connected.";
    case E_NUI_NOTGENUINE:	return "E_NUI_NOTGENUINE The device is not a valid Kinect.";
    case E_NUI_NOTSUPPORTED:	return "E_NUI_NOTSUPPORTED The device is an unsupported model.";
    case E_NUI_INSUFFICIENTBANDWIDTH:	return "E_NUI_INSUFFICIENTBANDWIDTH The device is connected to a hub without the necessary bandwidth requirements.";
    case E_NUI_NOTPOWERED:	return "E_NUI_NOTPOWERED The device is connected, but unpowered.";
    case E_NUI_NOTREADY:	return "E_NUI_NOTREADY There was some other unspecified error.";
    default: return "Uh Oh undefined kinect error! " + std::to_string(stat);
    }
}

 void KinectV1Handler::initialise() {
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

 void KinectV1Handler::update() {
    if (isInitialised()) {
        HRESULT kinectStatus = kinectSensor->NuiStatus();
        if (kinectStatus == S_OK) {
            getKinectRGBData();
            updateSkeletalData();
        }
    }
}

 void KinectV1Handler::drawKinectData(sf::RenderWindow &drawingWindow) {
    if (isInitialised()) {
        if (KinectSettings::isKinectDrawn) {
            drawKinectImageData(drawingWindow);
        }
        if (KinectSettings::isSkeletonDrawn) {
            drawTrackedSkeletons(drawingWindow);
        }
    }
};
 void KinectV1Handler::drawKinectImageData(sf::RenderWindow &drawingWindow) {

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
 void KinectV1Handler::drawTrackedSkeletons(sf::RenderWindow &drawingWindow) {
    for (int i = 0; i < NUI_SKELETON_POSITION_COUNT; ++i) {
        screenSkelePoints[i] = sf::Vector2f(0.0f, 0.0f);
    }
    for (int i = 0; i < NUI_SKELETON_COUNT; ++i) {
        NUI_SKELETON_TRACKING_STATE trackingState = skeletonFrame.SkeletonData[i].eTrackingState;

        if (NUI_SKELETON_TRACKED == trackingState)
        {
            if (KinectSettings::isSkeletonDrawn) {
                drawingWindow.pushGLStates();
                drawingWindow.resetGLStates();

                DrawSkeleton(skeletonFrame.SkeletonData[i], drawingWindow);

                drawingWindow.popGLStates();
            }

        }
        else if (NUI_SKELETON_POSITION_ONLY == trackingState) {
            //ONLY CENTER POINT TO DRAW
            if (KinectSettings::isSkeletonDrawn) {
                sf::CircleShape circle(KinectSettings::g_JointThickness, 30);
                circle.setRadius(KinectSettings::g_JointThickness);
                circle.setPosition(SkeletonToScreen(skeletonFrame.SkeletonData[i].Position, SFMLsettings::m_window_width, SFMLsettings::m_window_height));
                circle.setFillColor(sf::Color::Yellow);

                drawingWindow.pushGLStates();
                drawingWindow.resetGLStates();

                drawingWindow.draw(circle);

                drawingWindow.popGLStates();
            }
        }
    }
};

//Consider moving this tracking stuff into a seperate class
 void KinectV1Handler::zeroAllTracking(vr::IVRSystem* &m_sys) {
    for (int i = 0; i < NUI_SKELETON_COUNT; ++i) {
        NUI_SKELETON_TRACKING_STATE trackingState = skeletonFrame.SkeletonData[i].eTrackingState;

        if (NUI_SKELETON_TRACKED == trackingState)
        {
            KinectSettings::hmdZero = getHMDPosition(m_sys);
            kinectZero = zeroKinectPosition(i);
            setKinectToVRMultiplier(i);
            zeroed = true;
            break;

        }
    }
}
 // TODO: MOVE THIS VECTOR STUFF TO LIB IN A NAMESPACE
 std::string to_string(sf::Vector3f v) {
     return std::to_string(v.x) + ", " + std::to_string(v.y) + ", " + std::to_string(v.z);
 }
 sf::Vector3f cross(sf::Vector3f v1, sf::Vector3f v2) {
     float x = (v1.y * v2.z) - (v1.z*v2.y);
     float y = -((v1.x*v2.z) - (v1.z*v2.x));
     float z = (v1.x*v2.y) - (v1.y*v2.x);
     return { x,y,z };
 }
 float dot(sf::Vector3f v1, sf::Vector3f v2) {
     return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
 }
 //https://stackoverflow.com/questions/42421611/3d-vector-rotation-in-c
 // v: a vector in 3D space
 // k: a unit vector describing the axis of rotation
 // theta: the angle (in radians) that v rotates around k
 sf::Vector3f rotate(const sf::Vector3f& v, const sf::Vector3f& k, double theta) {
     std::cout << "Rotating " << to_string(v) << " "
         << theta << " radians around "
         << to_string(k) << "..." << std::endl;

     float cos_theta = cos(theta);
     float sin_theta = sin(theta);

     sf::Vector3f rotated = (v * cos_theta) + (cross(k, v) * sin_theta) + (k * dot(k, v)) * (1 - cos_theta);

     std::cout << "Rotated: " << to_string(rotated) << std::endl;

     return rotated;
 }

 void KinectV1Handler::updateTrackersWithSkeletonPosition(
    vrinputemulator::VRInputEmulator &emulator,
    std::vector<KinectTrackedDevice> trackers)
{
    for (KinectTrackedDevice device : trackers) {
        if (!device.isKinectRepresentation) {
            vr::HmdVector3_t jointPosition{ 0,0,0 };
            if (getRawTrackedJointPos(device, jointPosition)) {
                sf::Vector3f zero = { kinectZero.x, kinectZero.y, kinectZero.z };
                
                //Perspective Tilt Correction
                sf::Vector3f converted = { jointPosition.v[0],jointPosition.v[1] ,jointPosition.v[2] };
                sf::Vector3f rot = rotate(converted, { 1,0,0 }, 10.0f * 3.14159265359f / 180.0f);
                vr::HmdVector3_t pos = { {rot.x,rot.y,rot.z} };

                //Rotation

                device.update(trackedPositionVROffset, pos, zero);
                
            } 
        }
        else {
            //updateKinectTracker(emulator, device);    //Not implemented right now
        }
    }
}
 

bool KinectV1Handler::getRawTrackedJointPos(KinectTrackedDevice device, vr::HmdVector3_t& position) {
    for (int i = 0; i < NUI_SKELETON_COUNT; ++i) {
        NUI_SKELETON_TRACKING_STATE trackingState = skeletonFrame.SkeletonData[i].eTrackingState;

        if (trackingState == NUI_SKELETON_TRACKED)
        {
            // If we can't find either of these joints, exit
            if (jointsUntracked(device.joint0, device.joint1, skeletonFrame.SkeletonData[i]))
            {
                return false;
            }

            // Don't track if both points are inferred
            if (jointsInferred(device.joint0, device.joint1, skeletonFrame.SkeletonData[i]))
            {
                return false;
            }
            else
            {
                float jointX = skeletonFrame.SkeletonData[i].SkeletonPositions[convertJoint(device.joint0)].x;
                float jointY = skeletonFrame.SkeletonData[i].SkeletonPositions[convertJoint(device.joint0)].y;
                float jointZ = skeletonFrame.SkeletonData[i].SkeletonPositions[convertJoint(device.joint0)].z;
                position = vr::HmdVector3_t{ jointX,jointY,jointZ };
                return true;
            }
        }
    }
    return false;
}
NUI_SKELETON_POSITION_INDEX KinectV1Handler::convertJoint(KinectJoint joint)
{

    //Unfortunately I believe this is required because there are mismatches between v1 and v2 joint IDs
    //Might consider investigating to see if there's a way to shorten this
    switch (joint.joint) {
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
bool KinectV1Handler::initKinect() {
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
        NULL, 0 |
        NUI_SKELETON_TRACKING_FLAG_ENABLE_IN_NEAR_RANGE
    );
    return kinectSensor;
}
void KinectV1Handler::getKinectRGBData() {
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
    bool KinectV1Handler::acquireKinectFrame(NUI_IMAGE_FRAME &imageFrame, HANDLE & rgbStream, INuiSensor* &sensor)
    {
        return (sensor->NuiImageStreamGetNextFrame(rgbStream, 1, &imageFrame) < 0);
    }
    INuiFrameTexture* KinectV1Handler::lockKinectPixelData(NUI_IMAGE_FRAME &imageFrame, NUI_LOCKED_RECT &LockedRect)
    {
        INuiFrameTexture* texture = imageFrame.pFrameTexture;
        texture->LockRect(0, &LockedRect, NULL, 0);
        return imageFrame.pFrameTexture;
    }
    void KinectV1Handler::copyKinectPixelData(NUI_LOCKED_RECT &LockedRect, GLubyte* dest)
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
    void KinectV1Handler::unlockKinectPixelData(INuiFrameTexture* texture)
    {
        texture->UnlockRect(0);
    }
    void KinectV1Handler::releaseKinectFrame(NUI_IMAGE_FRAME &imageFrame, HANDLE& rgbStream, INuiSensor* &sensor)
    {
        sensor->NuiImageStreamReleaseFrame(rgbStream, &imageFrame);
    }

    void KinectV1Handler::updateSkeletalData() {
        if (kinectSensor->NuiSkeletonGetNextFrame(0, &skeletonFrame) >= 0) {
            NUI_TRANSFORM_SMOOTH_PARAMETERS params;
            params.fCorrection = .25f;
            params.fJitterRadius = .5f;
            params.fMaxDeviationRadius = .25f;
            params.fPrediction = .25f;
            params.fSmoothing = .25f;
            kinectSensor->NuiTransformSmooth(&skeletonFrame, &params);   //Smooths jittery tracking
        }
        return;
    };
    void KinectV1Handler::DrawSkeleton(const NUI_SKELETON_DATA & skel, sf::RenderWindow &window) {
        for (int i = 0; i < NUI_SKELETON_POSITION_COUNT; ++i) {
            screenSkelePoints[i] = SkeletonToScreen(skel.SkeletonPositions[i], SFMLsettings::m_window_width, SFMLsettings::m_window_height);
            std::cerr << "m_points[" << i << "] = " << screenSkelePoints[i].x << ", " << screenSkelePoints[i].y << std::endl;
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
            circle.setPosition(screenSkelePoints[i]);

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
    sf::Vector2f KinectV1Handler::SkeletonToScreen(Vector4 skeletonPoint, int _width, int _height) {
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
    void KinectV1Handler::DrawBone(const NUI_SKELETON_DATA & skel, NUI_SKELETON_POSITION_INDEX joint0,
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
            DrawLine(screenSkelePoints[joint0], screenSkelePoints[joint1], sf::Color::Green, KinectSettings::g_TrackedBoneThickness, window);
        }
        else
        {
            DrawLine(screenSkelePoints[joint0], screenSkelePoints[joint1], sf::Color::Red, KinectSettings::g_InferredBoneThickness, window);
        }
    }
    void KinectV1Handler::DrawLine(sf::Vector2f start, sf::Vector2f end, sf::Color colour, float lineThickness, sf::RenderWindow &window) {
        sfLine line(start, end);
        line.setColor(colour);
        line.setThickness(lineThickness);
        window.draw(line);
        std::cerr << "Line drawn at: " << start.x << ", " << start.y << " to " << end.x << ", " << end.y << "\n";
    }
    Vector4 KinectV1Handler::zeroKinectPosition(int trackedSkeletonIndex) {
        return skeletonFrame.SkeletonData[trackedSkeletonIndex].SkeletonPositions[NUI_SKELETON_POSITION_HEAD];
    }
    void KinectV1Handler::setKinectToVRMultiplier(int skeletonIndex) {
        KinectSettings::kinectToVRScale = KinectSettings::hmdZero.v[1]
            / (skeletonFrame.SkeletonData[skeletonIndex].SkeletonPositions[NUI_SKELETON_POSITION_HEAD].y
                +
                -skeletonFrame.SkeletonData[skeletonIndex].SkeletonPositions[NUI_SKELETON_POSITION_FOOT_LEFT].y);
        std::cerr << "HMD zero: " << KinectSettings::hmdZero.v[1] << '\n';
        std::cerr << "head pos: " << skeletonFrame.SkeletonData[skeletonIndex].SkeletonPositions[NUI_SKELETON_POSITION_HEAD].y << '\n';
        std::cerr << "foot pos: " << skeletonFrame.SkeletonData[skeletonIndex].SkeletonPositions[NUI_SKELETON_POSITION_FOOT_LEFT].y << '\n';
    }


    bool KinectV1Handler::jointsUntracked(KinectJoint joint0, KinectJoint joint1, NUI_SKELETON_DATA data) {
        NUI_SKELETON_POSITION_TRACKING_STATE joint0State = data.eSkeletonPositionTrackingState[convertJoint(joint0)];
        NUI_SKELETON_POSITION_TRACKING_STATE joint1State = data.eSkeletonPositionTrackingState[convertJoint(joint1)];

        // If we can't find either of these joints, exit
        return ((joint0State == NUI_SKELETON_POSITION_NOT_TRACKED
            || joint1State == NUI_SKELETON_POSITION_NOT_TRACKED)
            && KinectSettings::ignoreInferredPositions);
    }
    bool KinectV1Handler::jointsInferred(KinectJoint joint0, KinectJoint joint1, NUI_SKELETON_DATA data) {
        NUI_SKELETON_POSITION_TRACKING_STATE joint0State = data.eSkeletonPositionTrackingState[convertJoint(joint0)];
        NUI_SKELETON_POSITION_TRACKING_STATE joint1State = data.eSkeletonPositionTrackingState[convertJoint(joint1)];

        // If we can't find either of these joints, exit
        return (joint0State == NUI_SKELETON_POSITION_INFERRED
            && joint1State == NUI_SKELETON_POSITION_INFERRED
            && KinectSettings::ignoreInferredPositions);
    }
