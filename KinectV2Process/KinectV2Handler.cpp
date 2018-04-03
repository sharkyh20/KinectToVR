#pragma once
#include "stdafx.h"
#include "KinectV2Handler.h"
#include <sfLine.h>
#include <iostream>
#include <VRHelper.h>
#include "KinectJointFilter.h"

HRESULT KinectV2Handler::getStatusResult()
{
    BOOLEAN avail;
    return kinectSensor->get_IsAvailable(&avail);
}

std::string KinectV2Handler::statusResultString(HRESULT stat)
{
    switch (stat) {
    case S_OK: return "S_OK";
    case S_FALSE: return "Sensor Unavailable! Check if it's plugged in to your USB and power plugs";
    default: return "Uh Oh undefined kinect error! " + std::to_string(stat);
    }
}

void KinectV2Handler::initialise() {
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
void KinectV2Handler::initOpenGL()
{
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
void KinectV2Handler::update()
{
    if (isInitialised()) {
        BOOLEAN isAvailable = false;
        HRESULT kinectStatus = kinectSensor->get_IsAvailable(&isAvailable);
        if (kinectStatus == S_OK) {
            getKinectData();
        }
    }
}
void KinectV2Handler::drawKinectData(sf::RenderWindow &win) {
    if (KinectSettings::isKinectDrawn) {
        drawKinectImageData(win);
    }
    if (KinectSettings::isSkeletonDrawn) {
        drawTrackedSkeletons(win);
    }
}
void KinectV2Handler::drawKinectImageData(sf::RenderWindow &win) {
    glBindTexture(GL_TEXTURE_2D, kinectTextureId);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, SFMLsettings::m_window_width, SFMLsettings::m_window_height, GL_BGRA, GL_UNSIGNED_BYTE, (GLvoid*)kinectImageData.get());
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
}
void KinectV2Handler::drawTrackedSkeletons(sf::RenderWindow &win) {
    for (int i = 0; i < BODY_COUNT; ++i) {
        IBody* pBody = kinectBodies[i];
        if (pBody)
        {
            BOOLEAN bTracked = false;
            HRESULT thisBodyTracked = pBody->get_IsTracked(&bTracked);
            if (SUCCEEDED(thisBodyTracked) && bTracked)
            {
                Joint joints[JointType_Count];
                sf::Vector2f jointPoints[JointType_Count];
                HandState leftHandState = HandState_Unknown;
                HandState rightHandState = HandState_Unknown;

                pBody->get_HandLeftState(&leftHandState);
                pBody->get_HandRightState(&rightHandState);

                HRESULT jointsFound = pBody->GetJoints(_countof(joints), joints);
                if (SUCCEEDED(jointsFound))
                {
                    for (int j = 0; j < _countof(joints); ++j)
                    {
                        jointPoints[j] = BodyToScreen(joints[j].Position, SFMLsettings::m_window_width, SFMLsettings::m_window_height);
                    }

                    if (KinectSettings::isSkeletonDrawn) {
                        win.pushGLStates();
                        win.resetGLStates();

                        drawBody(joints, jointPoints, win);

                        drawHand(leftHandState, jointPoints[JointType_HandLeft], win);
                        drawHand(rightHandState, jointPoints[JointType_HandRight], win);

                        win.popGLStates();
                    }
                    
                }
            }
        }
    }
}
void KinectV2Handler::drawHand(HandState handState, const sf::Vector2f& handPosition, sf::RenderWindow &win)
{
    sf::CircleShape circle{};
    circle.setPosition(handPosition);
    static const float c_HandSize = 30.0f;
    circle.setRadius(c_HandSize);

    switch (handState)
    {
    case HandState_Closed:
        circle.setFillColor(sf::Color::Red);
        win.draw(circle);
        break;

    case HandState_Open:
        circle.setFillColor(sf::Color::Green);
        win.draw(circle);
        break;

    case HandState_Lasso:
        circle.setFillColor(sf::Color::Blue);
        win.draw(circle);
        break;
    }
}

void KinectV2Handler::updateSkeletalData() {
    IBodyFrame* bodyFrame = nullptr;
    IBodyFrameReference* frameRef = nullptr;
    multiFrame->get_BodyFrameReference(&frameRef);
    frameRef->AcquireFrame(&bodyFrame);
    if (frameRef) frameRef->Release();

    if (!bodyFrame) return;

    
    bodyFrame->GetAndRefreshBodyData(BODY_COUNT, kinectBodies);
    for (int i = 0; i < BODY_COUNT; i++) {
        kinectBodies[i]->get_IsTracked(&isTracking);
        if (isTracking) {
            kinectBodies[i]->GetJoints(JointType_Count, joints);
            kinectBodies[i]->GetJointOrientations(JointType_Count, jointOrientations);


            for (int i = 0; i < JointType_Count; i++) {
                Vector4 orientation = jointOrientations[i].Orientation;
                //std::cerr << "Joint " << i << ": " << orientation.w << ", " << orientation.x << ", " << orientation.y << ", " << orientation.z << '\n';
            }

            //Smooth
            filter.update(joints);
            //rotFilter.update(jointOrientations);  //TODO FIX
            rotationFilter.UpdateFilter(kinectBodies[i], jointOrientations); // New filter, still not properly implemented
            break;
        }
    }
    if (bodyFrame) bodyFrame->Release();
}
sf::Vector3f KinectV2Handler::zeroKinectPosition(int trackedSkeletonIndex) {
    return sf::Vector3f(
        joints[JointType_Head].Position.X,
        joints[JointType_Head].Position.Y, 
        joints[JointType_Head].Position.Z);
}
void KinectV2Handler::zeroAllTracking(vr::IVRSystem *& m_sys)
{
    /*
    for (int i = 0; i < BODY_COUNT; ++i) {
        TrackingState trackingState = joints->TrackingState;

        if (trackingState == TrackingState_Tracked) {
            KinectSettings::hmdZero = getHMDPosition(m_sys);
            setKinectToVRMultiplier(i);
            zeroed = true;
            break;
        }
    }
    */
}
void KinectV2Handler::setKinectToVRMultiplier(int skeletonIndex) {
    /*
    KinectSettings::kinectToVRScale = KinectSettings::hmdZero.v[1]
        / (joints[JointType_Head].Position.Y
            +
            -joints[JointType_AnkleLeft].Position.Y);
    std::cerr << "HMD zero: " << KinectSettings::hmdZero.v[1] << '\n';
    std::cerr << "head pos: " << joints[JointType_Head].Position.Y << '\n';
    std::cerr << "foot pos: " << joints[JointType_AnkleLeft].Position.Y << '\n';
    */
}
void KinectV2Handler::updateTrackersWithSkeletonPosition(vrinputemulator::VRInputEmulator & emulator, std::vector<KVR::KinectTrackedDevice> trackers)
{
    for (KVR::KinectTrackedDevice device : trackers) {
        if (device.isSensor()) {
            device.update(KinectSettings::kinectRepPosition, {0,0,0}, KinectSettings::kinectRepRotation);
        } else {
            vr::HmdVector3_t jointPosition{ 0,0,0 };
            vr::HmdQuaternion_t jointRotation{ 0,0,0,0 };
            if (getFilteredJoint(device, jointPosition, jointRotation)) {
                
                device.update(trackedPositionVROffset, jointPosition, jointRotation);
            }
        }
    }
}
bool KinectV2Handler::getFilteredJoint(KVR::KinectTrackedDevice device, vr::HmdVector3_t& position, vr::HmdQuaternion_t &rotation) {
    sf::Vector3f filteredPos = filter.GetFilteredJoints()[convertJoint(device.joint0)];
    float jointX = filteredPos.x;
    float jointY = filteredPos.y;
    float jointZ = filteredPos.z;
    position = vr::HmdVector3_t{ jointX,jointY,jointZ };

    //Rotation - need to seperate into function
    Vector4 kRotation;
    switch (device.rotationOption) {
    case KVR::JointRotationOption::Unfiltered:
        kRotation = jointOrientations[convertJoint(device.joint0)].Orientation;
        break;
    case KVR::JointRotationOption::Filtered:
        //kRotation = rotFilter.GetFilteredJoints()[convertJoint(device.joint0)];
        kRotation = rotationFilter.GetFilteredJoints()[convertJoint(device.joint0)]; //New filter
        break;
    case KVR::JointRotationOption::HeadLook: {        // Ew
        auto q = KinectSettings::hmdRotation;
        //Isolate Yaw
        float yaw = atan2(2 * q.w*q.y + 2 * q.x*q.z, +q.w*q.w + q.x*q.x - q.z*q.z - q.y*q.y);

        auto kq = vrmath::quaternionFromRotationY(yaw);
        kRotation.w = kq.w;
        kRotation.x = kq.x;
        kRotation.y = kq.y;
        kRotation.z = kq.z;
    }
                                             break;
    default:
        std::cerr << "JOINT ROTATION OPTION UNDEFINED IN DEVICE " << device.deviceId << '\n';
        break;
    }
    rotation.w = kRotation.w;
    rotation.x = kRotation.x;
    rotation.y = kRotation.y;
    rotation.z = kRotation.z;
    
    return true;
}
bool KinectV2Handler::initKinect() {
    if (FAILED(GetDefaultKinectSensor(&kinectSensor))) {
        return false;
    }
    if (kinectSensor) {
        kinectSensor->get_CoordinateMapper(&coordMapper);

        kinectSensor->Open();
        //             FrameSourceTypes::FrameSourceTypes_Depth
        // | FrameSourceTypes::FrameSourceTypes_Color |
        kinectSensor->OpenMultiSourceFrameReader( FrameSourceTypes::FrameSourceTypes_Body,
            &frameReader);
        return frameReader;
    }
    else {
        return false;
    }
}
void KinectV2Handler::getKinectData() {
    if (SUCCEEDED(frameReader->AcquireLatestFrame(&multiFrame))) {
        getRGBImageData(multiFrame);
        updateSkeletalData();
    }
    if (multiFrame) multiFrame->Release();
}
void KinectV2Handler::getRGBImageData(IMultiSourceFrame* multiFrame) {
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
void KinectV2Handler::drawBody(const Joint * pJoints, const sf::Vector2f * pJointPoints, sf::RenderWindow & window)
{
    // Draw the bones

    // Torso
    drawBone(pJoints, pJointPoints, JointType_Head, JointType_Neck, window);
    drawBone(pJoints, pJointPoints, JointType_Neck, JointType_SpineShoulder, window);
    drawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_SpineMid, window);
    drawBone(pJoints, pJointPoints, JointType_SpineMid, JointType_SpineBase, window);
    drawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_ShoulderRight, window);
    drawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_ShoulderLeft, window);
    drawBone(pJoints, pJointPoints, JointType_SpineBase, JointType_HipRight, window);
    drawBone(pJoints, pJointPoints, JointType_SpineBase, JointType_HipLeft, window);

    // Right Arm    
    drawBone(pJoints, pJointPoints, JointType_ShoulderRight, JointType_ElbowRight, window);
    drawBone(pJoints, pJointPoints, JointType_ElbowRight, JointType_WristRight, window);
    drawBone(pJoints, pJointPoints, JointType_WristRight, JointType_HandRight, window);
    drawBone(pJoints, pJointPoints, JointType_HandRight, JointType_HandTipRight, window);
    drawBone(pJoints, pJointPoints, JointType_WristRight, JointType_ThumbRight, window);

    // Left Arm
    drawBone(pJoints, pJointPoints, JointType_ShoulderLeft, JointType_ElbowLeft, window);
    drawBone(pJoints, pJointPoints, JointType_ElbowLeft, JointType_WristLeft, window);
    drawBone(pJoints, pJointPoints, JointType_WristLeft, JointType_HandLeft, window);
    drawBone(pJoints, pJointPoints, JointType_HandLeft, JointType_HandTipLeft, window);
    drawBone(pJoints, pJointPoints, JointType_WristLeft, JointType_ThumbLeft, window);

    // Right Leg
    drawBone(pJoints, pJointPoints, JointType_HipRight, JointType_KneeRight, window);
    drawBone(pJoints, pJointPoints, JointType_KneeRight, JointType_AnkleRight, window);
    drawBone(pJoints, pJointPoints, JointType_AnkleRight, JointType_FootRight, window);
    // Left Leg
    drawBone(pJoints, pJointPoints, JointType_HipLeft, JointType_KneeLeft, window);
    drawBone(pJoints, pJointPoints, JointType_KneeLeft, JointType_AnkleLeft, window);
    drawBone(pJoints, pJointPoints, JointType_AnkleLeft, JointType_FootLeft, window);

    // Draw the joints
    for (int i = 0; i < JointType_Count; ++i)
    {
        sf::CircleShape circle{};
        circle.setRadius(KinectSettings::g_JointThickness);
        circle.setPosition(pJointPoints[i]);

        if (pJoints[i].TrackingState == TrackingState_Inferred)
        {
            circle.setFillColor(sf::Color::Red);
            window.draw(circle);
        }
        else if (pJoints[i].TrackingState == TrackingState_Tracked)
        {
            circle.setFillColor(sf::Color::Yellow);
            window.draw(circle);
        }
    }
}
void KinectV2Handler::drawBone(const Joint* pJoints, const sf::Vector2f* pJointPoints, JointType joint0, JointType joint1, sf::RenderWindow &window) {
    TrackingState joint0State = pJoints[joint0].TrackingState;
    TrackingState joint1State = pJoints[joint1].TrackingState;

    // If we can't find either of these joints, exit
    if ((joint0State == TrackingState_NotTracked) || (joint1State == TrackingState_NotTracked))
    {
        return;
    }

    // Don't draw if both points are inferred
    if ((joint0State == TrackingState_Inferred) && (joint1State == TrackingState_Inferred))
    {
        return;
    }

    // We assume all drawn bones are inferred unless BOTH joints are tracked
    if ((joint0State == TrackingState_Tracked) && (joint1State == TrackingState_Tracked))
    {
        drawLine(pJointPoints[joint0], pJointPoints[joint1], sf::Color::Green, KinectSettings::g_TrackedBoneThickness, window);
    }
    else
    {
        drawLine(pJointPoints[joint0], pJointPoints[joint1], sf::Color::Green, KinectSettings::g_TrackedBoneThickness, window);
    }
}
void KinectV2Handler::drawLine(sf::Vector2f start, sf::Vector2f end, sf::Color colour, float lineThickness, sf::RenderWindow &window) {
    sfLine line(start, end);
    line.setColor(colour);
    line.setThickness(lineThickness);
    window.draw(line);
}
JointType KinectV2Handler::convertJoint(KVR::KinectJoint kJoint) {
    // Currently, the v2 SDK id's match my jointtype class 1:1
    return static_cast<JointType>(kJoint.joint);
}