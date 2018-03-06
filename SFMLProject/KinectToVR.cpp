#include "stdafx.h"
#include "KinectToVR.h"

#include "KinectSettings.h"
#include "VRController.h"
#include "GUIHandler.h"
#include <SFML\Audio.hpp>
#include <iostream>
//GUI
#include <SFGUI\SFGUI.hpp>
#include <SFGUI/Widgets.hpp>
/*
void updateKinectTracker(vrinputemulator::VRInputEmulator &emulator, KinectTrackedDevice device)
{
    auto pose = emulator.getVirtualDevicePose(device.deviceId);
    // The Kinect Tracker position must be rotated, as otherwise the tracker is oriented to the wrong direction
    
    double kRelativeX =  - kinectZero.x;
    double kRelativeY =  - kinectZero.y;
    double kRelativeZ = - kinectZero.z;
    double rawPositionX = hmdZero.v[0] + kRelativeX;
    double rawPositionZ =  hmdZero.v[2] + kRelativeZ;

    pose.vecPosition[0] = kinectToVRScale * rawPositionX;
    pose.vecPosition[1] = kinectToVRScale * kRelativeY;
    pose.vecPosition[2] = kinectToVRScale * rawPositionZ;
    

    double kRelativeX = -kinectZero.x;
    double kRelativeY = -kinectZero.y;
    double kRelativeZ = -kinectZero.z;
    double rawPositionX = hmdZero.v[0] + kRelativeX;
    double rawPositionZ = hmdZero.v[2] + kRelativeZ;

    pose.vecPosition[0] = rawPositionX;
    pose.vecPosition[1] = kinectToVRScale * kRelativeY;
    pose.vecPosition[2] = rawPositionZ;

    pose.poseIsValid = true;
    pose.result = vr::TrackingResult_Running_OK;
    emulator.setVirtualDevicePose(device.deviceId, pose);
}
*/
double deltaScaled(double valuePerSecond, double delta) {
    return valuePerSecond * delta;
}
void toEulerAngle(vr::HmdQuaternion_t q, double& roll, double& pitch, double& yaw)
{
    // roll (x-axis rotation)
    double sinr = +2.0 * (q.w * q.x + q.y * q.z);
    double cosr = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    roll = atan2(sinr, cosr);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny = +2.0 * (q.w * q.z + q.x * q.y);
    double cosy = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    yaw = atan2(siny, cosy);
}



void processKeyEvents(sf::Event event) {
    switch (event.key.code) {
    case sf::Keyboard::A:
        toggle(KinectSettings::isKinectDrawn);
        break;
    case sf::Keyboard::S:
        toggle(KinectSettings::isSkeletonDrawn);
        break;
    case sf::Keyboard::Q:
        KinectSettings::userChangingZero = true;
        break;
    default:
        break;
    }
}
void toggle(bool &b) {
    b = !b;
}

void processLoop(KinectHandlerBase& kinect) {
    sf::RenderWindow renderWindow(sf::VideoMode(SFMLsettings::m_window_width, SFMLsettings::m_window_height), "KinectToVR", sf::Style::Titlebar | sf::Style::Close);
    renderWindow.setFramerateLimit(45);   //Prevents ridiculous overupdating and high CPU usage - plus 90Hz is the recommended refresh rate for most VR panels 

    sf::Clock clock;

    sf::Font font;
    sf::Text text;
    // Global Debug Font

    font.loadFromFile("arial.ttf");
    text.setFont(font);
    text.setString("");
    text.setCharacterSize(40);
    text.setFillColor(sf::Color::Red);
    renderWindow.draw(text);

    //SFGUI Handling -------------------------------------- 
    GUIHandler guiRef;
    // ----------------------------------------------------

    //Initialise Kinect
    KinectSettings::serializeKinectSettings();
    KinectSettings::kinectRepRotation = vrmath::quaternionFromYawPitchRoll(KinectSettings::kinectRadRotation.v[1], KinectSettings::kinectRadRotation.v[0], KinectSettings::kinectRadRotation.v[2]);
    kinect.update();

    guiRef.updateKinectStatusLabel(kinect);
    // Reconnect Kinect Event Signal
    guiRef.setKinectButtonSignal(kinect);

    //Initialise InputEmu and Trackers
    std::vector<KinectTrackedDevice> v_trackers{};
    vrinputemulator::VRInputEmulator inputEmulator;
    try {
        inputEmulator.connect();
    }
    catch (vrinputemulator::vrinputemulator_connectionerror e) {
        guiRef.updateEmuStatusLabelError(e);
        std::cerr << "Attempted connection to Input Emulator" << std::to_string(e.errorcode) + " " + e.what() + "\n\n Is SteamVR open and InputEmulator installed?" << std::endl;   // DEBUG
    }

    // Tracker Initialisation Lambda
    if (inputEmulator.isConnected()) {
        guiRef.setTrackerInitButtonSignal(inputEmulator, v_trackers);
        guiRef.updateEmuStatusLabelSuccess();
    }
    else {
        guiRef.updateTrackerInitButtonLabelFail();
    }

    //v_trackers.push_back(kinectTrackerRef);
    std::cerr << "Attempting connection to vrsystem.... " << std::endl;    // DEBUG
                                                                           //Initialise VR System
    VRcontroller rightController(vr::TrackedControllerRole_RightHand);
    VRcontroller leftController(vr::TrackedControllerRole_LeftHand);

    vr::EVRInitError eError = vr::VRInitError_None;
    vr::IVRSystem *m_VRSystem = vr::VR_Init(&eError, vr::VRApplication_Utility);
    if (eError == vr::VRInitError_None) {
        std::cerr << "Attempting connection to controllers.... " << std::endl;    // DEBUG
        leftController.Connect(m_VRSystem);
        rightController.Connect(m_VRSystem);
        std::cerr << "Attempted connection to controllers! " << std::endl;    // DEBUG
    }
    guiRef.updateVRStatusLabel(eError);
    std::cerr << "Attempted connection to vrsystem! " << eError << std::endl;    // DEBUG

    guiRef.setReconnectControllerButtonSignal(leftController, rightController, m_VRSystem);

    KinectSettings::userChangingZero = true;
    while (renderWindow.isOpen())
    {
        std::stringstream ss;
        double currentTime = clock.restart().asSeconds();
        double deltaT = currentTime;
        ss << "FPS = " << 1.0 / deltaT << '\n';

        sf::Event event;
        while (renderWindow.pollEvent(event))
        {
            guiRef.desktopHandleEvents(event);

            if (event.type == sf::Event::Closed)
                renderWindow.close();
            if (event.type == sf::Event::KeyPressed) {
                processKeyEvents(event);
            }
        }
        //Update GUI
        guiRef.updateDesktop(deltaT);

        //Clear ---------------------------------------
        renderWindow.clear();


        //Process -------------------------------------
        // Update Kinect Status
        rightController.Connect(m_VRSystem);
        leftController.Connect(m_VRSystem);
        if (eError == vr::VRInitError_None) {
            rightController.update(deltaT);
            leftController.update(deltaT);
        }
        else {
            std::cerr << "Error updating controllers: Could not connect to the SteamVR system! OpenVR init error-code " << std::to_string(eError) << std::endl;
        }
        guiRef.updateKinectStatusLabel(kinect);
        if (kinect.isInitialised()) {
            kinect.update();
            
            if (KinectSettings::adjustingKinectRepresentationPos) { //TEMP FOR TESTING IMPLMENTATION- Warning Gross af
                if (leftController.GetTouch(vr::EVRButtonId::k_EButton_SteamVR_Touchpad)) {
                    sf::Vector2f axis = leftController.GetControllerAxisValue(vr::EVRButtonId::k_EButton_SteamVR_Touchpad);
                    KinectSettings::kinectRepPosition.v[0] += deltaScaled(1.0, deltaT) * axis.x;
                    KinectSettings::kinectRepPosition.v[2] += deltaScaled(1.0, deltaT) * axis.y;
                }
                if (rightController.GetTouch(vr::EVRButtonId::k_EButton_SteamVR_Touchpad)) {
                    sf::Vector2f axis = rightController.GetControllerAxisValue(vr::EVRButtonId::k_EButton_SteamVR_Touchpad);
                    KinectSettings::kinectRepPosition.v[1] += deltaScaled(1.0, deltaT) * axis.y;
                }
                if (rightController.GetTrigger()) {
                    KinectSettings::adjustingKinectRepresentationPos = false;
                    guiRef.togglePosButton();
                    rightController.setHapticPulse(.15, 1000, 0);
                }
            }
            else if (KinectSettings::adjustingKinectRepresentationRot) { //TEMP FOR TESTING IMPLMENTATION
                KinectSettings::kinectRepRotation = vrmath::quaternionFromYawPitchRoll(KinectSettings::kinectRadRotation.v[1], KinectSettings::kinectRadRotation.v[0], KinectSettings::kinectRadRotation.v[2]);
                //std::cerr << isRotating << '\n';
                if (leftController.GetTouch(vr::EVRButtonId::k_EButton_SteamVR_Touchpad)) {
                    sf::Vector2f axis = leftController.GetControllerAxisValue(vr::EVRButtonId::k_EButton_SteamVR_Touchpad);
                    KinectSettings::kinectRadRotation.v[1] += deltaScaled(3.0, deltaT) * axis.x;
                        //std::cerr << "ROT ADJUSTED L\n";
                }
                if (rightController.GetTouch(vr::EVRButtonId::k_EButton_SteamVR_Touchpad)) {
                    sf::Vector2f axis = rightController.GetControllerAxisValue(vr::EVRButtonId::k_EButton_SteamVR_Touchpad);
                    KinectSettings::kinectRadRotation.v[0] += deltaScaled(3.0, deltaT) * axis.y;
                        //std::cerr << "ROT ADJUSTED R\n";
                }
                if (rightController.GetTrigger()) {
                    KinectSettings::adjustingKinectRepresentationRot = false;
                    guiRef.toggleRotButton();
                    rightController.setHapticPulse(.15, 1000, 0);
                }
            }
            else {
                if (!kinect.isZeroed()) {          //Initial attempt to allow user to get into position before setting the trackers -  ** zeroAll sets zeroed to true  **
                    if (rightController.GetPress(vr::EVRButtonId::k_EButton_Grip)) {
                        kinect.zeroAllTracking(m_VRSystem);
                        rightController.setHapticPulse(.15, 1000, 0);
                    }
                }
                if (KinectSettings::userChangingZero) {
                    if (leftController.GetTouch(vr::EVRButtonId::k_EButton_SteamVR_Touchpad)) {
                        sf::Vector2f axis = leftController.GetControllerAxisValue(vr::EVRButtonId::k_EButton_SteamVR_Touchpad);

                        kinect.trackedPositionVROffset.v[0] += deltaScaled(1.0, deltaT) * axis.x;
                        kinect.trackedPositionVROffset.v[2] += deltaScaled(1.0, deltaT) * axis.y;
                    }
                    if (rightController.GetTouch(vr::EVRButtonId::k_EButton_SteamVR_Touchpad)) {
                        sf::Vector2f axis = rightController.GetControllerAxisValue(vr::EVRButtonId::k_EButton_SteamVR_Touchpad);

                        kinect.trackedPositionVROffset.v[1] += deltaScaled(1.0, deltaT) * axis.y;
                    }
                    if (rightController.GetTrigger()) {
                        KinectSettings::userChangingZero = false;
                        rightController.setHapticPulse(.15, 1000, 0);
                    }
                }
            }
            kinect.updateTrackersWithSkeletonPosition(inputEmulator, v_trackers);

            //Draw
            kinect.drawKinectData(renderWindow);
        }

        renderWindow.pushGLStates();
        renderWindow.resetGLStates();

        //Draw debug font
        renderWindow.draw(text);

        // Draw GUI
        renderWindow.setActive(true);

        guiRef.display(renderWindow);

        renderWindow.popGLStates();
        //End Frame
        renderWindow.display();

    }
    for (KinectTrackedDevice d : v_trackers) {
        d.destroy();
    }
    KinectSettings::writeKinectSettings();
    vr::VR_Shutdown();
}

void spawnAndConnectTracker(vrinputemulator::VRInputEmulator & inputE, std::vector<KinectTrackedDevice>& v_trackers, KinectJointType mainJoint, KinectJointType secondaryJoint)
{
    KinectTrackedDevice device(inputE, mainJoint, secondaryJoint, false);
    v_trackers.push_back(device);
}

void spawnDefaultFullBodyTrackers(vrinputemulator::VRInputEmulator & inputE, std::vector<KinectTrackedDevice>& v_trackers)
{
    spawnAndConnectTracker(inputE, v_trackers, KinectJointType::AnkleLeft, KinectJointType::FootLeft);
    spawnAndConnectTracker(inputE, v_trackers, KinectJointType::AnkleRight, KinectJointType::FootLeft);
    spawnAndConnectTracker(inputE, v_trackers, KinectJointType::SpineBase, KinectJointType::SpineMid);
}

void spawnAndConnectKinectTracker(vrinputemulator::VRInputEmulator &inputE, std::vector<KinectTrackedDevice> &v_trackers)
{
    KinectTrackedDevice kinectTrackerRef(inputE, KinectJointType::Head, KinectJointType::Head, true);
    setKinectTrackerProperties(kinectTrackerRef.deviceId);
    v_trackers.push_back(kinectTrackerRef);
}
