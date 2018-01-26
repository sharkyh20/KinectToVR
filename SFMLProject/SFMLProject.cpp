// SFMLProject.cpp : Defines the entry point for the console application.
//
#include "stdafx.h"
#include "KinectToVR.h"
#include "VRController.h"
#include <SFML\Audio.hpp>

double deltaScaled(double valuePerSecond, double delta) {
    return valuePerSecond * delta;
}

int main()
{
    sf::RenderWindow window(sf::VideoMode(SFMLsettings::m_window_width, SFMLsettings::m_window_height), "SFML WORKS");
    window.setFramerateLimit(90);   //Prevents ridiculous overupdating and high CPU usage

    sf::Clock clock;
    

    sf::Font font;
    sf::Text text;
    // Global Debug Font

    font.loadFromFile("arial.ttf");
    text.setFont(font);
    text.setString("");
    text.setCharacterSize(40);
    text.setFillColor(sf::Color::Red);
    window.draw(text);


    //Initialise Kinect
    KinectHandler kinect;
    initOpenGL(kinect.kinectTextureId, kinect.kinectImageData.get());
    NUI_SKELETON_FRAME skeletonFrame = { 0 };
    updateSkeletalData(skeletonFrame, kinect.kinectSensor);

    //Initialise InputEmu and Trackers
    vrinputemulator::VRInputEmulator inputEmulator;
    inputEmulator.connect();
    KinectTrackedDevice leftFootTracker(inputEmulator, NUI_SKELETON_POSITION_FOOT_LEFT, NUI_SKELETON_POSITION_ANKLE_LEFT, false);
    KinectTrackedDevice rightFootTracker(inputEmulator, NUI_SKELETON_POSITION_FOOT_RIGHT, NUI_SKELETON_POSITION_ANKLE_RIGHT, false);
    KinectTrackedDevice hipTracker(inputEmulator, NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_SPINE, false);
    //KinectTrackedDevice kinectTrackerRef(inputEmulator, NUI_SKELETON_POSITION_HEAD, NUI_SKELETON_POSITION_HEAD, true);
    //setKinectTrackerProperties(kinectTrackerRef.deviceId);
    std::vector<KinectTrackedDevice> v_trackers{};
    v_trackers.push_back(leftFootTracker);
    v_trackers.push_back(rightFootTracker);
    v_trackers.push_back(hipTracker);
    //v_trackers.push_back(kinectTrackerRef);

    //Initialise VR System
    vr::EVRInitError eError = vr::VRInitError_None;
    vr::IVRSystem *m_VRSystem = vr::VR_Init(&eError, vr::VRApplication_Utility);

    VRcontroller rightController(m_VRSystem, vr::TrackedControllerRole_RightHand);
    VRcontroller leftController(m_VRSystem, vr::TrackedControllerRole_LeftHand);

    KinectSettings::userChangingZero = true;
    while (window.isOpen()) 
    {
        std::stringstream ss;
        double currentTime = clock.restart().asSeconds();
        double deltaT = currentTime;
        ss << "FPS = " << 1.0 / deltaT << '\n';
        
        sf::Event event;
        while (window.pollEvent(event)) 
        {
            if (event.type == sf::Event::Closed)
                window.close();
            if (event.type == sf::Event::KeyPressed) {
                processKeyEvents(event);
            }
        }
        
        //VR input -- Holy shit fuck the openvr 'docs'
        //https://github.com/zecbmo/ViveSkyrim/blob/master/Source/ViveSupport.cpp - Has quite a few useful bits of stuff that the docs don't tell

        //Clear
        window.clear();
        
        //TODO - Initialise the position at the start of the program on user input, as otherwise you need to sprint to the position to get it right
        //Process
        rightController.update();
        leftController.update();

        updateSkeletalData(skeletonFrame, kinect.kinectSensor);
        if(!zeroed) {          //Initial attempt to allow user to get into position before setting the trackers -  ** zeroAll sets zeroed to true  **
            if (rightController.GetPress(vr::EVRButtonId::k_EButton_Grip)) {
                ss << "Grip get!\n";
                zeroAllTracking(skeletonFrame, m_VRSystem);
            }
            
        }
        else if (KinectSettings::userChangingZero) {
            if (leftController.GetTouch(vr::EVRButtonId::k_EButton_SteamVR_Touchpad)) { //works
                    sf::Vector2f axis = leftController.GetControllerAxisValue(vr::EVRButtonId::k_EButton_SteamVR_Touchpad); //works
                    KinectSettings::trackedPositionOffset[0] += deltaScaled(1.0, deltaT) * axis.x;
                    KinectSettings::trackedPositionOffset[2] += deltaScaled(1.0, deltaT) * axis.y;
                }
            if (rightController.GetTouch(vr::EVRButtonId::k_EButton_SteamVR_Touchpad)) {
                sf::Vector2f axis = rightController.GetControllerAxisValue(vr::EVRButtonId::k_EButton_SteamVR_Touchpad); //works
                KinectSettings::trackedPositionOffset[1] += deltaScaled(1.0, deltaT) * axis.y;
            }
            if (rightController.GetTrigger()){  //works
                ss << "Right trigger is down\n";
                KinectSettings::userChangingZero = false;
            }
        }
        ss << "Offset = " << KinectSettings::trackedPositionOffset[0] << ", " << KinectSettings::trackedPositionOffset[1] << ", " << KinectSettings::trackedPositionOffset[2] << '\n';
        updateTrackersWithSkeletonPosition(inputEmulator, v_trackers, skeletonFrame);
        
        text.setString(ss.str());
           
        
        //Draw
        
        if (KinectSettings::isKinectDrawn)
            drawKinectImageData(kinect);
        drawTrackedSkeletons( skeletonFrame, window);
        //Draw debug font
        window.pushGLStates();
        window.resetGLStates();
        window.draw(text);
        window.popGLStates();
        //End Frame
        window.display();
        
    }
    vr::VR_Shutdown();
    destroyTrackers(inputEmulator, v_trackers);
    return 0;
}

