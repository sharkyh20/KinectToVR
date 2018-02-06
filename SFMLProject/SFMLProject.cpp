// SFMLProject.cpp : Defines the entry point for the console application.
//
#include "stdafx.h"
#include "KinectToVR.h"
#include "VRController.h"
#include "GUIHandler.h"
#include <SFML\Audio.hpp>

//GUI
#include <SFGUI\SFGUI.hpp>
#include <SFGUI/Widgets.hpp>

double deltaScaled(double valuePerSecond, double delta) {
    return valuePerSecond * delta;
}

int main()
{
    sf::RenderWindow renderWindow(sf::VideoMode(SFMLsettings::m_window_width, SFMLsettings::m_window_height), "KinectToVR", sf::Style::Titlebar | sf::Style::Close);
    renderWindow.setFramerateLimit(90);   //Prevents ridiculous overupdating and high CPU usage - plus 90Hz is the recommended refresh rate for most VR panels 

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
    KinectHandler kinect;

    NUI_SKELETON_FRAME skeletonFrame = { 0 };

    if (kinect.initStatus()) {
        HRESULT kinectStatus = kinect.kinectSensor->NuiStatus();
        
        if (kinectStatus == S_OK) {
            initOpenGL(kinect);
            updateSkeletalData(skeletonFrame, kinect.kinectSensor);
            std::cerr << "Attempted connection to kinect.... " << kinect.status_str(kinectStatus) << std::endl;    // DEBUG
        }
        else {
            std::cerr << "Attempted connection to kinect.... " << kinect.status_str(kinectStatus) << std::endl;    // DEBUG
        }
    }
    else {
        std::cerr << "Attempted connection to kinect.... UNDETECTED" << std::endl;    // DEBUG
    }
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
    VRcontroller leftController( vr::TrackedControllerRole_LeftHand);

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
        if (eError == vr::VRInitError_None) {
            rightController.update(deltaT);
            leftController.update(deltaT);
        }
        else {
            std::cerr << "Error updating controllers: Could not connect to the SteamVR system! OpenVR init error-code " << std::to_string(eError) << std::endl;
        }
        guiRef.updateKinectStatusLabel(kinect);
        if (kinect.initStatus()) {

            updateSkeletalData(skeletonFrame, kinect.kinectSensor);
            if (!zeroed) {          //Initial attempt to allow user to get into position before setting the trackers -  ** zeroAll sets zeroed to true  **
                if (rightController.GetPress(vr::EVRButtonId::k_EButton_Grip)) {
                    ss << "Grip get!\n";    //DEBUG TEXT
                    zeroAllTracking(skeletonFrame, m_VRSystem);
                    rightController.setHapticPulse(.15, 1000, 0);
                }

            }
            if (KinectSettings::userChangingZero) {
                if (leftController.GetTouch(vr::EVRButtonId::k_EButton_SteamVR_Touchpad)) { //works
                    sf::Vector2f axis = leftController.GetControllerAxisValue(vr::EVRButtonId::k_EButton_SteamVR_Touchpad); //works
                    KinectSettings::trackedPositionOffset[0] += deltaScaled(1.0, deltaT) * axis.x;
                    KinectSettings::trackedPositionOffset[2] += deltaScaled(1.0, deltaT) * axis.y;
                    ss << "LEFT TOUCHPAD DETECTED!\n";
                }
                if (rightController.GetTouch(vr::EVRButtonId::k_EButton_SteamVR_Touchpad)) {
                    sf::Vector2f axis = rightController.GetControllerAxisValue(vr::EVRButtonId::k_EButton_SteamVR_Touchpad); //works
                    KinectSettings::trackedPositionOffset[1] += deltaScaled(1.0, deltaT) * axis.y;
                }
                if (rightController.GetTrigger()) {  //works
                    ss << "Right trigger is down\n";
                    KinectSettings::userChangingZero = false;
                    rightController.setHapticPulse(.15, 1000, 0);
                }
            }
            ss << "Offset = " << KinectSettings::trackedPositionOffset[0] << ", " << KinectSettings::trackedPositionOffset[1] << ", " << KinectSettings::trackedPositionOffset[2] << '\n';
            updateTrackersWithSkeletonPosition(inputEmulator, v_trackers, skeletonFrame);
            //-------------------------------------------
            text.setString(ss.str());


            //Draw

            if (KinectSettings::isKinectDrawn) {
                //drawKinectImageData(kinect);  // CURRENTLY NOT WORKING AND NEEDS TO BE REFACTORED TO WORK WITH DIFFERENT RESOLUTIONS
            }
            if (KinectSettings::isSkeletonDrawn) {
                drawTrackedSkeletons(skeletonFrame, renderWindow);
            }
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
    
    vr::VR_Shutdown();
    destroyTrackers(inputEmulator, v_trackers);
    
    return 0;
}

