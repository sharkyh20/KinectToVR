// SFMLProject.cpp : Defines the entry point for the console application.
//
#include "stdafx.h"
#include "KinectToVR.h"
#include "VRController.h"
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
    renderWindow.setFramerateLimit(90);   //Prevents ridiculous overupdating and high CPU usage

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
    renderWindow.pushGLStates();
    renderWindow.resetGLStates();
    sfg::SFGUI sfguiRef;
    
    auto guiWindow = sfg::Window::Create();
    guiWindow->SetTitle("Our Window");

    auto box = sfg::Box::Create(sfg::Box::Orientation::VERTICAL, 5.f);
    auto button = sfg::Button::Create("Click me");

    auto opengl_window = sfg::Window::Create();
    opengl_window->SetTitle("OpenGL canvas");
    opengl_window->SetPosition(sf::Vector2f(50.f, 50.f));
    auto opengl_canvas = sfg::Canvas::Create(true);
    opengl_window->Add(opengl_canvas);
    opengl_canvas->SetRequisition(sf::Vector2f(640.f, 480.f));

    box->Pack(button);
    button->GetSignal(sfg::Widget::OnLeftClick).Connect([&button] {
        button->SetLabel("Hello World!");
    });

    guiWindow->Add(box);
    sfg::Desktop desktop;
    desktop.Add(opengl_window);
    desktop.Add(guiWindow);

    desktop.Update(0.f);

    renderWindow.popGLStates();
    // ----------------------------------------------------

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
    while (renderWindow.isOpen())
    {
        std::stringstream ss;
        double currentTime = clock.restart().asSeconds();
        double deltaT = currentTime;
        ss << "FPS = " << 1.0 / deltaT << '\n';
        
        sf::Event event;
        while (renderWindow.pollEvent(event))
        {
            desktop.HandleEvent(event);

            if (event.type == sf::Event::Closed)
                renderWindow.close();
            if (event.type == sf::Event::KeyPressed) {
                processKeyEvents(event);
            }
        }
        
        //VR input -- Holy shit fuck the openvr 'docs'
        //https://github.com/zecbmo/ViveSkyrim/blob/master/Source/ViveSupport.cpp - Has quite a few useful bits of stuff that the docs don't tell

        //Update GUI
        desktop.Update(deltaT);

        //Clear
        renderWindow.clear();
        
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
        if (KinectSettings::isSkeletonDrawn)
            drawTrackedSkeletons( skeletonFrame, renderWindow);
        
       
        renderWindow.pushGLStates();
        renderWindow.resetGLStates();
        //Draw debug font
        renderWindow.draw(text);
        // Draw GUI
        renderWindow.setActive(true);
        sfguiRef.Display(renderWindow);
        renderWindow.popGLStates();
        //End Frame
        renderWindow.display();
        
    }
    vr::VR_Shutdown();
    destroyTrackers(inputEmulator, v_trackers);
    return 0;
}

