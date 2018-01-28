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

    auto SteamInitButton = sfg::Button::Create("Initialise SteamVR Controllers");

    auto ShowSkeletonButton = sfg::ToggleButton::Create("Show/Hide Skeleton Tracking");
    ShowSkeletonButton->GetSignal(sfg::Widget::OnLeftClick).Connect([] {    toggle(KinectSettings::isSkeletonDrawn); });
    
    //Zeroing
    auto zeroLabel = sfg::Label::Create("Enables resetting Zero - After clicking, stand in front of your Kinect, and press the grip button.");
    auto ZeroButton = sfg::Button::Create("Reset Standing Zero Position");
    ZeroButton->GetSignal(sfg::Widget::OnLeftClick).Connect([] {    zeroed = true; });

    //Position Adjust
    auto posLabel = sfg::Label::Create("This re-enables Tracker adjustment with the thumbsticks. Press the trigger when you're happy with the position!");
    auto PositionAdjustButton = sfg::Button::Create("Enable Tracker Offset Adjustment");
    PositionAdjustButton->GetSignal(sfg::Widget::OnLeftClick).Connect([] {    KinectSettings::userChangingZero = true; });

    // Allows for unrestricted tracking, but may be unstable
    auto inferredLabel = sfg::Label::Create("Checking this makes the trackers directly copy the Kinect movement. This may have the benefit of improving tracking when partially occluded, but may also have the consequence of spazzing wildly if tracking is lost.");
    inferredLabel->SetLineWrap(true);
    inferredLabel->SetRequisition(sf::Vector2f(600.f, 20.f));
    auto ignoreInferredCheckButton = sfg::CheckButton::Create("Enable Raw Tracking");
    ignoreInferredCheckButton->GetSignal(sfg::ToggleButton::OnToggle).Connect([&ignoreInferredCheckButton] {
        if (ignoreInferredCheckButton->IsActive()) {
            KinectSettings::ignoreInferredPositions = false;    // No longer stops updating trackers when Kinect isn't sure about a position
        }
        else {
            KinectSettings::ignoreInferredPositions = true;
        }
    });

    std::stringstream ssMessage;
    ssMessage << "Put on your headset and stand in front of the Kinect. The optimal distance microsoft reccommends is about 1.5-2.5m away.\n\n"
        << "You may see a bunch of trackers floating behind you!\n" << "Don't worry, you can initialise them to your current position by pressing down the right grip button.\n"
        << "Although the Kinect tries it's best to find your exact position, it's not always correct, so you can use the left stick to offset the trackers laterally on the ground, and the right stick to move them up and down to your body.\n\n"
        << "When you have set it to your desired position click down on the right trigger.\n NOTE: If you hit the trigger on accident before it reached your desired position, then you can press the button or 'Q' on your keyboard to enable tracker adjusting again";
    auto instructionsLabel = sfg::Label::Create(ssMessage.str());
    instructionsLabel->SetLineWrap(true);
    instructionsLabel->SetRequisition(sf::Vector2f(600.f, 50.f));

    box->Pack(SteamInitButton);
    box->Pack(instructionsLabel);

    box->Pack(ShowSkeletonButton);

    box->Pack(zeroLabel);
    box->Pack(ZeroButton);

    box->Pack(posLabel);
    box->Pack(PositionAdjustButton);

    box->Pack(inferredLabel);
    box->Pack(ignoreInferredCheckButton);

    guiWindow->Add(box);
    sfg::Desktop desktop;
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
    std::vector<KinectTrackedDevice> v_trackers{};
    vrinputemulator::VRInputEmulator inputEmulator;
    inputEmulator.connect();
    // Tracker Initialisation Lambda
    SteamInitButton->GetSignal(sfg::Widget::OnLeftClick).Connect([&SteamInitButton, &v_trackers, &inputEmulator] {  
            SteamInitButton->SetLabel("Trackers Initialised");
            KinectTrackedDevice leftFootTracker(inputEmulator, NUI_SKELETON_POSITION_FOOT_LEFT, NUI_SKELETON_POSITION_ANKLE_LEFT, false);
            KinectTrackedDevice rightFootTracker(inputEmulator, NUI_SKELETON_POSITION_FOOT_RIGHT, NUI_SKELETON_POSITION_ANKLE_RIGHT, false);
            KinectTrackedDevice hipTracker(inputEmulator, NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_SPINE, false);

            //KinectTrackedDevice kinectTrackerRef(inputEmulator, NUI_SKELETON_POSITION_HEAD, NUI_SKELETON_POSITION_HEAD, true);
            //setKinectTrackerProperties(kinectTrackerRef.deviceId);

            v_trackers.push_back(leftFootTracker);
            v_trackers.push_back(rightFootTracker);
            v_trackers.push_back(hipTracker);
            SteamInitButton->SetState(sfg::Widget::State::INSENSITIVE);
    });
    
    
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
        //Update GUI
        desktop.Update(deltaT);

        //Clear ---------------------------------------
        renderWindow.clear();
        

        //Process -------------------------------------
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
        //-------------------------------------------
        text.setString(ss.str());
           
        
        //Draw
        
        if (KinectSettings::isKinectDrawn) {
            //drawKinectImageData(kinect);  // CURRENTLY NOT WORKING AND NEEDS TO BE REFACTORED TO WORK WITH DIFFERENT RESOLUTIONS
        }
        if (KinectSettings::isSkeletonDrawn) {
            drawTrackedSkeletons(skeletonFrame, renderWindow);
        }
       
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

