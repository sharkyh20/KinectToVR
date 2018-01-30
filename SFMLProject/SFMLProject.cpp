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
class GUIHandler {
public:
    GUIHandler() {
        guiWindow->SetTitle("Main Window");

        setSignals();
        setLineWrapping();
        packElementsIntoMainBox();

        guiWindow->Add(mainGUIBox);
        sfg::Desktop desktop;
        desktop.Add(guiWindow);
        desktop.Update(0.f);
    }
    ~GUIHandler() {}

    void setSignals() {
        ShowSkeletonButton->GetSignal(sfg::Widget::OnLeftClick).Connect([] {    
            toggle(KinectSettings::isSkeletonDrawn); 
        });
        ZeroButton->GetSignal(sfg::Widget::OnLeftClick).Connect([] {    
            zeroed = true; 
        });
        PositionAdjustButton->GetSignal(sfg::Widget::OnLeftClick).Connect([]{    KinectSettings::userChangingZero = true; 
        });
        IgnoreInferredCheckButton->GetSignal(sfg::ToggleButton::OnToggle).Connect([this] {
            if (IgnoreInferredCheckButton->IsActive()) {
                KinectSettings::ignoreInferredPositions = false;    // No longer stops updating trackers when Kinect isn't sure about a position
            }
            else {
                KinectSettings::ignoreInferredPositions = true;
            }
        });
    }
    void setLineWrapping() {
        InferredLabel->SetLineWrap(true);
        InferredLabel->SetRequisition(sf::Vector2f(600.f, 20.f));

        InstructionsLabel->SetLineWrap(true);
        InstructionsLabel->SetRequisition(sf::Vector2f(600.f, 50.f));
    }
    void packElementsIntoMainBox() {
        //Statuses are at the top
        mainGUIBox->Pack(KinectStatusLabel);
        mainGUIBox->Pack(SteamVRStatusLabel);
        mainGUIBox->Pack(InputEmulatorStatusLabel);

        mainGUIBox->Pack(reconKinectButton);
        mainGUIBox->Pack(TrackerInitButton);
        mainGUIBox->Pack(InstructionsLabel);

        mainGUIBox->Pack(ShowSkeletonButton);

        mainGUIBox->Pack(ReconControllersLabel);
        mainGUIBox->Pack(ReconControllersButton);

        mainGUIBox->Pack(ZeroLabel);
        mainGUIBox->Pack(ZeroButton);

        mainGUIBox->Pack(PosLabel);
        mainGUIBox->Pack(PositionAdjustButton);

        mainGUIBox->Pack(InferredLabel);
        mainGUIBox->Pack(IgnoreInferredCheckButton);
    }

    void updateKinectStatusLabel(HRESULT status, KinectHandler& kinect) {
        // TODO UPDATE KINECT INIT WITH THIS
        switch (status) {
        case S_OK:
            KinectStatusLabel->SetText("Kinect Status: Success!");
            break;
        default: 
            KinectStatusLabel->SetText("Kinect Status: ERROR " + kinect.status_str(status));
            break;
        }
    }

private:
    sfg::SFGUI sfguiRef;
    sfg::Window::Ptr guiWindow = sfg::Window::Create();
    sfg::Desktop guiDesktop;
    
    sfg::Box::Ptr mainGUIBox = sfg::Box::Create(sfg::Box::Orientation::VERTICAL, 5.f);
    //Statuses
    sfg::Label::Ptr KinectStatusLabel = sfg::Label::Create();
    sfg::Label::Ptr SteamVRStatusLabel = sfg::Label::Create();
    sfg::Label::Ptr InputEmulatorStatusLabel = sfg::Label::Create();

    sfg::Button::Ptr reconKinectButton = sfg::Button::Create("Reconnect Kinect");
    sfg::Button::Ptr TrackerInitButton = sfg::Button::Create("Initialise SteamVR Kinect Trackers");

    sfg::Button::Ptr ShowSkeletonButton = sfg::CheckButton::Create("Show/Hide Skeleton Tracking: MAY CAUSE LAG IN TRACKERS");
   
    //Zeroing
    sfg::Label::Ptr ZeroLabel = sfg::Label::Create("Enables resetting Zero - After clicking, stand in front of your Kinect, and press the grip button.");
    sfg::Button::Ptr ZeroButton = sfg::Button::Create("Reset Standing Zero Position");
    

    //Position Adjust
    sfg::Label::Ptr PosLabel = sfg::Label::Create("This re-enables Tracker adjustment with the thumbsticks. Press the trigger when you're happy with the position!");
    sfg::Button::Ptr PositionAdjustButton = sfg::Button::Create("Enable Tracker Offset Adjustment");
    

    //Redetect Controllers
    sfg::Label::Ptr ReconControllersLabel = sfg::Label::Create("If controller input isn't working, press this to reconnect them.\n Make sure both are on, and not in standby.");
    sfg::Button::Ptr ReconControllersButton = sfg::Button::Create("Reconnect VR Controllers");


    // Allows for unrestricted tracking, but may be unstable
    sfg::Label::Ptr InferredLabel = sfg::Label::Create("Checking this makes the trackers directly copy the Kinect movement. This may have the benefit of improving tracking when partially occluded, but may also have the consequence of spazzing wildly if tracking is lost.");
    sfg::CheckButton::Ptr IgnoreInferredCheckButton = sfg::CheckButton::Create("Enable Raw Tracking");

    sfg::Label::Ptr InstructionsLabel = sfg::Label::Create("Put on your headset and stand in front of the Kinect. The optimal distance microsoft reccommends is about 1.5-2.5m away.\n\nYou may see a bunch of trackers floating behind you!\nDon't worry, you can initialise them to your current position by pressing down the right grip button.\nAlthough the Kinect tries it's best to find your exact position, it's not always correct, so you can use the left stick to offset the trackers laterally on the ground, and the right stick to move them up and down to your body.\n\nWhen you have set it to your desired position click down on the right trigger.\n NOTE: If you hit the trigger on accident before it reached your desired position, then you can press the button or 'Q' on your keyboard to enable tracker adjusting again");    //Blegh - There has to be a better way than this, maybe serialization?
};

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
    // THIS NEEDS TO BE SEPERATED FROM EVERYTHING JESUS CHRIST
    /*
    renderWindow.pushGLStates();
    renderWindow.resetGLStates();
    sfg::SFGUI sfguiRef;
    
    auto guiWindow = sfg::Window::Create();
    guiWindow->SetTitle("Our Window");

    auto box = sfg::Box::Create(sfg::Box::Orientation::VERTICAL, 5.f);

    //Statuses
    auto KinectStatusLabel = sfg::Label::Create();
    auto SteamVRStatusLabel = sfg::Label::Create();
    auto InputEmulatorStatusLabel = sfg::Label::Create();

    auto reconKinectButton = sfg::Button::Create("Reconnect Kinect");
    auto TrackerInitButton = sfg::Button::Create("Initialise SteamVR Kinect Trackers");

    auto ShowSkeletonButton = sfg::CheckButton::Create("Show/Hide Skeleton Tracking: MAY CAUSE LAG IN TRACKERS");
    ShowSkeletonButton->GetSignal(sfg::Widget::OnLeftClick).Connect([] {    toggle(KinectSettings::isSkeletonDrawn); });
    
    //Zeroing
    auto ZeroLabel = sfg::Label::Create("Enables resetting Zero - After clicking, stand in front of your Kinect, and press the grip button.");
    auto ZeroButton = sfg::Button::Create("Reset Standing Zero Position");
    ZeroButton->GetSignal(sfg::Widget::OnLeftClick).Connect([] {    zeroed = true; });

    //Position Adjust
    auto PosLabel = sfg::Label::Create("This re-enables Tracker adjustment with the thumbsticks. Press the trigger when you're happy with the position!");
    auto PositionAdjustButton = sfg::Button::Create("Enable Tracker Offset Adjustment");
    PositionAdjustButton->GetSignal(sfg::Widget::OnLeftClick).Connect([] {    KinectSettings::userChangingZero = true; });

    //Redetect Controllers
    auto ReconControllersLabel = sfg::Label::Create("If controller input isn't working, press this to reconnect them.\n Make sure both are on, and not in standby.");
    auto ReconControllersButton = sfg::Button::Create("Reconnect VR Controllers");
    

    // Allows for unrestricted tracking, but may be unstable
    auto InferredLabel = sfg::Label::Create("Checking this makes the trackers directly copy the Kinect movement. This may have the benefit of improving tracking when partially occluded, but may also have the consequence of spazzing wildly if tracking is lost.");
    InferredLabel->SetLineWrap(true);
    InferredLabel->SetRequisition(sf::Vector2f(600.f, 20.f));
    auto IgnoreInferredCheckButton = sfg::CheckButton::Create("Enable Raw Tracking");
    IgnoreInferredCheckButton->GetSignal(sfg::ToggleButton::OnToggle).Connect([&IgnoreInferredCheckButton] {
        if (IgnoreInferredCheckButton->IsActive()) {
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
    auto InstructionsLabel = sfg::Label::Create(ssMessage.str());
    InstructionsLabel->SetLineWrap(true);
    InstructionsLabel->SetRequisition(sf::Vector2f(600.f, 50.f));

    //Statuses are at the top
    box->Pack(KinectStatusLabel);
    box->Pack(SteamVRStatusLabel);
    box->Pack(InputEmulatorStatusLabel);

    box->Pack(reconKinectButton);
    box->Pack(TrackerInitButton);
    box->Pack(InstructionsLabel);

    box->Pack(ShowSkeletonButton);

    box->Pack(ReconControllersLabel);
    box->Pack(ReconControllersButton);

    box->Pack(ZeroLabel);
    box->Pack(ZeroButton);

    box->Pack(PosLabel);
    box->Pack(PositionAdjustButton);

    box->Pack(InferredLabel);
    box->Pack(IgnoreInferredCheckButton);

    guiWindow->Add(box);
    sfg::Desktop desktop;
    desktop.Add(guiWindow);

    desktop.Update(0.f);

    */
    //New GUI implementation
    GUIHandler guiRef;


    renderWindow.popGLStates();
    // ----------------------------------------------------

    //Initialise Kinect
    KinectHandler kinect;

    NUI_SKELETON_FRAME skeletonFrame = { 0 };

    if (kinect.initStatus()) {
        HRESULT kinectStatus = kinect.kinectSensor->NuiStatus();
        
        
        if (kinectStatus == S_OK) {
            initOpenGL(kinect.kinectTextureId, kinect.kinectImageData.get());
            updateSkeletalData(skeletonFrame, kinect.kinectSensor);
            KinectStatusLabel->SetText("Kinect Status: Success!");
            std::cerr << "Attempted connection to kinect.... " << kinect.status_str(kinectStatus) << std::endl;    // DEBUG
        }
        else {
            KinectStatusLabel->SetText("Kinect Status: ERROR " + kinect.status_str(kinectStatus));
            std::cerr << "Attempted connection to kinect.... " << kinect.status_str(kinectStatus) << std::endl;    // DEBUG
        }
    }
    else {
        KinectStatusLabel->SetText("Kinect Status: ERROR KINECT NOT DETECTED");
        std::cerr << "Attempted connection to kinect.... UNDETECTED" << std::endl;    // DEBUG
    }
    // Reconnect Kinect Event Signal
    reconKinectButton->GetSignal(sfg::Widget::OnLeftClick).Connect([&kinect] {
        kinect.initialise();
    });

    //Initialise InputEmu and Trackers
    std::vector<KinectTrackedDevice> v_trackers{};
    vrinputemulator::VRInputEmulator inputEmulator;
    try {
        inputEmulator.connect();
    }
    catch (vrinputemulator::vrinputemulator_connectionerror e) {
        InputEmulatorStatusLabel->SetText("Input Emu Status: NOT Connected! Error " + std::to_string(e.errorcode) + " " + e.what() + "\n\n Is SteamVR open and InputEmulator installed?");
        std::cerr << "Attempted connection to Input Emulator" << std::to_string(e.errorcode) + " " + e.what() + "\n\n Is SteamVR open and InputEmulator installed?" << std::endl;
    }
    // Tracker Initialisation Lambda
    if (inputEmulator.isConnected()) {
        TrackerInitButton->GetSignal(sfg::Widget::OnLeftClick).Connect([&TrackerInitButton, &v_trackers, &inputEmulator] {
            TrackerInitButton->SetLabel("Trackers Initialised");
            KinectTrackedDevice leftFootTracker(inputEmulator, NUI_SKELETON_POSITION_FOOT_LEFT, NUI_SKELETON_POSITION_ANKLE_LEFT, false);
            KinectTrackedDevice rightFootTracker(inputEmulator, NUI_SKELETON_POSITION_FOOT_RIGHT, NUI_SKELETON_POSITION_ANKLE_RIGHT, false);
            KinectTrackedDevice hipTracker(inputEmulator, NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_SPINE, false);

            //KinectTrackedDevice kinectTrackerRef(inputEmulator, NUI_SKELETON_POSITION_HEAD, NUI_SKELETON_POSITION_HEAD, true);
            //setKinectTrackerProperties(kinectTrackerRef.deviceId);

            v_trackers.push_back(leftFootTracker);
            v_trackers.push_back(rightFootTracker);
            v_trackers.push_back(hipTracker);
            TrackerInitButton->SetState(sfg::Widget::State::INSENSITIVE);
        });
        InputEmulatorStatusLabel->SetText("Input Emu Status: Success!");
    }
    else {
        TrackerInitButton->SetLabel("Input Emulator not connected! Can't init trackers");
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
        SteamVRStatusLabel->SetText("VR Status: Success!");
        leftController.Connect(m_VRSystem);
        rightController.Connect(m_VRSystem);
        std::cerr << "Attempted connection to controllers.... " << std::endl;    // DEBUG
    }
    else {
        SteamVRStatusLabel->SetText("VR Status: ERROR " + std::to_string(eError));
    }
    std::cerr << "Attempted connection to vrsystem.... " << eError << std::endl;    // DEBUG
    //Controllers
    

    ReconControllersButton->GetSignal(sfg::Button::OnLeftClick).Connect([&rightController, &leftController, &m_VRSystem, &ReconControllersLabel] {
        std::stringstream stream;
        stream << "If controller input isn't working, press this to reconnect them.\n Make sure both are on, and not in standby.\n";
        if (rightController.Connect(m_VRSystem)) {
            stream << "RIGHT: OK!\t";
        }
        else {
            stream << "RIGHT: DISCONNECTED!\t";
        }
        if (leftController.Connect(m_VRSystem)) {
            stream << "LEFT: OK!\t";
        }
        else {
            stream << "LEFT: DISCONNECTED!\t";
        }
        ReconControllersLabel->SetText(stream.str());
    });

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
        // Update Kinect Status
        if (eError == vr::VRInitError_None) {
            rightController.update();
            leftController.update();
        }
        else {
            std::cerr << "Error updating controllers: Could not connect to the SteamVR system! OpenVR init error-code " << std::to_string(eError) << std::endl;
        }
        if (kinect.initStatus()) {
            HRESULT kinectStatus = kinect.kinectSensor->NuiStatus();
            if (kinectStatus == S_OK) {
                KinectStatusLabel->SetText("Kinect Status: Success!");
            }
            else {
                KinectStatusLabel->SetText("Kinect Status: ERROR " + kinect.status_str(kinectStatus));
            }
            updateSkeletalData(skeletonFrame, kinect.kinectSensor);
            if (!zeroed) {          //Initial attempt to allow user to get into position before setting the trackers -  ** zeroAll sets zeroed to true  **
                if (rightController.GetPress(vr::EVRButtonId::k_EButton_Grip)) {
                    ss << "Grip get!\n";
                    zeroAllTracking(skeletonFrame, m_VRSystem);
                }

            }
            if (KinectSettings::userChangingZero) {
                if (leftController.GetTouch(vr::EVRButtonId::k_EButton_SteamVR_Touchpad)) { //works
                    sf::Vector2f axis = leftController.GetControllerAxisValue(vr::EVRButtonId::k_EButton_SteamVR_Touchpad); //works
                    KinectSettings::trackedPositionOffset[0] += deltaScaled(1.0, deltaT) * axis.x;
                    KinectSettings::trackedPositionOffset[2] += deltaScaled(1.0, deltaT) * axis.y;
                }
                if (rightController.GetTouch(vr::EVRButtonId::k_EButton_SteamVR_Touchpad)) {
                    sf::Vector2f axis = rightController.GetControllerAxisValue(vr::EVRButtonId::k_EButton_SteamVR_Touchpad); //works
                    KinectSettings::trackedPositionOffset[1] += deltaScaled(1.0, deltaT) * axis.y;
                }
                if (rightController.GetTrigger()) {  //works
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
        }
        else {
            KinectStatusLabel->SetText("Kinect Status: ERROR KINECT NOT DETECTED");
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

