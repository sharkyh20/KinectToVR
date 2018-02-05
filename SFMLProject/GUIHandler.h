#pragma once
#include "stdafx.h"

#include "VRController.h"
#include "KinectToVR.h"

#include <SFML/Graphics.hpp>
#include <SFML/Window/Mouse.hpp>
#include <SFML/OpenGL.hpp>
//GUI
#include <SFGUI\SFGUI.hpp>
#include <SFGUI/Widgets.hpp>

class GUIHandler {
public:
    GUIHandler() {
        guiWindow->SetTitle("Main Window");

        setDefaultSignals();
        setLineWrapping();
        packElementsIntoMainBox();

        guiWindow->Add(mainGUIBox);
        guiDesktop.Add(guiWindow);
        guiDesktop.Update(0.f);
    }
    ~GUIHandler() {}

    void display(sf::RenderWindow &window) {
        sfguiRef.Display(window);
    }

    void desktopHandleEvents(sf::Event event) {
        guiDesktop.HandleEvent(event);
    }
    void updateDesktop(float d) {
        guiDesktop.Update(d);
    }

    void setDefaultSignals() {
        ShowSkeletonButton->GetSignal(sfg::Widget::OnLeftClick).Connect([] {
            toggle(KinectSettings::isSkeletonDrawn);
        });
        ZeroButton->GetSignal(sfg::Widget::OnLeftClick).Connect([] {
            zeroed = false;
        });
        PositionAdjustButton->GetSignal(sfg::Widget::OnLeftClick).Connect([] {    KinectSettings::userChangingZero = true;
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
    void setKinectButtonSignal(KinectHandler& kinect) {
        reconKinectButton->GetSignal(sfg::Widget::OnLeftClick).Connect([&kinect] {
            kinect.initialise();
        });
    }
    void setTrackerInitButtonSignal(vrinputemulator::VRInputEmulator &inputE, std::vector<KinectTrackedDevice> &v_trackers ) {
        TrackerInitButton->GetSignal(sfg::Widget::OnLeftClick).Connect([this, &v_trackers, &inputE] {
            TrackerInitButton->SetLabel("Trackers Initialised");
            KinectTrackedDevice leftFootTracker(inputE, KinectJointType::FootLeft, KinectJointType::AnkleLeft, false, KinectVersion::Version1);
            KinectTrackedDevice rightFootTracker(inputE, KinectJointType::FootRight, KinectJointType::AnkleRight, false, KinectVersion::Version1);
            KinectTrackedDevice hipTracker(inputE, KinectJointType::SpineBase, KinectJointType::SpineMid, false, KinectVersion::Version1);

            //KinectTrackedDevice kinectTrackerRef(inputEmulator, NUI_SKELETON_POSITION_HEAD, NUI_SKELETON_POSITION_HEAD, true);
            //setKinectTrackerProperties(kinectTrackerRef.deviceId);

            v_trackers.push_back(leftFootTracker);
            v_trackers.push_back(rightFootTracker);
            v_trackers.push_back(hipTracker);
            TrackerInitButton->SetState(sfg::Widget::State::INSENSITIVE);
        });
    }
    void updateTrackerInitButtonLabelFail() {
        TrackerInitButton->SetLabel("Input Emulator not connected! Can't init trackers");
    }

    void setReconnectControllerButtonSignal(VRcontroller& left, VRcontroller& right, vr::IVRSystem* &sys
    ) {
        ReconControllersButton->GetSignal(sfg::Button::OnLeftClick).Connect([&left, &right, &sys, this] {
            std::stringstream stream;
            stream << "If controller input isn't working, press this to reconnect them.\n Make sure both are on, and not in standby.\n";
            if (right.Connect(sys)) {
                stream << "RIGHT: OK!\t";
            }
            else {
                stream << "RIGHT: DISCONNECTED!\t";
            }
            if (left.Connect(sys)) {
                stream << "LEFT: OK!\t";
            }
            else {
                stream << "LEFT: DISCONNECTED!\t";
            }
            ReconControllersLabel->SetText(stream.str());
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

    void updateKinectStatusLabel(KinectHandler& kinect) {
        // TODO UPDATE KINECT INIT WITH THIS
        if (kinect.initStatus()) {
            HRESULT status = kinect.kinectSensor->NuiStatus();
            switch (status) {
            case S_OK:
                KinectStatusLabel->SetText("Kinect Status: Success!");
                break;
            default:
                KinectStatusLabel->SetText("Kinect Status: ERROR " + kinect.status_str(status));
                break;
            }
        }
        else
            updateKinectStatusLabelDisconnected();
    }


    void updateEmuStatusLabelError(vrinputemulator::vrinputemulator_connectionerror e) {
        InputEmulatorStatusLabel->SetText("Input Emu Status: NOT Connected! Error " + std::to_string(e.errorcode) + " " + e.what() + "\n\n Is SteamVR open and InputEmulator installed?");
    }
    void updateEmuStatusLabelSuccess() {
        InputEmulatorStatusLabel->SetText("Input Emu Status: Success!");
    }

    void updateVRStatusLabel(vr::EVRInitError eError) {
        if (eError == vr::VRInitError_None)
            SteamVRStatusLabel->SetText("VR Status: Success!");
        else
            SteamVRStatusLabel->SetText("VR Status: ERROR " + std::to_string(eError));
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


    void updateKinectStatusLabelDisconnected() {
        KinectStatusLabel->SetText("Kinect Status: ERROR KINECT NOT DETECTED");
    }
};