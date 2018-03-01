#pragma once
#include "stdafx.h"

#include "VRController.h"
#include "KinectToVR.h"
#include "KinectSettings.h"
#include "KinectHandlerBase.h"
#include "KinectTrackedDevice.h"

#include <SFML/Graphics.hpp>
#include <SFML/Window/Mouse.hpp>
#include <SFML/OpenGL.hpp>
//GUI
#include <SFGUI\SFGUI.hpp>
#include <SFGUI/Widgets.hpp>
#include <string>

class GUIHandler {
public:
    GUIHandler() {
        guiWindow->SetTitle("Main Window");

        setDefaultSignals();
        setLineWrapping();
        packElementsIntoMainBox();
        setRequisitions();

        guiWindow->Add(mainGUIBox);
        guiDesktop.Add(guiWindow);
        bool b = guiDesktop.LoadThemeFromFile("main_theme.theme");
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
    void setRequisitions() {
        CalibrationEntryPosX->SetRequisition(sf::Vector2f(40.f, 0.f));
        CalibrationEntryPosY->SetRequisition(sf::Vector2f(40.f, 0.f));
        CalibrationEntryPosZ->SetRequisition(sf::Vector2f(40.f, 0.f));
        CalibrationEntryRotX->SetRequisition(sf::Vector2f(40.f, 0.f));
        CalibrationEntryRotY->SetRequisition(sf::Vector2f(40.f, 0.f));
        CalibrationEntryRotZ->SetRequisition(sf::Vector2f(40.f, 0.f));
    }
    void setDefaultSignals() {
        //Post VR Tracker Initialisation
        hidePostTrackerInitUI();

        //Signals
        
        ShowSkeletonButton->GetSignal(sfg::Widget::OnLeftClick).Connect([] {
            toggle(KinectSettings::isSkeletonDrawn);
        });

        KinectRotButton->GetSignal(sfg::Widget::OnLeftClick).Connect([] {
            KinectSettings::adjustingKinectRepresentationRot = true;
        });
        KinectPosButton->GetSignal(sfg::Widget::OnLeftClick).Connect([] {    KinectSettings::adjustingKinectRepresentationPos = true;
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
    void setCalibrationSignal() {
        CalibrationSetButton->GetSignal(sfg::Widget::OnLeftClick).Connect(
            [this] {
            //NEED TO VALIDATE THESE INPUTS
            std::stringstream ss;
            ss <<  CalibrationEntryPosX->GetText().toAnsiString();
            KinectSettings::kinectRepPosition.v[0] = std::stof(ss.str());
            ss.clear();
            ss << CalibrationEntryPosY->GetText().toAnsiString();
            KinectSettings::kinectRepPosition.v[1] = std::stof(ss.str());
            ss.clear();
            ss << CalibrationEntryPosZ->GetText().toAnsiString();
            KinectSettings::kinectRepPosition.v[2] = std::stof(ss.str());
            ss.clear();

            ss << CalibrationEntryRotX->GetText().toAnsiString();
            KinectSettings::kinectRadRotation.v[0] = std::stof(ss.str());
            ss.clear();
            ss << CalibrationEntryRotY->GetText().toAnsiString();
            KinectSettings::kinectRadRotation.v[1] = std::stof(ss.str());
            ss.clear();
            ss << CalibrationEntryRotZ->GetText().toAnsiString();
            KinectSettings::kinectRadRotation.v[2] = std::stof(ss.str());
            ss.clear();
        });
    }
    void setKinectButtonSignal(KinectHandlerBase& kinect) {
        reconKinectButton->GetSignal(sfg::Widget::OnLeftClick).Connect([&kinect] {
            kinect.initialise();
        });
    }
    void setTrackerInitButtonSignal(vrinputemulator::VRInputEmulator &inputE, std::vector<KinectTrackedDevice> &v_trackers ) {
        TrackerInitButton->GetSignal(sfg::Widget::OnLeftClick).Connect([this, &v_trackers, &inputE] {
            TrackerInitButton->SetLabel("Trackers Initialised");
            spawnDefaultFullBodyTrackers(inputE, v_trackers);
            spawnAndConnectKinectTracker(inputE, v_trackers);

            showPostTrackerInitUI();

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

        CalibrationSettingsLabel->SetLineWrap(true);
        CalibrationSettingsLabel->SetRequisition(sf::Vector2f(600.f, 20.f));
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

        mainGUIBox->Pack(KinectRotLabel);
        mainGUIBox->Pack(KinectRotButton);

        mainGUIBox->Pack(KinectPosLabel);
        mainGUIBox->Pack(KinectPosButton);

        mainGUIBox->Pack(InferredLabel);
        mainGUIBox->Pack(IgnoreInferredCheckButton);

        mainGUIBox->Pack(CalibrationSettingsLabel);
        calibrationBox->Pack(CalibrationSetButton);
        calibrationBox->Pack(CalibrationEntryPosX);
        calibrationBox->Pack(CalibrationEntryPosY);
        calibrationBox->Pack(CalibrationEntryPosZ);
        calibrationBox->Pack(CalibrationEntryRotX);
        calibrationBox->Pack(CalibrationEntryRotY);
        calibrationBox->Pack(CalibrationEntryRotZ);

        mainGUIBox->Pack(calibrationBox);
    }

    void updateKinectStatusLabel(KinectHandlerBase& kinect) {
        if (kinect.isInitialised()) {
            HRESULT status = kinect.getStatusResult();
            switch (status) {
            case S_OK:
                KinectStatusLabel->SetText("Kinect Status: Success!");
                break;
            default:
                KinectStatusLabel->SetText("Kinect Status: ERROR " + kinect.statusResultString(status));
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
    sfg::Box::Ptr calibrationBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL, 5.f);
    //Statuses
    sfg::Label::Ptr KinectStatusLabel = sfg::Label::Create();
    sfg::Label::Ptr SteamVRStatusLabel = sfg::Label::Create();
    sfg::Label::Ptr InputEmulatorStatusLabel = sfg::Label::Create();

    sfg::Button::Ptr reconKinectButton = sfg::Button::Create("Reconnect Kinect");
    sfg::Button::Ptr TrackerInitButton = sfg::Button::Create("Initialise SteamVR Kinect Trackers - HIT ME");

    sfg::Button::Ptr ShowSkeletonButton = sfg::CheckButton::Create("Show/Hide Skeleton Tracking: MAY CAUSE LAG IN TRACKERS");

    //Zeroing
    sfg::Label::Ptr KinectRotLabel = sfg::Label::Create("Calibrate the rotation of the Kinect sensor with the controller thumbsticks. Press the trigger to confirm.");
    sfg::Button::Ptr KinectRotButton = sfg::Button::Create("Enable Kinect Rotation Calibration");


    //Position Adjust
    sfg::Label::Ptr KinectPosLabel = sfg::Label::Create("Calibrate the position of the Kinect sensor with the controller thumbsticks. Press the trigger to confirm.");
    sfg::Button::Ptr KinectPosButton = sfg::Button::Create("Enable Kinect Position Calibration");


    //Redetect Controllers
    sfg::Label::Ptr ReconControllersLabel = sfg::Label::Create("If controller input isn't working, press this to reconnect them.\n Make sure both are on, and not in standby.");
    sfg::Button::Ptr ReconControllersButton = sfg::Button::Create("Reconnect VR Controllers");


    // Allows for unrestricted tracking, but may be unstable
    sfg::Label::Ptr InferredLabel = sfg::Label::Create("Checking this makes the trackers directly copy the Kinect movement. This may have the benefit of reducing lag, and improving tracking when partially occluded, but may also have the consequence of spazzing wildly if tracking is lost.");
    sfg::CheckButton::Ptr IgnoreInferredCheckButton = sfg::CheckButton::Create("Enable Raw Tracking");

    sfg::Label::Ptr InstructionsLabel = sfg::Label::Create("Stand in front of the Kinect sensor.\n If the trackers don't update, then try crouching slightly until they move.\n\n Calibration: The arrow represents the position and rotation of the Kinect - match it as closely to real life as possible for the trackers to line up.\n\n The arrow pos/rot is set with the thumbsticks on the controllers, and confirmed with the trigger.");    //Blegh - There has to be a better way than this, maybe serialization?

    sfg::Label::Ptr CalibrationSettingsLabel = sfg::Label::Create("These settings are here for manual entry, and saving until a proper configuration system is implemented, you can use this to quickly calibrate if your Kinect is in the same place. (Rotation is in radians, and Pos should be in meters roughly)");
    sfg::Entry::Ptr CalibrationEntryPosX = sfg::Entry::Create("");
    sfg::Entry::Ptr CalibrationEntryPosY = sfg::Entry::Create("");
    sfg::Entry::Ptr CalibrationEntryPosZ = sfg::Entry::Create("");

    sfg::Entry::Ptr CalibrationEntryRotX = sfg::Entry::Create("");
    sfg::Entry::Ptr CalibrationEntryRotY = sfg::Entry::Create("");
    sfg::Entry::Ptr CalibrationEntryRotZ = sfg::Entry::Create("");

    sfg::Button::Ptr CalibrationSetButton = sfg::Button::Create();

    void updateKinectStatusLabelDisconnected() {
        KinectStatusLabel->SetText("Kinect Status: ERROR KINECT NOT DETECTED");
    }
    void showPostTrackerInitUI(bool show = true) {
        InstructionsLabel->Show(show);
        KinectRotLabel->Show(show);
        KinectRotButton->Show(show);
        KinectPosLabel->Show(show);
        KinectPosButton->Show(show);
        ReconControllersLabel->Show(show);
        ReconControllersButton->Show(show);
        InferredLabel->Show(show);
        IgnoreInferredCheckButton->Show(show);

        calibrationBox->Show(show);
    }
    void hidePostTrackerInitUI() {
        showPostTrackerInitUI(false);
    }
};