#pragma once
#include "stdafx.h"

#include "VRController.h"
#include "KinectToVR.h"
#include "KinectSettings.h"
#include "KinectHandlerBase.h"
#include "KinectTrackedDevice.h"
#include "KinectJoint.h"

#include <SFML/Graphics.hpp>
#include <SFML/Window/Mouse.hpp>
#include <SFML/OpenGL.hpp>
//GUI
#include <SFGUI\SFGUI.hpp>
#include <SFGUI/Widgets.hpp>
#include <string>

class GUIHandler {
private:
struct TempTracker {
        sfg::RadioButton::Ptr radioButton;
        int GUID = 404;
		KVR::KinectJointType joint0 = KVR::KinectJointType::Head;
		KVR::KinectJointType joint1 = KVR::KinectJointType::Head;
		KVR::KinectDeviceRole role= KVR::KinectDeviceRole::Unassigned;
        bool isController = false;
    };
public:
    GUIHandler() {
        guiWindow->SetTitle("Main Window");

        setDefaultSignals();
        
        setLineWrapping();
        packElementsIntoMainBox();
        packElementsIntoAdvTrackerBox();
		packElementsIntoPlayspaceMovementBox();
        setRequisitions();

        mainNotebook->AppendPage(mainGUIBox, sfg::Label::Create("KinectToVR"));
        mainNotebook->AppendPage(advancedTrackerBox, sfg::Label::Create("Adv. Trackers"));
		mainNotebook->AppendPage(playspaceMovementBox, sfg::Label::Create("Playspace Movement"));

        guiWindow->Add(mainNotebook);
        guiDesktop.Add(guiWindow);


        setScale();

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
void setScale() {
    guiWindow->SetRequisition(sf::Vector2f(.6f*SFMLsettings::m_window_width, .6f*SFMLsettings::m_window_height));
    //Text scaling
    /*
    Window > * > * > Label{
        FontSize : 18;
    /*FontName: data/linden_hill.otf;*/
    float defaultFontSize = 12.f / 1920.f; // Percentage relative to 1080p
    float scaledFontSize = defaultFontSize * (SFMLsettings::m_window_width / SFMLsettings::windowScale);
    guiDesktop.SetProperty("Window Label, Box, Button, Notebook, CheckButton, ToggleButton, Label, RadioButton, ComboBox, SpinButton", "FontSize", scaledFontSize);
}
void toggleRotButton() {
    KinectRotButton->SetActive(KinectSettings::adjustingKinectRepresentationRot);
}
void togglePosButton() {
    KinectPosButton->SetActive(KinectSettings::adjustingKinectRepresentationPos);
}
void setDefaultSignals() {
    //Post VR Tracker Initialisation
    hidePostTrackerInitUI();

    //Signals
    EnableGamepadButton->GetSignal(sfg::ToggleButton::OnToggle).Connect([this] {
        if (EnableGamepadButton->IsActive()) {
            SFMLsettings::usingGamepad = true;
        }
        else {
            SFMLsettings::usingGamepad = false;
        }
    });

	ShowSkeletonButton->GetSignal(sfg::Widget::OnLeftClick).Connect([] {
		//toggle(KinectSettings::isSkeletonDrawn);
	});

    KinectRotButton->GetSignal(sfg::ToggleButton::OnToggle).Connect([this] {
        if (KinectRotButton->IsActive()) {
            KinectSettings::adjustingKinectRepresentationRot = true;
        }
        else
            KinectSettings::adjustingKinectRepresentationRot = false;
    });
    KinectPosButton->GetSignal(sfg::ToggleButton::OnToggle).Connect([this]
    {    if (KinectPosButton->IsActive()) {
        KinectSettings::adjustingKinectRepresentationPos = true;
    }
    else
        KinectSettings::adjustingKinectRepresentationPos = false;
    });
    IgnoreInferredCheckButton->GetSignal(sfg::ToggleButton::OnToggle).Connect([this] {
        if (IgnoreInferredCheckButton->IsActive()) {
            KinectSettings::ignoreInferredPositions = true;    // No longer stops updating trackers when Kinect isn't sure about a position
        }
        else {
            KinectSettings::ignoreInferredPositions = false;
        }
    });
    IgnoreRotSmoothingCheckButton->GetSignal(sfg::ToggleButton::OnToggle).Connect([this] {
        if (IgnoreRotSmoothingCheckButton->IsActive()) {
            KinectSettings::ignoreRotationSmoothing = true;    // No longer tries to smooth the joints
        }
        else {
            KinectSettings::ignoreRotationSmoothing = false;
        }
    });


    AddHandControllersToList->GetSignal(sfg::Widget::OnLeftClick).Connect([this] {
        //Add a left and right hand tracker as a controller
        addTrackerToList(KVR::KinectJointType::HandLeft, KVR::KinectDeviceRole::LeftHand, true);
        addTrackerToList(KVR::KinectJointType::HandRight, KVR::KinectDeviceRole::RightHand, true);
    });
    AddLowerTrackersToList->GetSignal(sfg::Widget::OnLeftClick).Connect([this] {
        addTrackerToList(KVR::KinectJointType::AnkleLeft, KVR::KinectDeviceRole::LeftFoot, false);
        addTrackerToList(KVR::KinectJointType::AnkleRight, KVR::KinectDeviceRole::RightFoot, false);
        addTrackerToList(KVR::KinectJointType::SpineBase, KVR::KinectDeviceRole::Hip, false);
    });
    AddTrackerToListButton->GetSignal(sfg::Widget::OnLeftClick).Connect([this] {
        addUserTrackerToList();
    });
    RemoveTrackerFromListButton->GetSignal(sfg::Widget::OnLeftClick).Connect([this] {
        int i = 0;
        for (; i < TrackersToBeInitialised.size(); ++i) {
            if (TrackersToBeInitialised[i].radioButton->IsActive()) {
                TrackersToBeInitialised[i].radioButton->Show(false);
                TrackersToBeInitialised.erase(TrackersToBeInitialised.begin() + i);
                break;
            }
        }
        //updateTempTrackerIDs();
        //updateTempTrackerButtonGroups();
    });

    HipScale->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this] {
        // Update the Global hip offset
        KinectSettings::hipRoleHeightAdjust = HipScale->GetValue();
    }
    );

	// Playspace Movement Bindings
	ButtonListLeftHand->GetSignal(sfg::ComboBox::OnSelect).Connect([this] {
		KinectSettings::leftHandPlayspaceMovementButton = ButtonListLeftHand->GetSelectedItem();
	}
	);
	ButtonListRightHand->GetSignal(sfg::ComboBox::OnSelect).Connect([this] {
		KinectSettings::rightHandPlayspaceMovementButton = ButtonListRightHand->GetSelectedItem();
	}
	);
	ButtonListLeftFoot->GetSignal(sfg::ComboBox::OnSelect).Connect([this] {
		KinectSettings::leftFootPlayspaceMovementButton = ButtonListLeftFoot->GetSelectedItem();
	}
	);
	ButtonListRightFoot->GetSignal(sfg::ComboBox::OnSelect).Connect([this] {
		KinectSettings::rightFootPlayspaceMovementButton = ButtonListRightFoot->GetSelectedItem();
	}
	);

}

void addUserTrackerToList() {
    TempTracker temp;
    temp.GUID = TrackersToBeInitialised.size();
    temp.isController = IsControllerButton->IsActive();
    temp.joint0 = KVR::KinectJointType(BonesList->GetSelectedItem());
	temp.joint1 = temp.joint0;	//TEMP BEFORE SELECTION IMPLEMENTED
    temp.role = KVR::KinectDeviceRole(RolesList->GetSelectedItem());
    updateTrackerLists(temp);
}
void addTrackerToList(KVR::KinectJointType joint, KVR::KinectDeviceRole role, bool isController) {
	TempTracker temp;
    temp.GUID = TrackersToBeInitialised.size();
    temp.isController = isController;
    temp.joint0 = joint;
	temp.joint1 = temp.joint0; //TEMP BEFORE SELECTION IMPLEMENTED
    temp.role = role;

    updateTrackerLists(temp);
}

void updateTrackerLists(TempTracker &temp) {
    // Display a radio button menu where selecting each button selects that tracker
    // Displays the joint of each tracker and (Tracker)/(Controller)
    std::stringstream ss;
    if (temp.isController)
        ss << " (Tracked Controller) ";
    else
        ss << " (Tracker) ";
    ss << "(Role: " << KVR::KinectDeviceRoleName[int(temp.role)] << ") ";
    temp.radioButton = sfg::RadioButton::Create(KVR::KinectJointName[int(temp.joint0)] + ss.str());
    if (TrackersToBeInitialised.size()) {
        auto group = TrackersToBeInitialised.back().radioButton->GetGroup();
        temp.radioButton->SetGroup(group);
    }

    TrackerList->Pack(temp.radioButton);

    TrackersToBeInitialised.push_back(temp);
}
void setCalibrationSignal() {
    CalibrationSetButton->GetSignal(sfg::Widget::OnLeftClick).Connect(
        [this] {
        //NEED TO VALIDATE THESE INPUTS
        std::stringstream ss;
        ss << CalibrationEntryPosX->GetText().toAnsiString();
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
void setTrackerInitButtonSignal(vrinputemulator::VRInputEmulator &inputE, std::vector<KVR::KinectTrackedDevice> &v_trackers) {
    TrackerInitButton->GetSignal(sfg::Widget::OnLeftClick).Connect([this, &v_trackers, &inputE] {
        TrackerInitButton->SetLabel("Trackers Initialised");
        if (TrackersToBeInitialised.empty()) {
            spawnDefaultLowerBodyTrackers(inputE, v_trackers);
        }
        else {
            for (TempTracker tracker : TrackersToBeInitialised) {
                spawnAndConnectTracker(inputE, v_trackers, tracker.joint0, tracker.joint1, tracker.role);
                if (tracker.isController) {
                    setDeviceProperty(v_trackers.back().deviceId, 1029, "int32", "2"); // Device Class: Controller
                    if (tracker.role == KVR::KinectDeviceRole::LeftHand) {
                        setDeviceProperty(v_trackers.back().deviceId, 3007, "int32", "1"); // ControllerRole Left
                    }
                    else if (tracker.role == KVR::KinectDeviceRole::RightHand) {
                        setDeviceProperty(v_trackers.back().deviceId, 3007, "int32", "2"); // ControllerRole Right
                    }
                }
            }
        }
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

    setHipScaleBox();
    mainGUIBox->Pack(ShowSkeletonButton);

    mainGUIBox->Pack(EnableGamepadButton);
    mainGUIBox->Pack(ReconControllersLabel);
    mainGUIBox->Pack(ReconControllersButton);

    mainGUIBox->Pack(KinectRotLabel);
    mainGUIBox->Pack(KinectRotButton);

    mainGUIBox->Pack(KinectPosLabel);
    mainGUIBox->Pack(KinectPosButton);

    mainGUIBox->Pack(InferredLabel);
    mainGUIBox->Pack(IgnoreInferredCheckButton);
    mainGUIBox->Pack(IgnoreRotSmoothingCheckButton);

    //mainGUIBox->Pack(CalibrationSettingsLabel); //Calibration left out of main UI because it is not currently implemented
    calibrationBox->Pack(CalibrationSetButton);
    calibrationBox->Pack(CalibrationEntryPosX);
    calibrationBox->Pack(CalibrationEntryPosY);
    calibrationBox->Pack(CalibrationEntryPosZ);
    calibrationBox->Pack(CalibrationEntryRotX);
    calibrationBox->Pack(CalibrationEntryRotY);
    calibrationBox->Pack(CalibrationEntryRotZ);

    //mainGUIBox->Pack(calibrationBox); //Calibration left out of main UI because it is not currently implemented
}

void setHipScaleBox() {
    auto HipLabel = sfg::Label::Create("Vertical Hip Adjustment (metres)");
    HipScale->SetDigits(3);
    
    HipScaleBox->Pack(HipLabel, false, false);
    HipScaleBox->Pack(HipScale);
    mainGUIBox->Pack(HipScaleBox);
}

void packElementsIntoPlayspaceMovementBox() {
	setButtonListLeftItems(ButtonListLeftHand);
	setButtonListRightItems(ButtonListRightHand);
	setButtonListRightItems(ButtonListRightFoot);
	setButtonListLeftItems(ButtonListLeftFoot);

	ButtonListRightHand->SelectItem(KinectSettings::rightHandPlayspaceMovementButton);
	ButtonListLeftHand->SelectItem(KinectSettings::leftHandPlayspaceMovementButton);
	ButtonListLeftFoot->SelectItem(KinectSettings::leftFootPlayspaceMovementButton);
	ButtonListRightFoot->SelectItem(KinectSettings::rightFootPlayspaceMovementButton);

	ButtonListRightHand->SelectItem(KinectSettings::rightHandPlayspaceMovementButton);
	ButtonListLeftHand->SelectItem(KinectSettings::leftHandPlayspaceMovementButton);
	ButtonListRightFoot->SelectItem(KinectSettings::rightFootPlayspaceMovementButton);
	ButtonListLeftFoot->SelectItem(KinectSettings::leftFootPlayspaceMovementButton);

	playspaceMovementBox->Pack(LeftHandPlayspaceMovementEnabled);
	playspaceMovementBox->Pack(ButtonListLeftHand);
	playspaceMovementBox->Pack(RightHandPlayspaceMovementEnabled);
	playspaceMovementBox->Pack(ButtonListRightHand);
	playspaceMovementBox->Pack(LeftFootPlayspaceMovementEnabled);
	playspaceMovementBox->Pack(ButtonListLeftFoot);
	playspaceMovementBox->Pack(RightFootPlayspaceMovementEnabled);
	playspaceMovementBox->Pack(ButtonListRightFoot);

}

void packElementsIntoAdvTrackerBox() {
    advancedTrackerBox->Pack(AddHandControllersToList);
    advancedTrackerBox->Pack(AddLowerTrackersToList);

    advancedTrackerBox->Pack(TrackerList);

    TrackerList->Pack(TrackerListLabel);

    setBonesListItems();
    setRolesListItems(RolesList);

    TrackerListOptionsBox->Pack(BonesList);
    TrackerListOptionsBox->Pack(RolesList);
    TrackerListOptionsBox->Pack(IsControllerButton);
    TrackerListOptionsBox->Pack(AddTrackerToListButton);
    TrackerListOptionsBox->Pack(RemoveTrackerFromListButton);

    advancedTrackerBox->Pack(TrackerListOptionsBox);
}

void setBonesListItems() {
    using namespace KVR;
    for (int i = 0; i < KinectJointCount; ++i) {
        BonesList->AppendItem(KinectJointName[i]);
    }
    // Set as default - to prevent garbage additions 
    BonesList->SelectItem(0);
}

void setRolesListItems(sfg::ComboBox::Ptr comboBox) {
    for (int i = 0; i < (int)KVR::KinectDeviceRole::Count; ++i) {
		comboBox->AppendItem(KVR::KinectDeviceRoleName[i]);
    }
    // Set as default - to prevent garbage additions 
	comboBox->SelectItem(0);
}

void setButtonListRightItems(sfg::ComboBox::Ptr comboBox) {
	// FIXME: Actually do button/controller detection stuff
	comboBox->AppendItem("Disabled");
	for (int i = 0; i < 16; i++) {
		std::stringstream ss;
		ss << "Right Controller: Button ";
		ss << i;
		comboBox->AppendItem(ss.str());
	}
	comboBox->SelectItem(0);
}

void setButtonListLeftItems(sfg::ComboBox::Ptr comboBox) {
	// FIXME: Actually do button/controller detection stuff
	comboBox->AppendItem("Disabled");
	for (int i = 0; i < 16; i++) {
		std::stringstream ss;
		ss << "Left Controller: Button ";
		ss << i;
		comboBox->AppendItem(ss.str());
	}
	comboBox->SelectItem(0);
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
    sf::Font mainGUIFont;
    sfg::SFGUI sfguiRef;
    sfg::Window::Ptr guiWindow = sfg::Window::Create();
    sfg::Notebook::Ptr mainNotebook = sfg::Notebook::Create();

    sfg::Desktop guiDesktop;

    sfg::Box::Ptr mainGUIBox = sfg::Box::Create(sfg::Box::Orientation::VERTICAL, 5.f);
    sfg::Box::Ptr calibrationBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL, 5.f);
    sfg::Box::Ptr advancedTrackerBox = sfg::Box::Create(sfg::Box::Orientation::VERTICAL, 5.f);
	sfg::Box::Ptr playspaceMovementBox = sfg::Box::Create(sfg::Box::Orientation::VERTICAL, 5.f);
    //Statuses
    sfg::Label::Ptr KinectStatusLabel = sfg::Label::Create();
    sfg::Label::Ptr SteamVRStatusLabel = sfg::Label::Create();
    sfg::Label::Ptr InputEmulatorStatusLabel = sfg::Label::Create();

    sfg::Button::Ptr reconKinectButton = sfg::Button::Create("Reconnect Kinect");
    sfg::Button::Ptr TrackerInitButton = sfg::Button::Create("Initialise SteamVR Kinect Trackers - HIT ME");

    sfg::Button::Ptr ShowSkeletonButton = sfg::CheckButton::Create("Show/Hide Skeleton Tracking: MAY CAUSE LAG IN TRACKERS");

    //Zeroing
    sfg::Label::Ptr KinectRotLabel = sfg::Label::Create("Calibrate the rotation of the Kinect sensor with the controller thumbsticks. Press the trigger to confirm.");
    sfg::CheckButton::Ptr KinectRotButton = sfg::CheckButton::Create("Enable Kinect Rotation Calibration");


    //Position Adjust
    sfg::Label::Ptr KinectPosLabel = sfg::Label::Create("Calibrate the position of the Kinect sensor with the controller thumbsticks. Press the trigger to confirm.");
    sfg::CheckButton::Ptr KinectPosButton = sfg::CheckButton::Create("Enable Kinect Position Calibration");


    // Controllers
    sfg::CheckButton::Ptr EnableGamepadButton = sfg::CheckButton::Create("Enable Gamepad Calibration Controls");
    sfg::Label::Ptr ReconControllersLabel = sfg::Label::Create("If controller input isn't working, press this to reconnect them.\n Make sure both are on, and not in standby.");
    sfg::Button::Ptr ReconControllersButton = sfg::Button::Create("Reconnect VR Controllers");


    // Allows for unrestricted tracking, but may be unstable
    sfg::Label::Ptr InferredLabel = sfg::Label::Create("Checking this stops the trackers if it's not absolutely 100% sure where they are. Leaving this disabled may cause better tracking in poorly lit environments, but at the cost of slight jerks aside sometimes.");
    sfg::CheckButton::Ptr IgnoreInferredCheckButton = sfg::CheckButton::Create("Disable Raw Positional Tracking");
    sfg::CheckButton::Ptr IgnoreRotSmoothingCheckButton = sfg::CheckButton::Create("Enable Raw Orientation Tracking (Rotation smoothing is in development!!!)");

    sfg::Label::Ptr InstructionsLabel = sfg::Label::Create("Stand in front of the Kinect sensor.\n If the trackers don't update, then try crouching slightly until they move.\n\n Calibration: The arrow represents the position and rotation of the Kinect - match it as closely to real life as possible for the trackers to line up.\n\n The arrow pos/rot is set with the thumbsticks on the controllers, and confirmed with the trigger.");    //Blegh - There has to be a better way than this, maybe serialization?

    sfg::Label::Ptr CalibrationSettingsLabel = sfg::Label::Create("These settings are here for manual entry, and saving until a proper configuration system is implemented, you can use this to quickly calibrate if your Kinect is in the same place. (Rotation is in radians, and Pos should be in meters roughly)");
    sfg::Entry::Ptr CalibrationEntryPosX = sfg::Entry::Create("");
    sfg::Entry::Ptr CalibrationEntryPosY = sfg::Entry::Create("");
    sfg::Entry::Ptr CalibrationEntryPosZ = sfg::Entry::Create("");

    sfg::Entry::Ptr CalibrationEntryRotX = sfg::Entry::Create("");
    sfg::Entry::Ptr CalibrationEntryRotY = sfg::Entry::Create("");
    sfg::Entry::Ptr CalibrationEntryRotZ = sfg::Entry::Create("");

    sfg::Button::Ptr CalibrationSetButton = sfg::Button::Create();

	// Playspace Movement
	sfg::Label::Ptr LeftHandPlayspaceMovementEnabled = sfg::Label::Create("Left Hand Playspace Movement");
	sfg::Label::Ptr RightHandPlayspaceMovementEnabled = sfg::Label::Create("Right Hand Playspace Movement");
	sfg::Label::Ptr RightFootPlayspaceMovementEnabled = sfg::Label::Create("Right Foot Playspace Movement");
	sfg::Label::Ptr LeftFootPlayspaceMovementEnabled = sfg::Label::Create("Left Foot Playspace Movement");
	sfg::ComboBox::Ptr ButtonListLeftHand = sfg::ComboBox::Create();
	sfg::ComboBox::Ptr ButtonListRightHand = sfg::ComboBox::Create();
	sfg::ComboBox::Ptr ButtonListRightFoot = sfg::ComboBox::Create();
	sfg::ComboBox::Ptr ButtonListLeftFoot = sfg::ComboBox::Create();

    //Adv Trackers
    sfg::Button::Ptr AddHandControllersToList = sfg::Button::Create("Add Hand Controllers");
    sfg::Button::Ptr AddLowerTrackersToList = sfg::Button::Create("Add Lower Body Trackers");

    sfg::Box::Ptr TrackerList = sfg::Box::Create(sfg::Box::Orientation::VERTICAL, 5);
    sfg::Label::Ptr TrackerListLabel = sfg::Label::Create("Trackers to be spawned:");

    sfg::Box::Ptr TrackerListOptionsBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL, 5);
    sfg::SpinButton::Ptr HipScale = sfg::SpinButton::Create(sfg::Adjustment::Create(KinectSettings::hipRoleHeightAdjust, -1.f, 1.f, .01f));
    sfg::Box::Ptr HipScaleBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);

    sfg::ComboBox::Ptr BonesList = sfg::ComboBox::Create();
    sfg::ComboBox::Ptr RolesList = sfg::ComboBox::Create();
    sfg::CheckButton::Ptr IsControllerButton = sfg::CheckButton::Create("Controller");
    sfg::Button::Ptr AddTrackerToListButton = sfg::Button::Create("Add");
    sfg::Button::Ptr RemoveTrackerFromListButton = sfg::Button::Create("Remove");

    std::vector<TempTracker> TrackersToBeInitialised;


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
        IgnoreRotSmoothingCheckButton->Show(show);
        HipScale->Show(show);
        HipScaleBox->Show(show);

        calibrationBox->Show(show);
    }
    void hidePostTrackerInitUI() {
        showPostTrackerInitUI(false);
    }
};