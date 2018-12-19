#pragma once

#include "VRController.h"

#include "KinectSettings.h"
#include "KinectHandlerBase.h"
#include "KinectTrackedDevice.h"
#include "KinectJoint.h"
#include "ColorTracker.h"


#include "TrackingMethod.h"
#include "DeviceHandler.h"
#include "PSMoveHandler.h"

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

        KVR::JointPositionTrackingOption positionTrackingOption = KVR::JointPositionTrackingOption::Skeleton;
        uint32_t positionGlobalDeviceId = 0;

        KVR::JointRotationTrackingOption rotationTrackingOption = KVR::JointRotationTrackingOption::Skeleton;
        uint32_t rotationGlobalDeviceId = 0;

		KVR::KinectDeviceRole role = KVR::KinectDeviceRole::Unassigned;
        bool isController = false;
    };
public:
    GUIHandler() {
        guiWindow->SetTitle("Main Window");

        setDefaultSignals();
        
        setLineWrapping();
        packElementsIntoMainBox();
        packElementsIntoAdvTrackerBox();
        packElementsIntoTrackingMethodBox();
        packElementsIntoCalibrationBox();
        setRequisitions();

        mainNotebook->AppendPage(mainGUIBox, sfg::Label::Create("KinectToVR"));
        mainNotebook->AppendPage(advancedTrackerBox, sfg::Label::Create("Adv. Trackers"));
        mainNotebook->AppendPage(calibrationBox, sfg::Label::Create("Calibration"));
        mainNotebook->AppendPage(trackingMethodBox, sfg::Label::Create("Tracking Method"));
        
        

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
    guiWindow->SetAllocation(sf::FloatRect(0.f, 0.f, .4f*SFMLsettings::m_window_width, .4f*SFMLsettings::m_window_height));
    guiWindow->SetRequisition(sf::Vector2f(.2f*SFMLsettings::m_window_width, .2f*SFMLsettings::m_window_height));
    //Text scaling
    /*
    Window > * > * > Label{
        FontSize : 18;
    /*FontName: data/linden_hill.otf;*/
    /*
    float defaultFontSize = 10.f / 1920.f; // Percentage relative to 1080p
    float scaledFontSize = defaultFontSize * (SFMLsettings::m_window_width / SFMLsettings::windowScale);
    */
    float scaledFontSize = SFMLsettings::globalFontSize;
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

    {
        // Font Size Scaling
        FontSizeScale->SetDigits(3);
        FontSizeScale->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this] {
            // This checking is required due to some weird anomaly in Sfgui.
            // Without it, it will constantly reupdate the SpinButton event,
            // effectively lagging this for 2-10x as long as it should
            lastFontSizeValue = SFMLsettings::globalFontSize;
            SFMLsettings::globalFontSize = FontSizeScale->GetValue();
            if (lastFontSizeValue != SFMLsettings::globalFontSize) {
                setScale();
            }
        });
    }
    EnableGamepadButton->GetSignal(sfg::ToggleButton::OnToggle).Connect([this] {
        if (EnableGamepadButton->IsActive()) {
            SFMLsettings::usingGamepad = true;
        }
        else {
            SFMLsettings::usingGamepad = false;
        }
    });

	ShowSkeletonButton->GetSignal(sfg::Widget::OnLeftClick).Connect([] {
		KinectSettings::isSkeletonDrawn = !KinectSettings::isSkeletonDrawn;
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
    
    showJointDevicesButton->GetSignal(sfg::ToggleButton::OnToggle).Connect([this] {
        kinectJointDevicesHiddenFromList = !showJointDevicesButton->IsActive();
        updateDeviceLists();
    });
    refreshDeviceListButton->GetSignal(sfg::Widget::OnLeftClick).Connect([this] {
        updateDeviceLists();
    });
    identifyPosDeviceButton->GetSignal(sfg::ToggleButton::OnToggle).Connect([this] {
        if (PositionDeviceList->GetItemCount()) {
            auto index = selectedPositionDeviceIndex();
            auto d = TrackingPoolManager::getDeviceData(index);
            d.parentHandler->identify(index, identifyPosDeviceButton->IsActive());
        }
    });
    identifyRotDeviceButton->GetSignal(sfg::ToggleButton::OnToggle).Connect([this] {
        if (RotationDeviceList->GetItemCount()) {
            auto index = selectedRotationDeviceIndex();
            auto d = TrackingPoolManager::getDeviceData(index);
            d.parentHandler->identify(index, identifyRotDeviceButton->IsActive());
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
    setCalibrationSignal();

    StartPSMoveHandler->GetSignal(sfg::Widget::OnLeftClick).Connect([this] {
        if (psMoveHandler.active)
            return;

        auto errorCode = psMoveHandler.initialise();

        if (psMoveHandler.active) {
            static bool addedToVector = false;
            if (!addedToVector) {
                v_deviceHandlersRef->push_back(std::make_unique<PSMoveHandler>(psMoveHandler));
                addedToVector = true;
            }
            updateDeviceLists();
            PSMoveHandlerLabel->SetText("Status: Connected!");
        }
        else {
            PSMoveHandlerLabel->SetText(psMoveHandler.connectionMessages[errorCode]);
        }
    });
    StopPSMoveHandler->GetSignal(sfg::Widget::OnLeftClick).Connect([this] {
        if (!psMoveHandler.active)
            return;
        psMoveHandler.shutdown();
        updateDeviceLists();
        PSMoveHandlerLabel->SetText("Status: Disconnected!");
    });
}
void setColorTrackerSignals(ColorTracker & colorTracker) {
    InitiateColorTrackingButton->GetSignal(sfg::Button::OnMouseLeftPress).Connect([this, &colorTracker] {
        colorTracker.initialise();
    });
    DestroyColorTrackingButton->GetSignal(sfg::Button::OnMouseLeftPress).Connect([this, &colorTracker] {
        colorTracker.terminate();
    });
}
int selectedPositionDeviceIndex() {
    
    int posIndex = PositionDeviceList->GetSelectedItem();
    if (kinectJointDevicesHiddenFromList && trackerIdInKinectRange(posIndex))
        posIndex += KVR::KinectJointCount;
    // Really need to find a less hacky way to do this - as without it, when the kinect joints are hidden,
    // selecting a PSMove (ID of 25) would still use the kinect joint because it's technically the 0th item in the list
    return posIndex;
}
int selectedRotationDeviceIndex() {
    int rotIndex = RotationDeviceList->GetSelectedItem();
    if (kinectJointDevicesHiddenFromList && trackerIdInKinectRange(rotIndex))
        rotIndex += KVR::KinectJointCount;
    // Really need to find a less hacky way to do this - as without it, when the kinect joints are hidden,
    // selecting a PSMove (ID of 25) would still use the kinect joint because it's technically the 0th item in the list
    return rotIndex;
}
void addUserTrackerToList() {
    TempTracker temp;
    temp.GUID = TrackersToBeInitialised.size();
    temp.isController = IsControllerButton->IsActive();

    int posIndex = selectedPositionDeviceIndex();
    KVR::TrackedDeviceInputData posData = TrackingPoolManager::getDeviceData(posIndex);

    temp.positionGlobalDeviceId = posIndex;
    temp.positionTrackingOption = posData.positionTrackingOption;

    int rotIndex = selectedRotationDeviceIndex();
    KVR::TrackedDeviceInputData rotData = TrackingPoolManager::getDeviceData(rotIndex);

    temp.rotationGlobalDeviceId = rotIndex;
    temp.rotationTrackingOption = rotData.rotationTrackingOption;

    //temp.joint0 = KVR::KinectJointType(BonesList->GetSelectedItem());
	//temp.joint1 = temp.joint0;	//TEMP BEFORE SELECTION IMPLEMENTED
    temp.role = KVR::KinectDeviceRole(RolesList->GetSelectedItem());
    updateTrackerLists(temp);
}
void addTrackerToList(KVR::KinectJointType joint, KVR::KinectDeviceRole role, bool isController) {
	TempTracker temp;
    temp.GUID = TrackersToBeInitialised.size();
    temp.isController = isController;
    //temp.joint0 = joint;
	//temp.joint1 = temp.joint0; //TEMP BEFORE SELECTION IMPLEMENTED
    temp.role = role;

    updateTrackerLists(temp);
}
void updateTrackerLists(TempTracker &temp) {
    // Display a radio button menu where selecting each button selects that tracker
    // Displays the joint of each tracker and (Tracker)/(Controller)
    std::stringstream roleStrStream;
    if (temp.isController)
        roleStrStream << " (Tracked Controller) ";
    else
        roleStrStream << " (Tracker) ";
    roleStrStream << "(Role: " << KVR::KinectDeviceRoleName[int(temp.role)] << ") ";
    std::string posName = TrackingPoolManager::deviceGuiString(temp.positionGlobalDeviceId);
    std::string rotName = TrackingPoolManager::deviceGuiString(temp.rotationGlobalDeviceId);
    std::string finalTrackerName = "Position: " + posName + " | Rotation: " + rotName + " | " + roleStrStream.str();

    LOG(INFO) << "Adding tracker to list :: " << finalTrackerName;

    temp.radioButton = sfg::RadioButton::Create(finalTrackerName);
    if (TrackersToBeInitialised.size()) {
        auto group = TrackersToBeInitialised.back().radioButton->GetGroup();
        temp.radioButton->SetGroup(group);
    }

    TrackerList->Pack(temp.radioButton);

    TrackersToBeInitialised.push_back(temp);
}
void refreshCalibrationMenuValues() {
    using namespace KinectSettings;
    CalibrationEntryPosX->SetValue(KinectSettings::kinectRepPosition.v[0]);
    CalibrationEntryPosY->SetValue(KinectSettings::kinectRepPosition.v[1]);
    CalibrationEntryPosZ->SetValue(KinectSettings::kinectRepPosition.v[2]);

    CalibrationEntryRotX->SetValue(KinectSettings::kinectRadRotation.v[0]);
    CalibrationEntryRotY->SetValue(KinectSettings::kinectRadRotation.v[1]);
    CalibrationEntryRotZ->SetValue(KinectSettings::kinectRadRotation.v[2]);

}
void setCalibrationSignal() {
    CalibrationEntryPosX->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this] {
        KinectSettings::kinectRepPosition.v[0] = CalibrationEntryPosX->GetValue();
        KinectSettings::sensorConfigChanged = true;
    }
    );
    CalibrationEntryPosY->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this] {
        KinectSettings::kinectRepPosition.v[1] = CalibrationEntryPosY->GetValue();
        KinectSettings::sensorConfigChanged = true;
    }
    );
    CalibrationEntryPosZ->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this] {
        KinectSettings::kinectRepPosition.v[2] = CalibrationEntryPosZ->GetValue();
        KinectSettings::sensorConfigChanged = true;
    }
    );

    CalibrationEntryRotX->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this] {
        KinectSettings::kinectRadRotation.v[0] = CalibrationEntryRotX->GetValue();
        KinectSettings::updateKinectQuaternion();
        KinectSettings::sensorConfigChanged = true;
    }
    );
    CalibrationEntryRotY->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this] {
        KinectSettings::kinectRadRotation.v[1] = CalibrationEntryRotY->GetValue();
        KinectSettings::updateKinectQuaternion();
        KinectSettings::sensorConfigChanged = true;
    }
    );
    CalibrationEntryRotZ->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this] {
        KinectSettings::kinectRadRotation.v[2] = CalibrationEntryRotZ->GetValue();
        KinectSettings::updateKinectQuaternion();
        KinectSettings::sensorConfigChanged = true;
    }
    );
    CalibrationSaveButton->GetSignal(sfg::Button::OnLeftClick).Connect([this] {
        KinectSettings::updateKinectQuaternion();
        KinectSettings::writeKinectSettings();
    }
    );
}
void loadK2VRIntoBindingsMenu(vr::IVRSystem * & m_VRSystem) {
    // Only scene apps currently actually load into the Bindings menu
    // So, this momentarily opens the vrsystem as a scene, and closes it
    // Which actually allows the menu to stay open, while still functioning as normal
    vr::EVRInitError eError = vr::VRInitError_None;
    vr::VR_Shutdown();
    m_VRSystem = vr::VR_Init(&eError, vr::VRApplication_Scene);
    Sleep(100); // Necessary because of SteamVR timing occasionally being too quick to change the scenes
    vr::VR_Shutdown();
    m_VRSystem = vr::VR_Init(&eError, vr::VRApplication_Background);
}
void setVRSceneChangeButtonSignal(vr::IVRSystem * & m_VRSystem) {
    ActivateVRSceneTypeButton->GetSignal(sfg::Widget::OnLeftClick).Connect([this, &m_VRSystem] {
        loadK2VRIntoBindingsMenu(m_VRSystem);
    });

}
void setKinectButtonSignal(KinectHandlerBase& kinect) {
    reconKinectButton->GetSignal(sfg::Widget::OnLeftClick).Connect([&kinect] {
        kinect.initialise();
    });
}
void spawnAndConnectTracker(vrinputemulator::VRInputEmulator & inputE, std::vector<KVR::KinectTrackedDevice>& v_trackers, TempTracker t_tracker)
{
    KVR::KinectTrackedDevice device(inputE, t_tracker.positionGlobalDeviceId, t_tracker.rotationGlobalDeviceId, t_tracker.role);
    device.positionTrackingOption = t_tracker.positionTrackingOption;
    device.rotationTrackingOption = t_tracker.rotationTrackingOption;
    device.customModelName = TrackingPoolManager::getDeviceData(t_tracker.positionGlobalDeviceId).customModelName;
    device.init(inputE);
    v_trackers.push_back(device);
}
void setTrackerButtonSignals(vrinputemulator::VRInputEmulator &inputE, std::vector<KVR::KinectTrackedDevice> &v_trackers) {
    TrackerInitButton->GetSignal(sfg::Widget::OnLeftClick).Connect([this, &v_trackers, &inputE] {
        TrackerInitButton->SetLabel("Trackers Initialised");
        if (TrackersToBeInitialised.empty()) {
            spawnDefaultLowerBodyTrackers(inputE, v_trackers);
            spawnAndConnectKinectTracker(inputE, v_trackers);
        }
        else {
            TrackingPoolManager::leftFootDevicePosGID = k_invalidTrackerID;
            TrackingPoolManager::rightFootDevicePosGID = k_invalidTrackerID;
            TrackingPoolManager::leftFootDeviceRotGID = k_invalidTrackerID;
            TrackingPoolManager::rightFootDeviceRotGID = k_invalidTrackerID;

            for (TempTracker tracker : TrackersToBeInitialised) {
                spawnAndConnectTracker(inputE, v_trackers, tracker);

                if (tracker.role == KVR::KinectDeviceRole::LeftFoot) {
                    TrackingPoolManager::leftFootDevicePosGID = tracker.positionGlobalDeviceId;
                    TrackingPoolManager::leftFootDeviceRotGID = tracker.rotationGlobalDeviceId;
                }
                if (tracker.role == KVR::KinectDeviceRole::RightFoot) {
                    TrackingPoolManager::rightFootDevicePosGID = tracker.positionGlobalDeviceId;
                    TrackingPoolManager::rightFootDeviceRotGID = tracker.rotationGlobalDeviceId;
                }

                if (tracker.isController) {
                    setDeviceProperty(inputE, v_trackers.back().deviceId, vr::Prop_DeviceClass_Int32, "int32", "2"); // Device Class: Controller
                    if (tracker.role == KVR::KinectDeviceRole::LeftHand) {
                        setDeviceProperty(inputE, v_trackers.back().deviceId, vr::Prop_ControllerRoleHint_Int32, "int32", "1"); // ControllerRole Left
                    }
                    else if (tracker.role == KVR::KinectDeviceRole::RightHand) {
                        setDeviceProperty(inputE, v_trackers.back().deviceId, vr::Prop_ControllerRoleHint_Int32, "int32", "2"); // ControllerRole Right
                    }
                }
            }
        }

        showPostTrackerInitUI();

        TrackerInitButton->SetState(sfg::Widget::State::INSENSITIVE);
    });

    SetJointsToFootRotationButton->GetSignal(sfg::Widget::OnLeftClick).Connect([this, &v_trackers] {
        using namespace KinectSettings;
        leftFootJointWithRotation = KVR::KinectJointType::FootLeft;
        rightFootJointWithRotation = KVR::KinectJointType::FootRight;
        leftFootJointWithoutRotation = KVR::KinectJointType::AnkleLeft;
        rightFootJointWithoutRotation = KVR::KinectJointType::AnkleRight;
        for (KVR::KinectTrackedDevice &d : v_trackers) {
            if (d.role == KVR::KinectDeviceRole::LeftFoot) {
                d.joint0 = leftFootJointWithRotation;
                d.joint1 = leftFootJointWithoutRotation;
            }
            if (d.role == KVR::KinectDeviceRole::RightFoot) {
                d.joint0 = rightFootJointWithRotation;
                d.joint1 = rightFootJointWithoutRotation;
            }
        }
    });
    SetJointsToAnkleRotationButton->GetSignal(sfg::Widget::OnLeftClick).Connect([this, &v_trackers] {
        using namespace KinectSettings;
        leftFootJointWithRotation = KVR::KinectJointType::AnkleLeft;
        rightFootJointWithRotation = KVR::KinectJointType::AnkleRight;
        leftFootJointWithoutRotation = KVR::KinectJointType::FootLeft;
        rightFootJointWithoutRotation = KVR::KinectJointType::FootRight;
        for (KVR::KinectTrackedDevice &d : v_trackers) {
            if (d.role == KVR::KinectDeviceRole::LeftFoot) {
                d.joint0 = leftFootJointWithRotation;
                d.joint1 = leftFootJointWithoutRotation;
            }
            if (d.role == KVR::KinectDeviceRole::RightFoot) {
                d.joint0 = rightFootJointWithRotation;
                d.joint1 = rightFootJointWithoutRotation;
            }
        }
    });

    SetAllJointsRotUnfiltered->GetSignal(sfg::Widget::OnLeftClick).Connect([this, &v_trackers] {
        for (KVR::KinectTrackedDevice &d : v_trackers) {
            if (d.isSensor()){}
            else {
                d.rotationFilterOption = KVR::JointRotationFilterOption::Unfiltered;
            }
        }
    });
    SetAllJointsRotFiltered->GetSignal(sfg::Widget::OnLeftClick).Connect([this, &v_trackers] {
        for (KVR::KinectTrackedDevice &d : v_trackers) {
            if (d.isSensor()) {}
            else {
                d.rotationFilterOption = KVR::JointRotationFilterOption::Filtered;
            }
        }
    });
    SetAllJointsRotHead->GetSignal(sfg::Widget::OnLeftClick).Connect([this, &v_trackers] {
        for (KVR::KinectTrackedDevice &d : v_trackers) {
            if (d.isSensor()) {}
            else {
                d.rotationFilterOption = KVR::JointRotationFilterOption::HeadLook;
            }
        }
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

    auto fontSizeBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);
    fontSizeBox->Pack(FontSizeScaleLabel);
    fontSizeBox->Pack(FontSizeScale);
    mainGUIBox->Pack(fontSizeBox);

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
    
}

void setHipScaleBox() {
    auto HipLabel = sfg::Label::Create("Vertical Hip Adjustment (metres)");
    HipScale->SetDigits(3);
    
    HipScaleBox->Pack(HipLabel, false, false);
    HipScaleBox->Pack(HipScale);
    mainGUIBox->Pack(HipScaleBox);
}
void packElementsIntoTrackingMethodBox() {
    //trackingMethodBox->Pack(InitiateColorTrackingButton);
    //trackingMethodBox->Pack(DestroyColorTrackingButton);

    trackingMethodBox->Pack(TrackingMethodLabel);

    sfg::Box::Ptr horizontalPSMBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL, 5.f);
    horizontalPSMBox->Pack(StartPSMoveHandler);
    horizontalPSMBox->Pack(StopPSMoveHandler);
    horizontalPSMBox->Pack(PSMoveHandlerLabel);

    trackingMethodBox->Pack(horizontalPSMBox);
}

void updateDeviceLists() {
    setDeviceListItems(PositionDeviceList);
    setDeviceListItems(RotationDeviceList);
}

void packElementsIntoAdvTrackerBox() {
    advancedTrackerBox->Pack(AddHandControllersToList);
    advancedTrackerBox->Pack(AddLowerTrackersToList);

    auto jointBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);
    jointBox->Pack(SetJointsToFootRotationButton);
    jointBox->Pack(SetJointsToAnkleRotationButton);
    advancedTrackerBox->Pack(jointBox);

    advancedTrackerBox->Pack(SetAllJointsRotUnfiltered);
    advancedTrackerBox->Pack(SetAllJointsRotFiltered);
    advancedTrackerBox->Pack(SetAllJointsRotHead);

    advancedTrackerBox->Pack(TrackerList);
    advancedTrackerBox->Pack(showJointDevicesButton);

    TrackerList->Pack(TrackerListLabel);

    setBonesListItems();
    updateDeviceLists();
    setRolesListItems(RolesList);

    TrackerListOptionsBox->Pack(refreshDeviceListButton);
    TrackerListOptionsBox->Pack(BonesList);
    TrackerListOptionsBox->Pack(PositionDeviceList);
    TrackerListOptionsBox->Pack(identifyPosDeviceButton);
    TrackerListOptionsBox->Pack(RotationDeviceList);
    TrackerListOptionsBox->Pack(identifyRotDeviceButton);
    TrackerListOptionsBox->Pack(RolesList);
    TrackerListOptionsBox->Pack(IsControllerButton);
    TrackerListOptionsBox->Pack(AddTrackerToListButton);
    TrackerListOptionsBox->Pack(RemoveTrackerFromListButton);


    advancedTrackerBox->Pack(TrackerListOptionsBox);
}
void packElementsIntoCalibrationBox() {
    sfg::Box::Ptr verticalBox = sfg::Box::Create(sfg::Box::Orientation::VERTICAL);

    verticalBox->Pack(CalibrationSettingsLabel);

    auto horizontalPosBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);
    horizontalPosBox->Pack(CalibrationPosLabel);
    CalibrationEntryPosX->SetDigits(4);
    horizontalPosBox->Pack(CalibrationEntryPosX);
    CalibrationEntryPosY->SetDigits(4);
    horizontalPosBox->Pack(CalibrationEntryPosY);
    CalibrationEntryPosZ->SetDigits(4);
    horizontalPosBox->Pack(CalibrationEntryPosZ);
    verticalBox->Pack(horizontalPosBox);

    auto horizontalRotBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);
    horizontalRotBox->Pack(CalibrationRotLabel);
    CalibrationEntryRotX->SetDigits(4);
    horizontalRotBox->Pack(CalibrationEntryRotX);
    CalibrationEntryRotY->SetDigits(4);
    horizontalRotBox->Pack(CalibrationEntryRotY);
    CalibrationEntryRotZ->SetDigits(4);
    horizontalRotBox->Pack(CalibrationEntryRotZ);
    verticalBox->Pack(horizontalRotBox);
    verticalBox->Pack(CalibrationSaveButton);

    verticalBox->Pack(ActivateVRSceneTypeButton);

    calibrationBox->Pack(verticalBox);
}
void setBonesListItems() {
    using namespace KVR;
    for (int i = 0; i < KinectJointCount; ++i) {
        BonesList->AppendItem(KinectJointName[i]);
    }
    // Set as default - to prevent garbage additions 
    BonesList->SelectItem(0);
}
bool trackerIdInKinectRange(uint32_t trackerId) {
    static bool kinectIdLocated = false;
    static uint32_t kinectFirstId = k_invalidTrackerID;
    static uint32_t kinectLastId = k_invalidTrackerID;
    if (!kinectIdLocated) {
        for (int i = 0; i < TrackingPoolManager::count(); ++i) {
            auto data = TrackingPoolManager::getDeviceData(i);
            if (data.positionTrackingOption == KVR::JointPositionTrackingOption::Skeleton) {
                kinectFirstId = i;
                // Kinect Trackers spawned all together, so no need to account for different devices's between this range
                kinectLastId = i + KVR::KinectJointCount - 1;

                kinectIdLocated = true;
                break;
            }
        }

        // Outside for loop, in case it couldn't find any kinect joints, so any calls to this will always return false because the invalid ID is so high
        kinectIdLocated = true;
    }
    return trackerId >= kinectFirstId && trackerId <= kinectLastId;
}
void setDeviceListItems(sfg::ComboBox::Ptr comboBox) {
    comboBox->Clear();
    for (int i = 0; i < TrackingPoolManager::count(); ++i) {
        if (kinectJointDevicesHiddenFromList && trackerIdInKinectRange(i)) {
            continue;
        }
        comboBox->AppendItem(TrackingPoolManager::deviceGuiString(i));
    }
    // Set as default - to prevent garbage additions 
    comboBox->SelectItem(0);
}

void setRolesListItems(sfg::ComboBox::Ptr comboBox) {
    for (int i = 0; i < (int)KVR::KinectDeviceRole::Count; ++i) {
		comboBox->AppendItem(KVR::KinectDeviceRoleName[i]);
    }
    // Set as default - to prevent garbage additions 
	comboBox->SelectItem(0);
}

void updateKinectStatusLabel(KinectHandlerBase& kinect) {
    HRESULT status = kinect.getStatusResult();
    if (kinect.isInitialised()) {
        if (status == lastKinectStatus)
            return; // No need to waste time updating it;
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
    if (status != lastKinectStatus) {
        LOG(INFO) << "Kinect Status changed to: " << KinectStatusLabel->GetText().toAnsiString();
        lastKinectStatus = status;
    }
}


void updateEmuStatusLabelError(vrinputemulator::vrinputemulator_connectionerror e) {
    InputEmulatorStatusLabel->SetText("Input Emu Status: NOT Connected! Error " + std::to_string(e.errorcode) + " " + e.what() + "\n\n Is SteamVR open and InputEmulator installed?");
}
void updateEmuStatusLabelSuccess() {
    InputEmulatorStatusLabel->SetText("Input Emu Status: Success!");
}

void updateVRStatusLabel(vr::EVRInitError eError) {
    if (eError == vr::VRInitError_None)
        SteamVRStatusLabel->SetText("SteamVR Status: Success!");
    else
        SteamVRStatusLabel->SetText("SteamVR Status: ERROR " + std::to_string(eError) + "\nPlease restart K2VR with SteamVR successfully running!");
}

void setTrackingMethodsReference(std::vector<std::unique_ptr<TrackingMethod>> & ref) {
    v_trackingMethodsRef = &ref;
}
void setDeviceHandlersReference(std::vector<std::unique_ptr<DeviceHandler>> & ref) {
    v_deviceHandlersRef = &ref;
}

void updateWithNewWindowSize(sf::Vector2f size) {
    guiWindow->SetAllocation(sf::FloatRect(0.f, 0.f, .4f * size.x, .4f * size.y));
    //setScale();
    //guiWindow->SetAllocation(sf::FloatRect(size.x - width, 0.f, width, size.y));
    //mGUI.SideBar->SetAllocation(sf::FloatRect(0.f, 0.f, width, size.y));
}
private:
    sf::Font mainGUIFont;
    sfg::SFGUI sfguiRef;
    sfg::Window::Ptr guiWindow = sfg::Window::Create();
    sfg::Notebook::Ptr mainNotebook = sfg::Notebook::Create();


    std::vector<std::unique_ptr<DeviceHandler>> * v_deviceHandlersRef;
    std::vector<std::unique_ptr<TrackingMethod>> * v_trackingMethodsRef;

    // All the device handlers
    PSMoveHandler psMoveHandler;

    HRESULT lastKinectStatus = E_FAIL;

    sfg::Desktop guiDesktop;

    sfg::Box::Ptr mainGUIBox = sfg::Box::Create(sfg::Box::Orientation::VERTICAL, 5.f);
    sfg::Box::Ptr calibrationBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL, 5.f);
    sfg::Box::Ptr advancedTrackerBox = sfg::Box::Create(sfg::Box::Orientation::VERTICAL, 5.f);
    sfg::Box::Ptr trackingMethodBox = sfg::Box::Create(sfg::Box::Orientation::VERTICAL, 5.f);
    
    sfg::Adjustment::Ptr fontSizeAdjustment = sfg::Adjustment::Create();
    sfg::Label::Ptr FontSizeScaleLabel = sfg::Label::Create("(WARNING, LAGS ON CHANGE) Font Size: ");
    sfg::SpinButton::Ptr FontSizeScale = sfg::SpinButton::Create(sfg::Adjustment::Create(SFMLsettings::globalFontSize, 5.f, 100.f, .5f));
    float lastFontSizeValue = SFMLsettings::globalFontSize;


    //Statuses
    sfg::Label::Ptr KinectStatusLabel = sfg::Label::Create();
    sfg::Label::Ptr SteamVRStatusLabel = sfg::Label::Create();
    sfg::Label::Ptr InputEmulatorStatusLabel = sfg::Label::Create();

    sfg::Button::Ptr reconKinectButton = sfg::Button::Create("Reconnect Kinect");
    sfg::Button::Ptr TrackerInitButton = sfg::Button::Create("**Please be in VR before hitting me!** Initialise SteamVR Kinect Trackers - HIT ME");

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


    sfg::Label::Ptr InferredLabel = sfg::Label::Create("Checking this stops the trackers if it's not absolutely 100% sure where they are. Leaving this disabled may cause better tracking in poorly lit environments, but at the cost of slight jerks aside sometimes.");
    sfg::CheckButton::Ptr IgnoreInferredCheckButton = sfg::CheckButton::Create("Disable Raw Positional Tracking");

    sfg::Button::Ptr SetJointsToFootRotationButton = sfg::Button::Create("Enable (buggy) foot rotation for 360 Kinect");
    sfg::Button::Ptr SetJointsToAnkleRotationButton = sfg::Button::Create("Disable (buggy) foot rotation for 360 Kinect");

    sfg::Button::Ptr SetAllJointsRotUnfiltered = sfg::Button::Create("Disable rotation smoothing for ALL joints (Rotation smoothing is in development!!!)");
    sfg::Button::Ptr SetAllJointsRotFiltered = sfg::Button::Create("Enable rotation smoothing for ALL joints (Rotation smoothing is in development!!!)");
    sfg::Button::Ptr SetAllJointsRotHead = sfg::Button::Create("Use Head orientation for ALL joints - may fix issues with jumping trackers at cost of limited rotation");

    sfg::Label::Ptr InstructionsLabel = sfg::Label::Create("Stand in front of the Kinect sensor.\n If the trackers don't update, then try crouching slightly until they move.\n\n Calibration: The arrow represents the position and rotation of the Kinect - match it as closely to real life as possible for the trackers to line up.\n\n The arrow pos/rot is set with the thumbsticks on the controllers, and confirmed with the trigger.");    //Blegh - There has to be a better way than this, maybe serialization?

    sfg::Label::Ptr CalibrationSettingsLabel = sfg::Label::Create("These settings are here for manual entry, and saving until a proper configuration system is implemented.\nYou can use this to quickly calibrate if your Kinect is in the same place. \n(Rotation is in radians, and Pos should be in meters roughly)");
    sfg::Label::Ptr CalibrationPosLabel = sfg::Label::Create("Position x, y, z");
    sfg::SpinButton::Ptr CalibrationEntryPosX = sfg::SpinButton::Create(sfg::Adjustment::Create(KinectSettings::kinectRepPosition.v[0], -10.f, 10.f, .01f, .2f));
    sfg::SpinButton::Ptr CalibrationEntryPosY = sfg::SpinButton::Create(sfg::Adjustment::Create(KinectSettings::kinectRepPosition.v[1], -10.f, 10.f, .01f, .2f));
    sfg::SpinButton::Ptr CalibrationEntryPosZ = sfg::SpinButton::Create(sfg::Adjustment::Create(KinectSettings::kinectRepPosition.v[2], -10.f, 10.f, .01f, .2f));

    sfg::Label::Ptr CalibrationRotLabel = sfg::Label::Create("Rotation x, y, z");
    sfg::SpinButton::Ptr CalibrationEntryRotX = sfg::SpinButton::Create(sfg::Adjustment::Create(KinectSettings::kinectRadRotation.v[0], -10.f, 10.f, .01f, .2f));
    sfg::SpinButton::Ptr CalibrationEntryRotY = sfg::SpinButton::Create(sfg::Adjustment::Create(KinectSettings::kinectRadRotation.v[1], -10.f, 10.f, .01f, .2f));
    sfg::SpinButton::Ptr CalibrationEntryRotZ = sfg::SpinButton::Create(sfg::Adjustment::Create(KinectSettings::kinectRadRotation.v[2], -10.f, 10.f, .01f, .2f));

    sfg::Button::Ptr CalibrationSaveButton = sfg::Button::Create("Save Calibration Values");

    sfg::Button::Ptr ActivateVRSceneTypeButton = sfg::Button::Create("Show K2VR in the VR Bindings Menu!");

    //Adv Trackers
    sfg::Button::Ptr AddHandControllersToList = sfg::Button::Create("Add Hand Controllers");
    sfg::Button::Ptr AddLowerTrackersToList = sfg::Button::Create("Add Lower Body Trackers");

    sfg::Box::Ptr TrackerList = sfg::Box::Create(sfg::Box::Orientation::VERTICAL, 5);
    sfg::Label::Ptr TrackerListLabel = sfg::Label::Create("Trackers to be spawned:");

    sfg::Box::Ptr TrackerListOptionsBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL, 5);
    sfg::SpinButton::Ptr HipScale = sfg::SpinButton::Create(sfg::Adjustment::Create(KinectSettings::hipRoleHeightAdjust, -1.f, 1.f, .01f));
    sfg::Box::Ptr HipScaleBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);

    bool kinectJointDevicesHiddenFromList = true;
    sfg::CheckButton::Ptr showJointDevicesButton = sfg::CheckButton::Create("Show joints in devices");
    sfg::Button::Ptr refreshDeviceListButton = sfg::Button::Create("Refresh Devices");
    sfg::ComboBox::Ptr BonesList = sfg::ComboBox::Create();
    sfg::ComboBox::Ptr PositionDeviceList = sfg::ComboBox::Create();
    sfg::CheckButton::Ptr identifyPosDeviceButton = sfg::CheckButton::Create("Boop");
    sfg::ComboBox::Ptr RotationDeviceList = sfg::ComboBox::Create();
    sfg::CheckButton::Ptr identifyRotDeviceButton = sfg::CheckButton::Create("Beep");
    sfg::ComboBox::Ptr RolesList = sfg::ComboBox::Create();
    sfg::CheckButton::Ptr IsControllerButton = sfg::CheckButton::Create("Controller");
    sfg::Button::Ptr AddTrackerToListButton = sfg::Button::Create("Add");
    sfg::Button::Ptr RemoveTrackerFromListButton = sfg::Button::Create("Remove");

    std::vector<TempTracker> TrackersToBeInitialised;


    //Tracking Method Box
    sfg::Button::Ptr InitiateColorTrackingButton = sfg::Button::Create("Start Color Tracker");
    sfg::Button::Ptr DestroyColorTrackingButton = sfg::Button::Create("Destroy Color Tracker");
    sfg::Label::Ptr TrackingMethodLabel = sfg::Label::Create("Click the corresponding button for the devices you wish to use, and K2VR will try its best to connect to them. (Go to the 'Adv. Trackers' tab once these are connected.");

    sfg::Button::Ptr StartPSMoveHandler = sfg::Button::Create("Run PS Move Handler");
    sfg::Button::Ptr StopPSMoveHandler = sfg::Button::Create("Stop PS Move Handler");
    sfg::Label::Ptr PSMoveHandlerLabel = sfg::Label::Create("Status: Off");

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
        SetAllJointsRotUnfiltered->Show(show);
        HipScale->Show(show);
        HipScaleBox->Show(show);
        SetAllJointsRotHead->Show(show);
        SetAllJointsRotFiltered->Show(show);
        SetJointsToAnkleRotationButton->Show(show);
        SetJointsToFootRotationButton->Show(show);

        //calibrationBox->Show(show);
    }
    void hidePostTrackerInitUI() {
        showPostTrackerInitUI(false);
    }
};