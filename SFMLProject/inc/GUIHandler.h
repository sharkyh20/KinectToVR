#pragma once
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include "VRController.h"
#include <atlbase.h>
#include "KinectSettings.h"
#include "KinectHandlerBase.h"
#include "KinectTrackedDevice.h"
#include "KinectJoint.h"
#include "ColorTracker.h"
#include "VRHelper.h"
#include <TlHelp32.h>
#include "TrackingMethod.h"
#include "DeviceHandler.h"
#include "PSMoveHandler.h"
#include "VRDeviceHandler.h"
#include <ShellAPI.h>

#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <Eigen/Geometry>
#include <codecvt>
#include <string>

#include <SFML/Graphics.hpp>
#include <SFML/OpenGL.hpp>
//GUI
#include <SFGUI/SFGUI.hpp>
#include <SFGUI/Widgets.hpp>
#include <string>

#include <cereal/archives/json.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/access.hpp>

struct TempTrackerData
{
	TempTrackerData()
	{
	}

	uint32_t positionGlobalDeviceId = 0;
	std::string posDeviceName = "INVALID";
	std::string posDeviceSerial = "INVALID";

	uint32_t rotationGlobalDeviceId = 0;
	std::string rotDeviceName = "INVALID";
	std::string rotDeviceSerial = "INVALID";

	KVR::KinectDeviceRole role = KVR::KinectDeviceRole::Unassigned;
	bool isController = false;

	friend class cereal::access;

	template <class Archive>
	void serialize(Archive& archive)
	{
		archive(
			CEREAL_NVP(positionGlobalDeviceId),
			CEREAL_NVP(posDeviceName),
			CEREAL_NVP(posDeviceSerial),
			CEREAL_NVP(positionGlobalDeviceId),
			CEREAL_NVP(rotDeviceName),
			CEREAL_NVP(rotDeviceSerial),
			CEREAL_NVP(role),
			CEREAL_NVP(isController)
		);
	}
};

struct TempTracker
{
	sfg::RadioButton::Ptr radioButton = sfg::RadioButton::Create("");
	int GUID = 404;

	KVR::JointPositionTrackingOption positionTrackingOption = KVR::JointPositionTrackingOption::Skeleton;

	KVR::JointRotationTrackingOption rotationTrackingOption = KVR::JointRotationTrackingOption::Skeleton;

	TempTrackerData data;
};

struct serialTest
{
	int butt;

	template <class Archive>
	void serialize(Archive& archive,
	               serialTest& t)
	{
		archive(butt);
	}
};

class GUIHandler
{
	
public:
	sfg::ComboBox::Ptr coptbox = sfg::ComboBox::Create();
	sfg::ComboBox::Ptr coptbox1 = sfg::ComboBox::Create();
	sfg::ComboBox::Ptr foptbox = sfg::ComboBox::Create();

	sfg::ComboBox::Ptr psmovebox = sfg::ComboBox::Create();
	sfg::ComboBox::Ptr psmovebox1 = sfg::ComboBox::Create();

	sfg::ComboBox::Ptr contrackingselectbox = sfg::ComboBox::Create();
	sfg::ComboBox::Ptr bodytrackingselectbox = sfg::ComboBox::Create();
	sfg::ComboBox::Ptr headtrackingselectbox = sfg::ComboBox::Create();

	sfg::ComboBox::Ptr psmoveboxa = sfg::ComboBox::Create();

	sfg::ComboBox::Ptr psmoveboxhi = sfg::ComboBox::Create();
	sfg::ComboBox::Ptr psmoveboxmi = sfg::ComboBox::Create();
	sfg::ComboBox::Ptr psmoveboxyo = sfg::ComboBox::Create();
	sfg::Box::Ptr psmidbox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL, 5.f);
	sfg::Box::Ptr psmidbox1 = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL, 5.f);
	sfg::Button::Ptr TrackerInitButton = sfg::Button::Create("Spawn Trackers");
	sfg::Label::Ptr DriverStatusLabel = sfg::Label::Create("Driver Status: UNKNOWN (Code: -1)");

	GUIHandler()
	{
		guiWindow->SetTitle("KinectToVR EX 0.8.0");

		setDefaultSignals();

		setLineWrapping();
		packElementsIntoMainBox();
		packElementsIntoAdvTrackerBox();
		packElementsIntoTrackingMethodBox();
		packElementsIntoCalibrationBox();
		packElementsIntoVirtualHipsBox();
		setRequisitions();

		mainNotebook->AppendPage(mainGUIBox, sfg::Label::Create(" Body Trackers "));
		mainNotebook->AppendPage(calibrationBox, sfg::Label::Create(" Offsets "));
		mainNotebook->AppendPage(advancedTrackerBox, sfg::Label::Create(" Tracking Options "));
		//mainNotebook->AppendPage(controllersBox, sfg::Label::Create(" Controllers "));
		//mainNotebook->AppendPage(virtualHipsBox, sfg::Label::Create(" Head Tracking "));

		guiWindow->Add(mainNotebook);
		guiDesktop.Add(guiWindow);

		setScale();

		bool b = guiDesktop.LoadThemeFromFile("main_theme.theme");

		guiDesktop.Update(0.f);
	}

	~GUIHandler()
	{
	}

	void display(sf::RenderWindow& window)
	{
		sfguiRef.Display(window);
	}

	void desktopHandleEvents(sf::Event event)
	{
		guiDesktop.HandleEvent(event);
	}

	void updateDesktop(float d)
	{
		guiDesktop.Update(d);
	}

	void setRequisitions()
	{
		CalibrationEntryrPosX->SetRequisition(sf::Vector2f(40.f, 0.f));
		CalibrationEntryrPosY->SetRequisition(sf::Vector2f(40.f, 0.f));
		CalibrationEntryrPosZ->SetRequisition(sf::Vector2f(40.f, 0.f));
		CalibrationEntryrRotX->SetRequisition(sf::Vector2f(40.f, 0.f));
		CalibrationEntryrRotY->SetRequisition(sf::Vector2f(40.f, 0.f));
		CalibrationEntryrRotZ->SetRequisition(sf::Vector2f(40.f, 0.f));

		CalibrationEntrylPosX->SetRequisition(sf::Vector2f(40.f, 0.f));
		CalibrationEntrylPosY->SetRequisition(sf::Vector2f(40.f, 0.f));
		CalibrationEntrylPosZ->SetRequisition(sf::Vector2f(40.f, 0.f));
		CalibrationEntrylRotX->SetRequisition(sf::Vector2f(40.f, 0.f));
		CalibrationEntrylRotY->SetRequisition(sf::Vector2f(40.f, 0.f));
		CalibrationEntrylRotZ->SetRequisition(sf::Vector2f(40.f, 0.f));

		CalibrationEntryhPosX->SetRequisition(sf::Vector2f(40.f, 0.f));
		CalibrationEntryhPosY->SetRequisition(sf::Vector2f(40.f, 0.f));
		CalibrationEntryhPosZ->SetRequisition(sf::Vector2f(40.f, 0.f));
		CalibrationEntryhRotX->SetRequisition(sf::Vector2f(40.f, 0.f));
		CalibrationEntryhRotY->SetRequisition(sf::Vector2f(40.f, 0.f));
		CalibrationEntryhRotZ->SetRequisition(sf::Vector2f(40.f, 0.f));
	}

	void setScale()
	{
		guiWindow->SetAllocation(sf::FloatRect(0.f, 0.f, .8f * SFMLsettings::m_window_width,
		                                       .7f * SFMLsettings::m_window_height));
		guiWindow->SetRequisition(sf::Vector2f(.2f * SFMLsettings::m_window_width,
		                                       .2f * SFMLsettings::m_window_height));
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
		guiDesktop.SetProperty(
			"Window Label, Box, Button, Notebook, CheckButton, ToggleButton, Label, RadioButton, ComboBox, SpinButton",
			"FontSize", scaledFontSize);
	}

	void toggleRotButton()
	{
		KinectRotButton->SetActive(KinectSettings::adjustingKinectRepresentationRot);
	}

	void togglePosButton()
	{
		KinectPosButton->SetActive(KinectSettings::adjustingKinectRepresentationPos);
	}

	bool trackerConfigExists()
	{
		// NOTE: Does not necessarily mean that it is valid
		/*std::ifstream is(KVR::fileToDirPath(KVR::trackerConfig));
		return !is.fail();*/
	}

	void saveLastSpawnedTrackers(std::vector<TempTracker> v_trackers)
	{
		//std::vector<TempTrackerData> v_trackerData;
		//for (TempTracker& t : v_trackers) {
		//    v_trackerData.push_back(t.data);
		//}
		//std::ofstream os(KVR::fileToDirPath(KVR::trackerConfig));
		//if (os.fail()) {
		//    //FAIL!!!
		//    LOG(ERROR) << "ERROR: COULD NOT WRITE TO TRACKER CONFIG FILE\n";
		//}
		//else {
		//    cereal::JSONOutputArchive archive(os);
		//    LOG(INFO) << "Attempted to save last tracker settings to file";
		//    try {
		//        archive(
		//            CEREAL_NVP(v_trackerData)
		//        );
		//    }
		//    catch (cereal::RapidJSONException e) {
		//        LOG(ERROR) << "CONFIG FILE SAVE JSON ERROR: " << e.what();
		//    }

		//}
	}

	bool retrieveLastSpawnedTrackers()
	{
		//std::ifstream is(KVR::fileToDirPath(KVR::trackerConfig));

		//LOG(INFO) << "Attempted to load last set of spawned trackers at " << KVR::fileToDirPath(KVR::trackerConfig);

		//std::vector<TempTrackerData> v_trackerData;
		////CHECK IF VALID
		//if (is.fail()) {
		//    error_trackerCfgNotFound(KVR::trackerConfig);
		//    return false;
		//}
		//else {
		//    LOG(INFO) << KVR::trackerConfig << " load attempted!";
		//    try {
		//        cereal::JSONInputArchive archive(is);
		//        archive(CEREAL_NVP(v_trackerData));
		//    }
		//    catch (cereal::Exception e) {
		//        LOG(ERROR) << KVR::trackerConfig << "TRACKER FILE LOAD JSON ERROR: " << e.what();
		//    }
		//}
		//if (v_trackerData.size() == 0) {
		//    error_trackerCfgEmpty(KVR::trackerConfig);
		//    return false;
		//}
		//for (TempTrackerData& data : v_trackerData) {
		//    if (!addUserTrackerToList(data)) {
		//        error_lastTrackersRespawnFailure();
		//        return false;
		//    }
		//}
		return true;
	}

	void error_lastTrackersRespawnFailure()
	{
		LOG(ERROR) << "Attempted to respawn trackers, but at least one was invalid!";
		auto message = L"ERROR: INVALID TRACKERS DETECTED IN CFG: "
			+ SFMLsettings::fileDirectoryPath
			+ L"\n No trackers will be spawned"
			+ L"\n Refer to K2VR.log, to see what went wrong";
		auto result = MessageBox(nullptr, message.c_str(), L"ERROR!!!", MB_OK + MB_ICONWARNING);
	}

	void error_trackerCfgNotFound(std::wstring& trackerConfig)
	{
		//FAIL!!!!
		LOG(ERROR) << "ERROR: COULD NOT OPEN " << trackerConfig << " FILE";
		// Does not need to create config file - as the vector is empty anyway
		LOG(ERROR) << "ERROR: Can't find last used custom tracker file!";
		auto message = L"WARNING: NO lastTrackers.cfg DETECTED: "
			+ SFMLsettings::fileDirectoryPath
			+ L"\n No trackers will be added to the menu";
		auto result = MessageBox(nullptr, message.c_str(), L"WARNING!!!", MB_OK + MB_ICONWARNING);
	}

	void error_trackerCfgEmpty(std::wstring& trackerConfig)
	{
		LOG(ERROR) << "WARNING: " << trackerConfig << " FILE IS EMPTY OF TRACKERS";

		auto message = L"WARNING: No trackers were found in file at all! "
			+ SFMLsettings::fileDirectoryPath
			+ L"\n No trackers will be added to the menu";
		auto result = MessageBox(nullptr, message.c_str(), L"WARNING!!!", MB_OK + MB_ICONWARNING);
	}

	void connectPSMoveHandlerGUIEvents()
	{
		if (psMoveHandler.active)
		{
			PSMoveHandlerLabel->SetText("Status: Connected!");
		}
		else
		{
			PSMoveHandlerLabel->SetText("Status: Disconnected!");
		}

		StartPSMoveHandler->GetSignal(sfg::Widget::OnLeftClick).Connect([this]
		{
			initialisePSMoveHandlerIntoGUI();
		});
		StopPSMoveHandler->GetSignal(sfg::Widget::OnLeftClick).Connect([this]
		{
			if (!psMoveHandler.active)
				return;
			psMoveHandler.shutdown();
			updateDeviceLists();
			PSMoveHandlerLabel->SetText("Status: Disconnected!");
		});
	}

	void initialisePSMoveHandlerIntoGUI()
	{
		if (psMoveHandler.active)
		{
			LOG(INFO) << "Tried to initialise PSMoveHandler in the GUI, but it was already active";
			return;
		}

		auto errorCode = psMoveHandler.initialise();

		if (psMoveHandler.active)
		{
			static bool addedToVector = false;
			if (!addedToVector)
			{
				v_deviceHandlersRef->push_back(std::make_unique<PSMoveHandler>(psMoveHandler));
				addedToVector = true;
				PSMoveHandlerLabel->SetText("Status: Connected!");
			}
			updateDeviceLists();
		}
		else
		{
			PSMoveHandlerLabel->SetText(psMoveHandler.connectionMessages[errorCode]);
		}
	}

	void setDefaultSignals()
	{
		//Post VR Tracker Initialisation
		hidePostTrackerInitUI();

		{
			// Font Size Scaling
			FontSizeScale->SetDigits(3);
			FontSizeScale->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				// This checking is required due to some weird anomaly in Sfgui.
				// Without it, it will constantly reupdate the SpinButton event,
				// effectively lagging this for 2-10x as long as it should
				lastFontSizeValue = SFMLsettings::globalFontSize;
				SFMLsettings::globalFontSize = FontSizeScale->GetValue();
				if (lastFontSizeValue != SFMLsettings::globalFontSize)
				{
					setScale();
				}
			});
		}
		EnableGamepadButton->GetSignal(sfg::ToggleButton::OnToggle).Connect([this]
		{
			if (EnableGamepadButton->IsActive())
			{
				SFMLsettings::usingGamepad = true;
			}
			else
			{
				SFMLsettings::usingGamepad = false;
			}
		});

		ShowSkeletonButton->GetSignal(sfg::Widget::OnLeftClick).Connect([]
		{
			KinectSettings::isSkeletonDrawn = !KinectSettings::isSkeletonDrawn;
		});

		KinectRotButton->GetSignal(sfg::ToggleButton::OnToggle).Connect([this]
		{
			if (KinectRotButton->IsActive())
			{
				KinectSettings::adjustingKinectRepresentationRot = true;
			}
			else
				KinectSettings::adjustingKinectRepresentationRot = false;
		});
		KinectPosButton->GetSignal(sfg::ToggleButton::OnToggle).Connect([this]
		{
			if (KinectPosButton->IsActive())
			{
				KinectSettings::adjustingKinectRepresentationPos = true;
			}
			else
				KinectSettings::adjustingKinectRepresentationPos = false;
		});
		IgnoreInferredCheckButton->GetSignal(sfg::ToggleButton::OnToggle).Connect([this]
		{
			if (IgnoreInferredCheckButton->IsActive())
			{
				KinectSettings::ignoreInferredPositions = true;
				// No longer stops updating trackers when Kinect isn't sure about a position
			}
			else
			{
				KinectSettings::ignoreInferredPositions = false;
			}
		});

		showJointDevicesButton->GetSignal(sfg::ToggleButton::OnToggle).Connect([this]
		{
			kinectJointDevicesHiddenFromList = !showJointDevicesButton->IsActive();
			updateDeviceLists();
		});
		refreshDeviceListButton->GetSignal(sfg::Widget::OnLeftClick).Connect([this]
		{
			updateDeviceLists();
		});
		identifyPosDeviceButton->GetSignal(sfg::ToggleButton::OnToggle).Connect([this]
		{
			if (PositionDeviceList->GetItemCount())
			{
				auto globalIndex = selectedPositionDeviceIndex();
				auto d = TrackingPoolManager::getDeviceData(globalIndex);
				if (d.parentHandler)
				{
					d.parentHandler->identify(globalIndex, identifyPosDeviceButton->IsActive());
				}
				else
					LOG(ERROR) << "Attempted to identify a device with no valid parent handler bound";
			}
		});
		identifyRotDeviceButton->GetSignal(sfg::ToggleButton::OnToggle).Connect([this]
		{
			if (RotationDeviceList->GetItemCount())
			{
				auto globalIndex = selectedRotationDeviceIndex();
				auto d = TrackingPoolManager::getDeviceData(globalIndex);

				if (d.parentHandler)
				{
					d.parentHandler->identify(globalIndex, identifyRotDeviceButton->IsActive());
				}
				else
					LOG(ERROR) << "Attempted to identify a device with no valid parent handler bound";
			}
		});

		PositionDeviceList->GetSignal(sfg::ComboBox::OnSelect).Connect([this]
		{
			// QOL Change to make selecting trackers easier
			if (userSelectedDeviceRotIndex)
			{
				// Don't auto change
			}
			else
			{
				RotationDeviceList->SelectItem(PositionDeviceList->GetSelectedItem());
				userSelectedDeviceRotIndex = false;
				userSelectedDevicePosIndex = true;
			}
		});
		RotationDeviceList->GetSignal(sfg::ComboBox::OnSelect).Connect([this]
		{
			// QOL Change to make selecting trackers easier
			if (userSelectedDevicePosIndex)
			{
				// Don't auto change
			}
			else
			{
				PositionDeviceList->SelectItem(RotationDeviceList->GetSelectedItem());
				userSelectedDevicePosIndex = false;
				userSelectedDeviceRotIndex = true;
			}
		});

		psmovebox->GetSignal(sfg::ComboBox::OnSelect).Connect([this]
		{
			KinectSettings::psmh = psmovebox->GetSelectedItem();
			KinectSettings::flashnow[0] = psmovebox->GetSelectedItem();
			KinectSettings::flashnow[1] = true;
		});

		psmovebox1->GetSignal(sfg::ComboBox::OnSelect).Connect([this]
		{
			KinectSettings::psmm = psmovebox1->GetSelectedItem();
			KinectSettings::flashnow[0] = psmovebox1->GetSelectedItem();
			KinectSettings::flashnow[1] = true;
		});

		psmoveboxhi->GetSignal(sfg::ComboBox::OnSelect).Connect([this]
		{
			KinectSettings::psmhidari = psmoveboxhi->GetSelectedItem();
			KinectSettings::flashnow[0] = psmoveboxhi->GetSelectedItem();
			KinectSettings::flashnow[1] = true;
		});
		psmoveboxmi->GetSignal(sfg::ComboBox::OnSelect).Connect([this]
		{
			KinectSettings::psmmigi = psmoveboxmi->GetSelectedItem();
			KinectSettings::flashnow[0] = psmoveboxmi->GetSelectedItem();
			KinectSettings::flashnow[1] = true;
		});
		psmoveboxyo->GetSignal(sfg::ComboBox::OnSelect).Connect([this]
		{
			KinectSettings::psmyobu = psmoveboxyo->GetSelectedItem();
			KinectSettings::flashnow[0] = psmoveboxyo->GetSelectedItem();
			KinectSettings::flashnow[1] = true;
		});
		psmoveboxa->GetSignal(sfg::ComboBox::OnSelect).Connect([this]
		{
			KinectSettings::psmatama = psmoveboxa->GetSelectedItem();
			KinectSettings::flashnow[0] = psmoveboxa->GetSelectedItem();
			KinectSettings::flashnow[1] = true;
		});

		AddHandControllersToList->GetSignal(sfg::Widget::OnLeftClick).Connect([this]
		{
			//Add a left and right hand tracker as a controller
			addTrackerToList(KVR::KinectJointType::HandLeft, KVR::KinectDeviceRole::LeftHand, true);
			addTrackerToList(KVR::KinectJointType::HandRight, KVR::KinectDeviceRole::RightHand, true);
		});
		AddLowerTrackersToList->GetSignal(sfg::Widget::OnLeftClick).Connect([this]
		{
			addTrackerToList(KVR::KinectJointType::AnkleLeft, KVR::KinectDeviceRole::LeftFoot, false);
			addTrackerToList(KVR::KinectJointType::AnkleRight, KVR::KinectDeviceRole::RightFoot, false);
			addTrackerToList(KVR::KinectJointType::SpineBase, KVR::KinectDeviceRole::Hip, false);
		});
		AddTrackerToListButton->GetSignal(sfg::Widget::OnLeftClick).Connect([this]
		{
			addUserTrackerToList();
			userSelectedDeviceRotIndex = false;
			userSelectedDevicePosIndex = false;
		});
		RemoveTrackerFromListButton->GetSignal(sfg::Widget::OnLeftClick).Connect([this]
		{
			int i = 0;
			for (; i < TrackersToBeInitialised.size(); ++i)
			{
				if (TrackersToBeInitialised[i].radioButton->IsActive())
				{
					TrackersToBeInitialised[i].radioButton->Show(false);
					TrackersToBeInitialised.erase(TrackersToBeInitialised.begin() + i);
					break;
				}
			}
			//updateTempTrackerIDs();
			//updateTempTrackerButtonGroups();
		});

		connectPSMoveHandlerGUIEvents();

		HipScale->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				// Update the Global hip offset
				KinectSettings::hipRoleHeightAdjust = HipScale->GetValue();
			}
		);
		setCalibrationSignal();
	}

	void setColorTrackerSignals(ColorTracker& colorTracker)
	{
		InitiateColorTrackingButton->GetSignal(sfg::Button::OnMouseLeftPress).Connect([this, &colorTracker]
		{
			colorTracker.initialise();
		});
		DestroyColorTrackingButton->GetSignal(sfg::Button::OnMouseLeftPress).Connect([this, &colorTracker]
		{
			colorTracker.terminate();
		});
	}

	int selectedPositionDeviceIndex()
	{
		int posIndex = PositionDeviceList->GetSelectedItem();
		if (kinectJointDevicesHiddenFromList && TrackingPoolManager::trackerIdInKinectRange(posIndex))
			posIndex += KVR::KinectJointCount;
		// Really need to find a less hacky way to do this - as without it, when the kinect joints are hidden,
		// selecting a PSMove (ID of 25) would still use the kinect joint because it's technically the 0th item in the list
		return posIndex;
	}

	int selectedRotationDeviceIndex()
	{
		int rotIndex = RotationDeviceList->GetSelectedItem();
		if (kinectJointDevicesHiddenFromList && TrackingPoolManager::trackerIdInKinectRange(rotIndex))
			rotIndex += KVR::KinectJointCount;
		// Really need to find a less hacky way to do this - as without it, when the kinect joints are hidden,
		// selecting a PSMove (ID of 25) would still use the kinect joint because it's technically the 0th item in the list
		return rotIndex;
	}

	void addUserTrackerToList()
	{
		TempTracker temp;
		temp.GUID = TrackersToBeInitialised.size();
		temp.data.isController = IsControllerButton->IsActive();

		// Obtain Position Information
		int posIndex = selectedPositionDeviceIndex();
		if (posIndex < 0 || posIndex == k_invalidTrackerID) return;
		KVR::TrackedDeviceInputData posData = TrackingPoolManager::getDeviceData(posIndex);

		temp.data.positionGlobalDeviceId = posIndex;
		temp.positionTrackingOption = posData.positionTrackingOption;
		temp.data.posDeviceName = posData.deviceName;
		temp.data.posDeviceSerial = posData.serial;

		// Obtain Rotation Information
		int rotIndex = selectedRotationDeviceIndex();
		if (rotIndex < 0 || rotIndex == k_invalidTrackerID) return;
		KVR::TrackedDeviceInputData rotData = TrackingPoolManager::getDeviceData(rotIndex);

		temp.data.rotationGlobalDeviceId = rotIndex;
		temp.rotationTrackingOption = rotData.rotationTrackingOption;
		temp.data.rotDeviceName = rotData.deviceName;
		temp.data.rotDeviceSerial = rotData.serial;

		temp.data.role = static_cast<KVR::KinectDeviceRole>(RolesList->GetSelectedItem());
		updateTrackerLists(temp);
	}

	bool validatedTrackerData(TempTrackerData& data)
	{
		// Verify that data is correct

		// Index bound checks to prevent array access errors
		bool posMismatched = false;
		bool rotMismatched = false;
		if (data.positionGlobalDeviceId >= TrackingPoolManager::count())
		{
			// INVALID POS ID
			LOG(WARNING) << "POSITION ID " << data.positionGlobalDeviceId << " GREATER THAN THE SIZE OF TRACKING POOL";
			posMismatched = true;
		}
		if (data.rotationGlobalDeviceId >= TrackingPoolManager::count())
		{
			// INVALID ROT ID
			LOG(WARNING) << "ROTATION ID " << data.rotationGlobalDeviceId << " GREATER THAN THE SIZE OF TRACKING POOL";
			rotMismatched = true;
		}

		KVR::TrackedDeviceInputData posData = TrackingPoolManager::getDeviceData(data.positionGlobalDeviceId);
		KVR::TrackedDeviceInputData rotData = TrackingPoolManager::getDeviceData(data.rotationGlobalDeviceId);

		// Mismatched Device Index Checks
		if (data.posDeviceName != posData.deviceName)
		{
			// POTENTIALLY MISMATCHED DEVICE
			LOG(WARNING) << "POTENTIALLY MISMATCHED POS DEVICE NAME, EXPECTED " << data.posDeviceName <<
				" AND RECEIVED " << posData.deviceName;

			// If serial is also wrong, panic
			if (data.posDeviceSerial != posData.serial)
			{
				LOG(ERROR) << "MISMATCHED POS DEVICE SERIAL, EXPECTED " << data.posDeviceSerial << " AND RECEIVED " <<
					posData.serial;
				posMismatched = true;
			}
		}
		if (data.rotDeviceName != rotData.deviceName)
		{
			// POTENTIALLY MISMATCHED DEVICE
			LOG(WARNING) << "POTENTIALLY MISMATCHED ROT DEVICE NAME, EXPECTED " << data.rotDeviceName <<
				" AND RECEIVED " << rotData.deviceName;

			// If serial is also wrong, panic
			if (data.rotDeviceSerial != rotData.serial)
			{
				LOG(ERROR) << "MISMATCHED ROT DEVICE SERIAL, EXPECTED " << data.rotDeviceSerial << " AND RECEIVED " <<
					rotData.serial;
				rotMismatched = true;
			}
		}

		// If incorrect, search for device
		if (posMismatched)
		{
			LOG(INFO) << "Attempting to find Pos ID from device info...";
			uint32_t potentialNewID = TrackingPoolManager::locateGlobalDeviceID(data.posDeviceSerial);
			if (potentialNewID != k_invalidTrackerID)
			{
				LOG(INFO) << "Replacement Pos ID successfully found!";
				posMismatched = false;
				data.positionGlobalDeviceId = potentialNewID;
			}
			else
			{
				LOG(ERROR) << "Could not relocate pos device ID to spawn!";
			}
		}
		if (rotMismatched)
		{
			LOG(INFO) << "Attempting to find Rot ID from device info...";
			uint32_t potentialNewID = TrackingPoolManager::locateGlobalDeviceID(data.rotDeviceSerial);
			if (potentialNewID != k_invalidTrackerID)
			{
				LOG(INFO) << "Replacement Rot ID successfully found!";
				rotMismatched = false;
				data.rotationGlobalDeviceId = potentialNewID;
			}
			else
			{
				LOG(ERROR) << "Could not relocate rot device ID to spawn!";
			}
		}

		bool failed = rotMismatched || posMismatched;
		// If could not be found, produce warning to cancel
		if (failed)
		{
			return false;
		}
		return true;
	}

	bool addUserTrackerToList(TempTrackerData& data)
	{
		bool dataIsValid = validatedTrackerData(data);
		if (!dataIsValid)
			return false;

		TempTracker temp;
		temp.data = data;
		temp.GUID = TrackersToBeInitialised.size();

		KVR::TrackedDeviceInputData posData = TrackingPoolManager::getDeviceData(data.positionGlobalDeviceId);
		KVR::TrackedDeviceInputData rotData = TrackingPoolManager::getDeviceData(data.rotationGlobalDeviceId);

		temp.positionTrackingOption = posData.positionTrackingOption;
		temp.rotationTrackingOption = rotData.rotationTrackingOption;

		updateTrackerLists(temp);

		return true;
	}

	void addTrackerToList(KVR::KinectJointType joint, KVR::KinectDeviceRole role, bool isController)
	{
		TempTracker temp;
		temp.GUID = TrackersToBeInitialised.size();
		temp.data.isController = isController;
		// Obtain Position Information
		int posIndex = TrackingPoolManager::globalDeviceIDFromJoint(joint);
		if (posIndex < 0 || posIndex == k_invalidTrackerID) return;
		KVR::TrackedDeviceInputData posData = TrackingPoolManager::getDeviceData(posIndex);

		temp.data.positionGlobalDeviceId = posIndex;
		temp.positionTrackingOption = posData.positionTrackingOption;
		temp.data.posDeviceName = posData.deviceName;
		temp.data.posDeviceSerial = posData.serial;

		// Obtain Rotation Information
		int rotIndex = TrackingPoolManager::globalDeviceIDFromJoint(joint);
		if (rotIndex < 0 || rotIndex == k_invalidTrackerID) return;
		KVR::TrackedDeviceInputData rotData = TrackingPoolManager::getDeviceData(rotIndex);

		temp.data.rotationGlobalDeviceId = rotIndex;
		temp.rotationTrackingOption = rotData.rotationTrackingOption;
		temp.data.rotDeviceName = rotData.deviceName;
		temp.data.rotDeviceSerial = rotData.serial;
		temp.data.role = role;

		updateTrackerLists(temp);
	}

	void updateTrackerLists(TempTracker& temp)
	{
		// Display a radio button menu where selecting each button selects that tracker
		// Displays the joint of each tracker and (Tracker)/(Controller)
		std::stringstream roleStrStream;
		if (temp.data.isController)
			roleStrStream << " (Tracked Controller) ";
		else
			roleStrStream << " (Tracker) ";
		roleStrStream << "(Role: " << KVR::KinectDeviceRoleName[static_cast<int>(temp.data.role)] << ") ";
		std::string posName = TrackingPoolManager::deviceGuiString(temp.data.positionGlobalDeviceId);
		std::string rotName = TrackingPoolManager::deviceGuiString(temp.data.rotationGlobalDeviceId);
		std::string finalTrackerName = "Position: " + posName + " | Rotation: " + rotName + " | " + roleStrStream.str();

		LOG(INFO) << "Adding tracker to list :: " << finalTrackerName;

		temp.radioButton = sfg::RadioButton::Create(finalTrackerName);
		if (TrackersToBeInitialised.size())
		{
			auto group = TrackersToBeInitialised.back().radioButton->GetGroup();
			temp.radioButton->SetGroup(group);
		}

		TrackerList->Pack(temp.radioButton);

		TrackersToBeInitialised.push_back(temp);
	}

	void refreshCalibrationMenuValues()
	{
		using namespace KinectSettings;
		CalibrationEntryrPosX->SetValue(moffsets[0][0].v[0]);
		CalibrationEntryrPosY->SetValue(moffsets[0][0].v[1]);
		CalibrationEntryrPosZ->SetValue(moffsets[0][0].v[2]);

		CalibrationEntryrRotX->SetValue(moffsets[1][0].v[0]);
		CalibrationEntryrRotY->SetValue(moffsets[1][0].v[1]);
		CalibrationEntryrRotZ->SetValue(moffsets[1][0].v[2]);

		CalibrationEntrylPosX->SetValue(moffsets[0][1].v[0]);
		CalibrationEntrylPosY->SetValue(moffsets[0][1].v[1]);
		CalibrationEntrylPosZ->SetValue(moffsets[0][1].v[2]);

		CalibrationEntrylRotX->SetValue(moffsets[1][1].v[0]);
		CalibrationEntrylRotY->SetValue(moffsets[1][1].v[1]);
		CalibrationEntrylRotZ->SetValue(moffsets[1][1].v[2]);

		CalibrationEntryhPosX->SetValue(moffsets[0][2].v[0]);
		CalibrationEntryhPosY->SetValue(moffsets[0][2].v[1]);
		CalibrationEntryhPosZ->SetValue(moffsets[0][2].v[2]);

		CalibrationEntryhRotX->SetValue(moffsets[1][2].v[0]);
		CalibrationEntryhRotY->SetValue(moffsets[1][2].v[1]);
		CalibrationEntryhRotZ->SetValue(moffsets[1][2].v[2]);
	}

	void setCalibrationSignal()
	{
		CalibrationEntryrPosX->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::moffsets[0][0].v[0] = CalibrationEntryrPosX->GetValue();
				KinectSettings::sensorConfigChanged = true;
			}
		);
		CalibrationEntryrPosY->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::moffsets[0][0].v[1] = CalibrationEntryrPosY->GetValue();
				KinectSettings::sensorConfigChanged = true;
			}
		);
		CalibrationEntryrPosZ->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::moffsets[0][0].v[2] = CalibrationEntryrPosZ->GetValue();
				KinectSettings::sensorConfigChanged = true;
			}
		);

		CalibrationEntrylPosX->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::moffsets[0][1].v[0] = CalibrationEntrylPosX->GetValue();
				KinectSettings::sensorConfigChanged = true;
			}
		);
		CalibrationEntrylPosY->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::moffsets[0][1].v[1] = CalibrationEntrylPosY->GetValue();
				KinectSettings::sensorConfigChanged = true;
			}
		);
		CalibrationEntrylPosZ->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::moffsets[0][1].v[2] = CalibrationEntrylPosZ->GetValue();
				KinectSettings::sensorConfigChanged = true;
			}
		);

		CalibrationEntryhPosX->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::moffsets[0][2].v[0] = CalibrationEntryhPosX->GetValue();
				KinectSettings::sensorConfigChanged = true;
			}
		);
		CalibrationEntryhPosY->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::moffsets[0][2].v[1] = CalibrationEntryhPosY->GetValue();
				KinectSettings::sensorConfigChanged = true;
			}
		);
		CalibrationEntryhPosZ->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::moffsets[0][2].v[2] = CalibrationEntryhPosZ->GetValue();
				KinectSettings::sensorConfigChanged = true;
			}
		);

		CalibrationEntryrRotX->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::moffsets[1][0].v[0] = CalibrationEntryrRotX->GetValue();
				KinectSettings::updateKinectQuaternion();
				KinectSettings::sensorConfigChanged = true;
			}
		);
		CalibrationEntryrRotY->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::moffsets[1][0].v[1] = CalibrationEntryrRotY->GetValue();
				KinectSettings::updateKinectQuaternion();
				KinectSettings::sensorConfigChanged = true;
			}
		);
		CalibrationEntryrRotZ->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::moffsets[1][0].v[2] = CalibrationEntryrRotZ->GetValue();
				KinectSettings::updateKinectQuaternion();
				KinectSettings::sensorConfigChanged = true;
			}
		);

		CalibrationEntrylRotX->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::moffsets[1][1].v[0] = CalibrationEntrylRotX->GetValue();
				KinectSettings::updateKinectQuaternion();
				KinectSettings::sensorConfigChanged = true;
			}
		);
		CalibrationEntrylRotY->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::moffsets[1][1].v[1] = CalibrationEntrylRotY->GetValue();
				KinectSettings::updateKinectQuaternion();
				KinectSettings::sensorConfigChanged = true;
			}
		);
		CalibrationEntrylRotZ->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::moffsets[1][1].v[2] = CalibrationEntrylRotZ->GetValue();
				KinectSettings::updateKinectQuaternion();
				KinectSettings::sensorConfigChanged = true;
			}
		);

		CalibrationEntryhRotX->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::moffsets[1][2].v[0] = CalibrationEntryhRotX->GetValue();
				KinectSettings::updateKinectQuaternion();
				KinectSettings::sensorConfigChanged = true;
			}
		);
		CalibrationEntryhRotY->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::moffsets[1][2].v[1] = CalibrationEntryhRotY->GetValue();
				KinectSettings::updateKinectQuaternion();
				KinectSettings::sensorConfigChanged = true;
			}
		);
		CalibrationEntryhRotZ->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				KinectSettings::moffsets[1][2].v[2] = CalibrationEntryhRotZ->GetValue();
				KinectSettings::updateKinectQuaternion();
				KinectSettings::sensorConfigChanged = true;
			}
		);

		CalibrationSaveButton->GetSignal(sfg::Button::OnLeftClick).Connect([this]
			{
				KinectSettings::updateKinectQuaternion();
				KinectSettings::writeKinectSettings();
			}
		);
	}

	void loadK2VRIntoBindingsMenu(vr::IVRSystem*& m_VRSystem)
	{
		// Only scene apps currently actually load into the Bindings menu
		// So, this momentarily opens the vrsystem as a scene, and closes it
		// Which actually allows the menu to stay open, while still functioning as normal
		do
		{
			vr::EVRInitError eError = vr::VRInitError_None;
			vr::VR_Shutdown();
			LOG(INFO) << "(Workaround/Hack) Loading K2VR into bindings menu...";
			m_VRSystem = VR_Init(&eError, vr::VRApplication_Overlay);
			Sleep(100); // Necessary because of SteamVR timing occasionally being too quick to change the scenes
			vr::VR_Shutdown();
			m_VRSystem = VR_Init(&eError, vr::VRApplication_Background);
			LOG_IF(eError != vr::EVRInitError::VRInitError_None, ERROR) <<
 " (Workaround/Hack) VR System failed to reinitialise, attempting again...";
		}
		while (m_VRSystem == nullptr); // Potential Segfault if not actually initialised and used later on
		LOG(INFO) << "(Workaround/Hack) Successfully loaded K2VR into bindings menu!";
	}

	void setVRSceneChangeButtonSignal(vr::IVRSystem*& m_VRSystem)
	{
		ActivateVRSceneTypeButton->GetSignal(sfg::Widget::OnLeftClick).Connect([this, &m_VRSystem]
		{
			loadK2VRIntoBindingsMenu(m_VRSystem);
		});
	}

	void refreshpsms()
	{
		psmovebox->Clear();
		psmovebox1->Clear();
		psmoveboxhi->Clear();
		psmoveboxmi->Clear();
		psmoveboxyo->Clear();
		psmoveboxa->Clear();

		KinectSettings::psmindexidpsm[0].clear();
		KinectSettings::psmindexidpsm[1].clear();

		for (int i = 0; i < 11; i++)
		{
			if (KinectSettings::KVRPSMoveData[i].isValidController)
			{
				psmovebox->AppendItem("PSMove ID: " + boost::lexical_cast<std::string>(i));
				psmovebox1->AppendItem("PSMove ID: " + boost::lexical_cast<std::string>(i));
				psmoveboxhi->AppendItem("PSMove ID: " + boost::lexical_cast<std::string>(i));
				psmoveboxmi->AppendItem("PSMove ID: " + boost::lexical_cast<std::string>(i));
				psmoveboxyo->AppendItem("PSMove ID: " + boost::lexical_cast<std::string>(i));
				psmoveboxa->AppendItem("PSMove ID: " + boost::lexical_cast<std::string>(i));

				KinectSettings::psmindexidpsm[0].push_back(psmovebox->GetItemCount() - 1);
				KinectSettings::psmindexidpsm[1].push_back(i);
			}
		}
		if (psmovebox->GetItemCount() >= 1)
		{
			psmovebox->SelectItem(0);
			KinectSettings::psmh = 0;
		}
		if (psmovebox1->GetItemCount() >= 2)
		{
			psmovebox1->SelectItem(1);
			KinectSettings::psmm = 1;
		}
		if (psmoveboxhi->GetItemCount() >= 1)
		{
			psmoveboxhi->SelectItem(0);
			KinectSettings::psmhidari = 0;
		}
		if (psmoveboxmi->GetItemCount() >= 2)
		{
			psmoveboxmi->SelectItem(1);
			KinectSettings::psmmigi = 1;
		}
		if (psmoveboxyo->GetItemCount() >= 3)
		{
			psmoveboxyo->SelectItem(2);
			KinectSettings::psmyobu = 2;
		}
		if (psmoveboxa->GetItemCount() >= 1)
		{
			psmoveboxa->SelectItem(0);
			KinectSettings::psmatama = 0;
		}
	}

	void setKinectButtonSignal(KinectHandlerBase& kinect)
	{
		reconKinectButton->GetSignal(sfg::Widget::OnLeftClick).Connect([&kinect]
		{
			kinect.initialise();
		});

		refreshpsmovesbuton->GetSignal(sfg::Widget::OnLeftClick).Connect([this]
		{
			refreshpsms();
		});

		refreshpsmovesbuton1->GetSignal(sfg::Widget::OnLeftClick).Connect([this]
		{
			refreshpsms();
		});

		refreshpsmovesbuton11->GetSignal(sfg::Widget::OnLeftClick).Connect([this]
		{
			refreshpsms();
		});

		refreshcomports->GetSignal(sfg::Widget::OnLeftClick).Connect([this]
		{
			wchar_t lpTargetPath[5000];
			comportbox1->Clear();
			comportbox2->Clear();

			for (int i = 0; i < 255; i++) // checking ports from COM0 to COM255
			{
				std::wstring str = L"COM" + std::to_wstring(i); // converting to COM0, COM1, COM2
				DWORD res = QueryDosDevice(str.c_str(), lpTargetPath, 5000);

				// Test the return value and error if any
				if (res != 0) //QueryDosDevice returns zero if it didn't find an object
				{
					comportbox1->AppendItem(str);
					comportbox2->AppendItem(str);
					//std::cout << str << ": " << lpTargetPath << std::endl;
				}
				if (GetLastError() == ERROR_INSUFFICIENT_BUFFER)
				{
				}
			}

			comportbox1->SelectItem(0);
			comportbox2->SelectItem(1);
		});
	}

	void spawnAndConnectTracker(vrinputemulator::VRInputEmulator& inputE,
	                            std::vector<KVR::KinectTrackedDevice>& v_trackers, TempTracker t_tracker)
	{
		KVR::KinectTrackedDevice device(inputE, t_tracker.data.positionGlobalDeviceId,
		                                t_tracker.data.rotationGlobalDeviceId, t_tracker.data.role);
		device.positionTrackingOption = t_tracker.positionTrackingOption;
		device.rotationTrackingOption = t_tracker.rotationTrackingOption;
		device.customModelName = TrackingPoolManager::getDeviceData(t_tracker.data.positionGlobalDeviceId).
			customModelName;
		device.init(inputE);
		v_trackers.push_back(device);
	}

	std::wstring s2ws(const std::string& s)
	{
		int len;
		int slength = static_cast<int>(s.length()) + 1;
		len = MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, nullptr, 0);
		wchar_t* buf = new wchar_t[len];
		MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, buf, len);
		std::wstring r(buf);
		delete[] buf;
		return r;
	}

	void setTrackerButtonSignals(std::vector<KVR::KinectTrackedDevice>& v_trackers, vr::IVRSystem*& m_VRSystem)
	{
		calibrateOffsetButton->GetSignal(sfg::Widget::OnLeftClick).Connect([this, &v_trackers, &m_VRSystem]
		{
			// WARNING, SUPER HACKY!!!
			// Spawn device which sets it's vec position to 0
			vr::DriverPose_t pose = defaultReadyDriverPose();

			pose.vecPosition[0] = 0;
			pose.vecPosition[1] = 0;
			pose.vecPosition[2] = 0;
			pose.vecWorldFromDriverTranslation[0] -= KinectSettings::trackingOriginPosition.v[0];
			pose.vecWorldFromDriverTranslation[1] -= KinectSettings::trackingOriginPosition.v[1];
			pose.vecWorldFromDriverTranslation[2] -= KinectSettings::trackingOriginPosition.v[2];
			v_trackers[0].nextUpdatePoseIsSet = false;
			v_trackers[0].setPoseForNextUpdate(pose, true);
			// Get it's VR ID
			//auto info = inputE.getVirtualDeviceInfo(0);
			//uint32_t vrID = info.openvrDeviceId;
			// Get it's absolute tracking, set the secondary offset to that

			//v_trackers[0].update(pose);

			//vr::TrackedDevicePose_t devicePose[vr::k_unMaxTrackedDeviceCount];
			//m_VRSystem->GetDeviceToAbsoluteTrackingPose(vr::ETrackingUniverseOrigin::TrackingUniverseStanding, 0, devicePose, vr::k_unMaxTrackedDeviceCount);

			//KinectSettings::secondaryTrackingOriginOffset = GetVRPositionFromMatrix(devicePose[vrID].mDeviceToAbsoluteTracking);
			//LOG(INFO) << "SET THE SECONDARY OFFSET TO " << KinectSettings::secondaryTrackingOriginOffset.v[0] << ", " << KinectSettings::secondaryTrackingOriginOffset.v[1] << ", " << KinectSettings::secondaryTrackingOriginOffset.v[2];
		});

		TrackerInitButton->GetSignal(sfg::Widget::OnLeftClick).Connect([this, &v_trackers]
		{
			if (!KinectSettings::initialised)
			{
				/*
				bool reuseLastTrackers = true;
				if (reuseLastTrackers) {
					LOG(INFO) << "SPAWNING TRACKERS FROM LAST OPEN, MAY BE ISSUES";

					// Load Last Set of trackers used

					// Spawn
				}
				*/
				TrackerInitButton->SetLabel("Trackers Initialised - Destroy Trackers");
				spawnDefaultLowerBodyTrackers();

				showPostTrackerInitUI();

				TrackerLastInitButton->SetState(sfg::Widget::State::INSENSITIVE);

				modeTitleBox110->Show(!KinectSettings::isKinectPSMS);
				TDegreeButton->SetValue(KinectSettings::cpoints);
				TrackersConfigSaveButton->Show(true);
				TrackersCalibButton->Show(true);
				//TrackersCalibSButton->Show(true);
				expcalibbutton->Show(!KinectSettings::isKinectPSMS); //Manual only if PSMS

				AutoStartTrackers->Show(true);
				//AutoStartKinectToVR->Show(true);

				KinectSettings::initialised = true;
			}
			else
			{
				KinectSettings::initialised = false;
				TrackerInitButton->SetLabel("Spawn Trackers");
			}
		});
		// Make sure that users don't get confused and hit the spawn last button when they don't need it
		/*bool foundCachedTrackers = trackerConfigExists() ? true : false;
		TrackerLastInitButton->Show(foundCachedTrackers);*/

		if (VirtualHips::settings.astartt && KinectSettings::isDriverPresent)
		{
			std::thread* st = new std::thread([this]
			{
				std::this_thread::sleep_for(std::chrono::seconds(7));
				TrackerInitButton->SetLabel("Trackers Initialised - Destroy Trackers");
				spawnDefaultLowerBodyTrackers();

				showPostTrackerInitUI();

				TrackerLastInitButton->SetState(sfg::Widget::State::INSENSITIVE);

				modeTitleBox110->Show(!KinectSettings::isKinectPSMS);
				TDegreeButton->SetValue(KinectSettings::cpoints);
				TrackersConfigSaveButton->Show(true);
				TrackersCalibButton->Show(true);
				//TrackersCalibSButton->Show(true);
				expcalibbutton->Show(!KinectSettings::isKinectPSMS);

				AutoStartTrackers->Show(true);
				//AutoStartKinectToVR->Show(true);

				KinectSettings::initialised = true;
			});
		}

		//if (VirtualHips::settings.astarth)
		//{
		//	KinectSettings::headtracked = true;
		//	HeadTrackingStartButton->SetLabel("Head Tracking Started");
		//	HeadTrackingStartButton->Show(false);
		//	HeadTrackingCalibButton->Show(true);
		//	AutoStartHeadTracking->Show(true);

		//	std::thread* hedo_of = new std::thread([]()
		//	{
		//		while (true)
		//		{
		//			HANDLE pipeAtama = CreateFile(
		//				TEXT("\\\\.\\pipe\\LogPipeOf"), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0,
		//				nullptr);

		//			DWORD Written;

		//			std::string AtamaS = [&]()-> std::string
		//			{
		//				std::stringstream S;
		//				S << "X" << 10000 * (KinectSettings::huoffsets.v[0]) <<
		//					"/Y" << 10000 * (KinectSettings::huoffsets.v[1]) <<
		//					"/Z" << 10000 * (KinectSettings::huoffsets.v[2]) <<
		//					"/R" << 10000 * (0) << "/"; //DEPRECATED: GLM_ROTATE SCREWED UP WITH > 99

		//				return S.str();
		//			}();

		//			char AtamaD[1024];
		//			strcpy_s(AtamaD, AtamaS.c_str());

		//			WriteFile(pipeAtama, AtamaD, sizeof(AtamaD), &Written, nullptr);
		//			CloseHandle(pipeAtama);

		//			std::this_thread::sleep_for(std::chrono::milliseconds(370));
		//		}
		//	});
		//}

		//if (VirtualHips::settings.astarta)
		//{
		//	/*
		//	comportbox1->Clear();
		//	comportbox1->AppendItem(VirtualHips::settings.comph);
		//	comportbox1->SelectItem(0);

		//	comportbox2->Clear();
		//	comportbox2->AppendItem(VirtualHips::settings.compm);
		//	comportbox2->SelectItem(0);*/

		//	std::thread* activate = new std::thread([]
		//	{
		//		try
		//		{
		//			using namespace boost::asio;
		//			using ip::tcp;
		//			using std::string;

		//			io_service io_service;
		//			//socket creation
		//			tcp::socket socket(io_service);
		//			//connection
		//			socket.connect(tcp::endpoint(ip::address::from_string("127.0.0.1"), 5741));
		//			// request/message from client

		//			const string msg = "Hello from Client!\n";
		//			boost::system::error_code error;
		//			boost::asio::write(socket, buffer(msg), error);
		//			if (!error)
		//			{
		//				LOG(INFO) << "Client sent message!";
		//			}
		//			else
		//			{
		//				LOG(INFO) << "(Ignore) send failed: " << error.message();
		//			}

		//			// getting response from server
		//			streambuf receive_buffer;
		//			boost::asio::read(socket, receive_buffer, transfer_all(), error);
		//			if (error && error != error::eof)
		//			{
		//				LOG(INFO) << "(Ignore) receive failed: " << error.message();
		//			}
		//			else
		//			{
		//				const char* data = boost::asio::buffer_cast<const char*>(receive_buffer.data());
		//			}
		//		}
		//		catch (boost::exception const& e)
		//		{
		//		}
		//		catch (std::exception e)
		//		{
		//		}
		//	});

		//	//ShellExecute(NULL, _T("open"), _T("avr_hhost.exe"), s2ws(VirtualHips::settings.comph).c_str(), NULL, SW_HIDE);
		//	//ShellExecute(NULL, _T("open"), _T("avr_mhost.exe"), s2ws(VirtualHips::settings.compm).c_str(), NULL, SW_HIDE);

		//	stopControllers->SetState(sfg::Widget::State::NORMAL);
		//	startControllers->SetState(sfg::Widget::State::INSENSITIVE);
		//	//comportbox1->SetState(sfg::Widget::State::INSENSITIVE);
		//	//comportbox2->SetState(sfg::Widget::State::INSENSITIVE);
		//	//refreshcomports->SetState(sfg::Widget::State::INSENSITIVE);
		//}

		TrackerLastInitButton->GetSignal(sfg::Widget::OnLeftClick).Connect([this, &v_trackers]
		{
			if (!retrieveLastSpawnedTrackers())
			{
				return; // Don't actually spawn the trackers, as they will likely crash
			}
			TrackerLastInitButton->SetLabel("Trackers Initialised - Destroy Trackers");
			spawnDefaultLowerBodyTrackers();

			showPostTrackerInitUI();

			TrackerInitButton->SetState(sfg::Widget::State::INSENSITIVE);
			TrackerLastInitButton->SetState(sfg::Widget::State::INSENSITIVE);
		});

		SetJointsToFootRotationButton->GetSignal(sfg::Widget::OnLeftClick).Connect([this, &v_trackers]
		{
			using namespace KinectSettings;
			leftFootJointWithRotation = KVR::KinectJointType::FootLeft;
			rightFootJointWithRotation = KVR::KinectJointType::FootRight;
			leftFootJointWithoutRotation = KVR::KinectJointType::AnkleLeft;
			rightFootJointWithoutRotation = KVR::KinectJointType::AnkleRight;
			for (KVR::KinectTrackedDevice& d : v_trackers)
			{
				if (d.role == KVR::KinectDeviceRole::LeftFoot)
				{
					d.joint0 = leftFootJointWithRotation;
					d.joint1 = leftFootJointWithoutRotation;
				}
				if (d.role == KVR::KinectDeviceRole::RightFoot)
				{
					d.joint0 = rightFootJointWithRotation;
					d.joint1 = rightFootJointWithoutRotation;
				}
			}
		});
		SetJointsToAnkleRotationButton->GetSignal(sfg::Widget::OnLeftClick).Connect([this, &v_trackers]
		{
			using namespace KinectSettings;
			leftFootJointWithRotation = KVR::KinectJointType::AnkleLeft;
			rightFootJointWithRotation = KVR::KinectJointType::AnkleRight;
			leftFootJointWithoutRotation = KVR::KinectJointType::FootLeft;
			rightFootJointWithoutRotation = KVR::KinectJointType::FootRight;
			for (KVR::KinectTrackedDevice& d : v_trackers)
			{
				if (d.role == KVR::KinectDeviceRole::LeftFoot)
				{
					d.joint0 = leftFootJointWithRotation;
					d.joint1 = leftFootJointWithoutRotation;
				}
				if (d.role == KVR::KinectDeviceRole::RightFoot)
				{
					d.joint0 = rightFootJointWithRotation;
					d.joint1 = rightFootJointWithoutRotation;
				}
			}
		});

		SetAllJointsRotUnfiltered->GetSignal(sfg::Widget::OnLeftClick).Connect([this, &v_trackers]
		{
			for (KVR::KinectTrackedDevice& d : v_trackers)
			{
				if (d.isSensor())
				{
				}
				else
				{
					d.rotationFilterOption = KVR::JointRotationFilterOption::Unfiltered;
				}
			}
		});
		SetAllJointsRotFiltered->GetSignal(sfg::Widget::OnLeftClick).Connect([this, &v_trackers]
		{
			for (KVR::KinectTrackedDevice& d : v_trackers)
			{
				if (d.isSensor())
				{
				}
				else
				{
					d.rotationFilterOption = KVR::JointRotationFilterOption::Filtered;
				}
			}
		});
		SetAllJointsRotHead->GetSignal(sfg::Widget::OnLeftClick).Connect([this, &v_trackers]
		{
			for (KVR::KinectTrackedDevice& d : v_trackers)
			{
				if (d.isSensor())
				{
				}
				else
				{
					d.rotationFilterOption = KVR::JointRotationFilterOption::HeadLook;
				}
			}
		});

		using namespace VirtualHips;

		HeadTrackingStartButton->GetSignal(sfg::Button::OnLeftClick).Connect([this]
			{
				KinectSettings::headtracked = true;
				HeadTrackingStartButton->SetLabel("Head Tracking Started");
				HeadTrackingStartButton->Show(false);
				HeadTrackingCalibButton->Show(true);
				AutoStartHeadTracking->Show(true);

				std::thread* hedo_of = new std::thread([]()
				{
					while (true)
					{
						HANDLE pipeAtama = CreateFile(
							TEXT("\\\\.\\pipe\\LogPipeOf"), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0,
							nullptr);

						DWORD Written;

						std::string AtamaS = [&]()-> std::string
						{
							std::stringstream S;
							S << "X" << 10000 * (KinectSettings::huoffsets.v[0]) <<
								"/Y" << 10000 * (KinectSettings::huoffsets.v[1]) <<
								"/Z" << 10000 * (KinectSettings::huoffsets.v[2]) <<
								"/R" << 10000 * (0) << "/"; //DEPRECATED: GLM_ROTATE SCREWED UP WITH > 99

							return S.str();
						}();

						char AtamaD[1024];
						strcpy_s(AtamaD, AtamaS.c_str());

						WriteFile(pipeAtama, AtamaD, sizeof(AtamaD), &Written, nullptr);
						CloseHandle(pipeAtama);
					}
				});
			}
		);

		HeadTrackingCalibButton->GetSignal(sfg::Button::OnLeftClick).Connect([this]
			{
				KinectSettings::headtracked = true;

				vr::EVRInitError error;
				vr::TrackedDevicePose_t trackedDevicePose;
				std::this_thread::sleep_for(std::chrono::milliseconds(2000));

				vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, &trackedDevicePose, 1);
				double yaw = std::atan2(trackedDevicePose.mDeviceToAbsoluteTracking.m[0][2],
					trackedDevicePose.mDeviceToAbsoluteTracking.m[2][2]);
			
				if (yaw < 0.0)
				{
					yaw = 2 * M_PI + yaw;
				}

				/*svrposes.qRotation = quaternion;
	
				KinectSettings::hmdquat = quaternion;*/

				//glm::vec3 hmdrot = glm::eulerAngles(glm::quat(svrposes.qRotation.w, svrposes.qRotation.x, svrposes.qRotation.y, svrposes.qRotation.z));

				KinectSettings::hroffset = glm::degrees(yaw);
				DegreeButton->SetValue(static_cast<float>(glm::degrees(yaw)));
				settings.hmdegree = glm::degrees(yaw);

				/*glm::vec3 fixedpos = glm::rotateY(glm::vec4(
					glm::vec3(svrposes[0].vecPosition[0], svrposes[0].vecPosition[1], svrposes[0].vecPosition[2]), 1),
					glm::f32(glm::radians(KinectSettings::hroffset)));*/

				/*glm::vec3 fixedpos = glm::rotateY(glm::vec4(
					glm::vec3(KinectSettings::mposes[0].v[0], KinectSettings::mposes[0].v[1], KinectSettings::mposes[0].v[2]), 1),
					glm::f32(glm::radians(KinectSettings::hroffset)));*/

				//glm::vec3 diff = glm::vec3(
				//    KinectSettings::mposes[0].v[0] - fixedpos.x,
				//    KinectSettings::mposes[0].v[1] - fixedpos.y,
				//    KinectSettings::mposes[0].v[2] - fixedpos.z);

				//KinectSettings::hoffsets.v[0] = -svrposes[0].vecPosition[0];
				//KinectSettings::hoffsets.v[1] = 0.f;// -KinectSettings::mposes[0].v[1];
				//KinectSettings::hoffsets.v[2] = -svrposes[0].vecPosition[2];

				//HeadTrackingCalibButton->SetLabel(std::string("Calibrate: Look at Kinect and stand still").c_str());

				saveSettings();
			}
		);
	}

	void updateTrackerInitButtonLabelFail()
	{
		TrackerInitButton->SetLabel("Input Emulator not connected! Can't init trackers");
	}

	void setReconnectControllerButtonSignal(VRcontroller& left, VRcontroller& right, vr::IVRSystem*& sys
	)
	{
		ReconControllersButton->GetSignal(sfg::Button::OnLeftClick).Connect([&left, &right, &sys, this]
		{
			std::stringstream stream;
			stream <<
				"If controller input isn't working, press this to reconnect them.\n Make sure both are on, and not in standby.\n";
			if (right.Connect(sys))
			{
				stream << "RIGHT: OK!\t";
			}
			else
			{
				stream << "RIGHT: DISCONNECTED!\t";
			}
			if (left.Connect(sys))
			{
				stream << "LEFT: OK!\t";
			}
			else
			{
				stream << "LEFT: DISCONNECTED!\t";
			}
			ReconControllersLabel->SetText(stream.str());
		});
	}

	vr::HmdQuaternion_t GetRotation(vr::HmdMatrix34_t matrix)
	{
		vr::HmdQuaternion_t q;

		q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
		q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
		q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
		q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;
		q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
		q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
		q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);
		return q;
	}

	vr::HmdVector3_t GetPosition(vr::HmdMatrix34_t matrix)
	{
		vr::HmdVector3_t vector;

		vector.v[0] = matrix.m[0][3];
		vector.v[1] = matrix.m[1][3];
		vector.v[2] = matrix.m[2][3];

		return vector;
	}

	void getsvrposesnrots(vr::DriverPose_t in_out)
	{
		vr::EVRInitError error;
		vr::IVRSystem* system = VR_Init(&error, vr::VRApplication_Background);

		vr::TrackedDevicePose_t trackedDevicePose;
		vr::TrackedDevicePose_t trackedControllerPose;
		vr::VRControllerState_t controllerState;
		vr::HmdMatrix34_t poseMatrix;
		vr::HmdVector3_t position;
		vr::HmdQuaternion_t quaternion;
		vr::VRControllerState_t state;

		system->GetControllerState(0, &state, sizeof(state));
		vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, &trackedDevicePose, 1);
		// print positiona data for the HMD.
		poseMatrix = trackedDevicePose.mDeviceToAbsoluteTracking;
		// This matrix contains all positional and rotational data.
		position = GetPosition(trackedDevicePose.mDeviceToAbsoluteTracking);
		quaternion = GetRotation(trackedDevicePose.mDeviceToAbsoluteTracking);

		in_out.qRotation = quaternion;

		in_out.vecPosition[0] = position.v[0];
		in_out.vecPosition[1] = position.v[1];
		in_out.vecPosition[2] = position.v[2];

		//// Process SteamVR device states
		//for (vr::TrackedDeviceIndex_t unDevice = 0; unDevice < vr::k_unMaxTrackedDeviceCount; unDevice++)
		//{
		//    if (!system->IsTrackedDeviceConnected(unDevice))
		//        continue;

		//    vr::VRControllerState_t state;
		//    if (system->GetControllerState(unDevice, &state, sizeof(state)))
		//    {
		//        vr::TrackedDevicePose_t trackedDevicePose;
		//        vr::TrackedDevicePose_t trackedControllerPose;
		//        vr::VRControllerState_t controllerState;
		//        vr::HmdMatrix34_t poseMatrix;
		//        vr::HmdVector3_t position;
		//        vr::HmdQuaternion_t quaternion;
		//        vr::ETrackedDeviceClass trackedDeviceClass = vr::VRSystem()->GetTrackedDeviceClass(unDevice);

		//        switch (trackedDeviceClass) {
		//        case vr::ETrackedDeviceClass::TrackedDeviceClass_HMD:
		//            vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, &trackedDevicePose, 1);
		//            // print positiona data for the HMD.
		//            poseMatrix = trackedDevicePose.mDeviceToAbsoluteTracking; // This matrix contains all positional and rotational data.
		//            position = GetPosition(trackedDevicePose.mDeviceToAbsoluteTracking);
		//            quaternion = GetRotation(trackedDevicePose.mDeviceToAbsoluteTracking);

		//            in_out[0].qRotation = quaternion;

		//            std::cout << glm::vec3(glm::eulerAngles(glm::quat(quaternion.w, quaternion.x, quaternion.y, quaternion.z))).y << std::endl;

		//            in_out[0].vecPosition[0] = position.v[0];
		//            in_out[0].vecPosition[1] = position.v[1];
		//            in_out[0].vecPosition[2] = position.v[2];

		//            break;

		//        case vr::ETrackedDeviceClass::TrackedDeviceClass_GenericTracker:
		//            vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, &trackedDevicePose, 1);
		//            // print positiona data for a general vive tracker.
		//            break;

		//        case vr::ETrackedDeviceClass::TrackedDeviceClass_Controller:
		//            vr::VRSystem()->GetControllerStateWithPose(vr::TrackingUniverseStanding, unDevice, &controllerState,
		//                sizeof(controllerState), &trackedControllerPose);
		//            poseMatrix = trackedControllerPose.mDeviceToAbsoluteTracking; // This matrix contains all positional and rotational data.
		//            position = GetPosition(trackedControllerPose.mDeviceToAbsoluteTracking);
		//            quaternion = GetRotation(trackedControllerPose.mDeviceToAbsoluteTracking);

		//            auto trackedControllerRole = vr::VRSystem()->GetControllerRoleForTrackedDeviceIndex(unDevice);
		//            std::string whichHand = "";
		//            if (trackedControllerRole == vr::TrackedControllerRole_LeftHand)
		//            {
		//                whichHand = "LeftHand";
		//            }
		//            else if (trackedControllerRole == vr::TrackedControllerRole_RightHand)
		//            {
		//                whichHand = "RightHand";
		//            }

		//            switch (trackedControllerRole)
		//            {
		//            case vr::TrackedControllerRole_Invalid:
		//                // invalid
		//                break;

		//            case vr::TrackedControllerRole_LeftHand:
		//                in_out[1].qRotation = quaternion;
		//                in_out[1].vecPosition[0] = position.v[0];
		//                in_out[1].vecPosition[1] = position.v[1];
		//                in_out[1].vecPosition[2] = position.v[2];
		//                break;

		//            case vr::TrackedControllerRole_RightHand:
		//                in_out[2].qRotation = quaternion;
		//                in_out[2].vecPosition[0] = position.v[0];
		//                in_out[2].vecPosition[1] = position.v[1];
		//                in_out[2].vecPosition[2] = position.v[2];
		//                break;
		//            }

		//            break;
		//        }

		//    }
		//}
	}

	void setLineWrapping()
	{
		InferredLabel->SetLineWrap(true);
		InferredLabel->SetRequisition(sf::Vector2f(600.f, 20.f));

		InstructionsLabel->SetLineWrap(true);
		InstructionsLabel->SetRequisition(sf::Vector2f(600.f, 50.f));

		CalibrationSettingsLabel->SetLineWrap(true);
		CalibrationSettingsLabel->SetRequisition(sf::Vector2f(600.f, 20.f));
	}

	void packElementsIntoMainBox()
	{
		//Statuses are at the top
		mainGUIBox->Pack(KinectStatusLabel);
		mainGUIBox->Pack(DriverStatusLabel);
		mainGUIBox->Pack(SteamVRStatusLabel);
		mainGUIBox->Pack(InputEmulatorStatusLabel);

		auto fontSizeBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);
		fontSizeBox->Pack(FontSizeScaleLabel);
		fontSizeBox->Pack(FontSizeScale);
		mainGUIBox->Pack(fontSizeBox);

		auto recBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);
		recBox->Pack(reconKinectButton);
		recBox->Pack(ReconControllersButton);

		mainGUIBox->Pack(recBox);
		mainGUIBox->Pack(TrackerInitButton);

		sfg::Box::Ptr horizontalPSMBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL, 5.f);
		horizontalPSMBox->Pack(StartPSMoveHandler);
		horizontalPSMBox->Pack(StopPSMoveHandler);
		//mainGUIBox->Pack(horizontalPSMBox);

		//setHipScaleBox();
		mainGUIBox->Pack(ShowSkeletonButton);

		modeTitleBox110->Pack(sfg::Label::Create("Calibration Points (Reccomended: 3)"));
		modeTitleBox110->Pack(TDegreeButton);

		mainGUIBox->Pack(modeTitleBox110);
		modeTitleBox110->Show(false);

		mainGUIBox->Pack(TrackersConfigSaveButton);
		TrackersConfigSaveButton->Show(false);

		mainGUIBox->Pack(sfg::Label::Create(""));
		mainGUIBox->Pack(sfg::Label::Create(""));
		mainGUIBox->Pack(sfg::Label::Create(""));

		mainGUIBox->Pack(TrackersCalibButton);
		TrackersCalibButton->Show(false);

		mainGUIBox->Pack(expcalibbutton);
		expcalibbutton->Show(false);

		sfg::Box::Ptr astartbox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);

		astartbox->Pack(AutoStartTrackers);
		astartbox->Pack(AutoStartKinectToVR);
		AutoStartTrackers->Show(false);
		AutoStartKinectToVR->Show(false);

		mainGUIBox->Pack(astartbox);

		//mainGUIBox->Pack(TrackersCalibSButton);
		//TrackersCalibSButton->Show(false);

		//mainGUIBox->Pack(EnableGamepadButton);
		//mainGUIBox->Pack(ReconControllersLabel);
		//mainGUIBox->Pack(ReconControllersButton);

		//mainGUIBox->Pack(KinectRotLabel);
		//mainGUIBox->Pack(KinectRotButton);

		//mainGUIBox->Pack(KinectPosLabel);
		//mainGUIBox->Pack(KinectPosButton);

		//mainGUIBox->Pack(InferredLabel);
		//mainGUIBox->Pack(IgnoreInferredCheckButton);
	}

	void refreshcom()
	{
		wchar_t lpTargetPath[5000];
		comportbox1->Clear();
		comportbox2->Clear();

		for (int i = 0; i < 255; i++) // checking ports from COM0 to COM255
		{
			std::wstring str = L"COM" + std::to_wstring(i); // converting to COM0, COM1, COM2
			DWORD res = QueryDosDevice(str.c_str(), lpTargetPath, 5000);

			// Test the return value and error if any
			if (res != 0) //QueryDosDevice returns zero if it didn't find an object
			{
				comportbox1->AppendItem(str);
				comportbox2->AppendItem(str);
				//std::cout << str << ": " << lpTargetPath << std::endl;
			}
			if (GetLastError() == ERROR_INSUFFICIENT_BUFFER)
			{
			}
		}

		comportbox1->SelectItem(0);
		comportbox2->SelectItem(1);
	}

	void setHipScaleBox()
	{
		auto HipLabel = sfg::Label::Create("Vertical Hip Adjustment (metres)");
		HipScale->SetDigits(3);

		HipScaleBox->Pack(HipLabel, false, false);
		HipScaleBox->Pack(HipScale);
		mainGUIBox->Pack(HipScaleBox);
	}

	void packElementsIntoTrackingMethodBox()
	{
		//trackingMethodBox->Pack(InitiateColorTrackingButton);
		//trackingMethodBox->Pack(DestroyColorTrackingButton);

		sfg::Box::Ptr horbox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL, 5.f);

		horbox->Pack(startControllers);
		horbox->Pack(stopControllers);
		controllersBox->Pack(horbox);

		sfg::Box::Ptr toptbox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL, 5.f);
		toptbox->Pack(sfg::Label::Create("Choose configuration for controllers tracking"));
		toptbox->Pack(contrackingselectbox);

		controllersBox->Pack(toptbox);
		contrackingselectbox->AppendItem("Full PSMove setup");
		contrackingselectbox->AppendItem("PSMove + Kinect tracking");
		contrackingselectbox->SelectItem(VirtualHips::settings.conOption);

		stopControllers->SetState(sfg::Widget::State::INSENSITIVE);
		startControllers->SetState(sfg::Widget::State::NORMAL);

		auto offsets = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL, 5.5f);
		auto xhoffset = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL, 5.5f);
		auto yhoffset = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL, 5.5f);
		auto zhoffset = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL, 5.5f);
		auto xhoffset1 = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL, 5.5f);
		auto yhoffset1 = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL, 5.5f);
		auto zhoffset1 = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL, 5.5f);
		auto controllersid = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL, 5.5f);
		auto hidaricon = sfg::Box::Create(sfg::Box::Orientation::VERTICAL, 5.5f);
		auto migicon = sfg::Box::Create(sfg::Box::Orientation::VERTICAL, 5.5f);
		auto refreshcon = sfg::Box::Create(sfg::Box::Orientation::VERTICAL, 5.5f);

		hidaricon->Pack(sfg::Label::Create("Left Controller ID"));
		hidaricon->Pack(psmovebox);
		migicon->Pack(sfg::Label::Create("Right Controller ID"));
		migicon->Pack(psmovebox1);
		refreshcon->Pack(refreshpsmovesbuton);

		controllersid->Pack(hidaricon);
		controllersid->Pack(migicon);
		controllersid->Pack(refreshcon);
		controllersBox->Pack(controllersid);

		auto xoffset = sfg::Box::Create(sfg::Box::Orientation::VERTICAL);
		xoffset->Pack(sfg::Label::Create("X Offset (meters)"));
		xoffset->Pack(sfg::Label::Create(" "));
		xhoffset->Pack(sfg::Label::Create("Left-Con"));
		xhoffset->Pack(sfg::Label::Create("Right-Con"));
		xhoffset1->Pack(arduhx);
		xhoffset1->Pack(ardumx);
		xoffset->Pack(xhoffset);
		xoffset->Pack(xhoffset1);
		xoffset->Pack(sfg::Label::Create(" "));

		auto yoffset = sfg::Box::Create(sfg::Box::Orientation::VERTICAL);
		yoffset->Pack(sfg::Label::Create("Y Offset (meters)"));
		yoffset->Pack(sfg::Label::Create(" "));
		yhoffset->Pack(sfg::Label::Create("Left-Con"));
		yhoffset->Pack(sfg::Label::Create("Right-Con"));
		yhoffset1->Pack(arduhy);
		yhoffset1->Pack(ardumy);
		yoffset->Pack(yhoffset);
		yoffset->Pack(yhoffset1);
		yoffset->Pack(sfg::Label::Create(" "));

		auto zoffset = sfg::Box::Create(sfg::Box::Orientation::VERTICAL);
		zoffset->Pack(sfg::Label::Create("Z Offset (meters)"));
		zoffset->Pack(sfg::Label::Create(" "));
		zhoffset->Pack(sfg::Label::Create("Left-Con"));
		zhoffset->Pack(sfg::Label::Create("Right-Con"));
		zhoffset1->Pack(arduhz);
		zhoffset1->Pack(ardumz);
		zoffset->Pack(zhoffset);
		zoffset->Pack(zhoffset1);
		zoffset->Pack(sfg::Label::Create(" "));

		offsets->Pack(xoffset);
		offsets->Pack(yoffset);
		offsets->Pack(zoffset);

		controllersBox->Pack(sfg::Label::Create(" "));
		controllersBox->Pack(offsets);
		controllersBox->Pack(AutoStartControllers);
	}

	void updateDeviceLists()
	{
		setDeviceListItems(PositionDeviceList);
		setDeviceListItems(RotationDeviceList);
	}

	void packElementsIntoAdvTrackerBox()
	{
		advancedTrackerBox->Pack(sfg::Label::Create("PSMoveSerive handler"));

		sfg::Box::Ptr horizontalPSMBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL, 5.f);
		horizontalPSMBox->Pack(StartPSMoveHandler);
		horizontalPSMBox->Pack(StopPSMoveHandler);
		advancedTrackerBox->Pack(horizontalPSMBox);

		advancedTrackerBox->Pack(sfg::Label::Create("Body tracking option"));

		sfg::Box::Ptr selectoptionbox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL, 5.f);
		selectoptionbox->Pack(sfg::Label::Create("Tracking option for body trackers"));
		selectoptionbox->Pack(bodytrackingselectbox);
		advancedTrackerBox->Pack(selectoptionbox);

		bodytrackingselectbox->AppendItem("PSMove body tracking");
		bodytrackingselectbox->AppendItem("Kinect body tracking");
		bodytrackingselectbox->SelectItem(VirtualHips::settings.bodyTrackingOption);

		sfg::Box::Ptr psmleftidbox = sfg::Box::Create(sfg::Box::Orientation::VERTICAL, 5.f);
		sfg::Box::Ptr psmrightidbox = sfg::Box::Create(sfg::Box::Orientation::VERTICAL, 5.f);
		sfg::Box::Ptr psmhipsidbox = sfg::Box::Create(sfg::Box::Orientation::VERTICAL, 5.f);

		psmleftidbox->Pack(sfg::Label::Create("Left Foot tracker PSMove ID"));
		psmleftidbox->Pack(psmoveboxhi);

		psmrightidbox->Pack(sfg::Label::Create("Right Foot tracker PSMove ID"));
		psmrightidbox->Pack(psmoveboxmi);

		psmhipsidbox->Pack(sfg::Label::Create("Hips tracker PSMove ID"));
		psmhipsidbox->Pack(psmoveboxyo);

		psmidbox->Pack(psmleftidbox);
		psmidbox->Pack(psmrightidbox);
		psmidbox->Pack(psmhipsidbox);
		psmidbox->Pack(refreshpsmovesbuton1);

		advancedTrackerBox->Pack(psmidbox);

		//bodytrackingselectbox

		advancedTrackerBox->Pack(sfg::Label::Create("Trackers orientation filter"));

		auto box1 = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);
		box1->Pack(sfg::Label::Create("Feet trackers orientation tracking"));
		auto box2 = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);
		box2->Pack(sfg::Label::Create("Waist tracker orientation tracking"));

		box1->Pack(coptbox);
		box2->Pack(coptbox1);

		coptbox->AppendItem("Enable Feet Rotation");
		coptbox->AppendItem("Disable Feet Rotation");
		coptbox->AppendItem("Disable Feet Yaw (+Y)");
		coptbox->AppendItem("Use Head Orientation");

		coptbox1->AppendItem("Enable Waist Rotation");
		coptbox1->AppendItem("Disable Waist Rotation");
		coptbox1->AppendItem("Use Head Orientation");

		advancedTrackerBox->Pack(box1);
		advancedTrackerBox->Pack(box2);

		advancedTrackerBox->Pack(sfg::Label::Create("Positional filtering options"));

		auto box11 = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);
		box11->Pack(sfg::Label::Create("Positional tracking filter"));
		box11->Pack(foptbox);

		/*
		* EKF is really nice if you are planning to stand in place
		* Low pass filter is fot those, who have lit room and kinda bad tracking
		* interpolation predicts kinect results so it makes you faster but
		*    literally teleports trackers to next position, should be connected with lpf
		* "Magic Touch" actually disables filters, but only on k2vr side
		*    idk how it works in normal life, but driver should detect it
		*    and apply it's own filters (driver's source code will stay unpublished
		*    until arduvr is officially announced and promoted)
		*/

		foptbox->AppendItem("Extended Kalman filter - slow and smoothest"); //use ekf in k2vr
		foptbox->AppendItem("Low Pass Optical filter - smooth moves, little delay"); //use lpf in k2vr
		foptbox->AppendItem("Linear Interpolation - very fast, no smoothing"); //use glm::mix in k2vr
		foptbox->AppendItem("No filter - normal results, no smoothing");

		advancedTrackerBox->Pack(box11);
	}

	void packElementsIntoCalibrationBox()
	{
		sfg::Box::Ptr verticalBox = sfg::Box::Create(sfg::Box::Orientation::VERTICAL);

		verticalBox->Pack(
			sfg::Label::Create("This tab allows you to move and rotate trackers to fine tune the calibration values."));

		verticalBox->Pack(
			sfg::Label::Create("This is generally used to slighty adjust position and orientation of trackers."));
		verticalBox->Pack(sfg::Label::Create("\nRotation is in degrees and position is declared in meters.\n "));

		auto horizontalrPosBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);
		horizontalrPosBox->Pack(CalibrationrPosLabel);
		CalibrationEntryrPosX->SetDigits(4);
		horizontalrPosBox->Pack(CalibrationEntryrPosX);
		CalibrationEntryrPosY->SetDigits(4);
		horizontalrPosBox->Pack(CalibrationEntryrPosY);
		CalibrationEntryrPosZ->SetDigits(4);
		horizontalrPosBox->Pack(CalibrationEntryrPosZ);
		verticalBox->Pack(horizontalrPosBox);

		auto horizontalrRotBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);
		horizontalrRotBox->Pack(CalibrationrRotLabel);
		CalibrationEntryrRotX->SetDigits(4);
		horizontalrRotBox->Pack(CalibrationEntryrRotX);
		CalibrationEntryrRotY->SetDigits(4);
		horizontalrRotBox->Pack(CalibrationEntryrRotY);
		CalibrationEntryrRotZ->SetDigits(4);
		horizontalrRotBox->Pack(CalibrationEntryrRotZ);
		verticalBox->Pack(horizontalrRotBox);

		auto horizontallPosBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);
		horizontallPosBox->Pack(CalibrationlPosLabel);
		CalibrationEntrylPosX->SetDigits(4);
		horizontallPosBox->Pack(CalibrationEntrylPosX);
		CalibrationEntrylPosY->SetDigits(4);
		horizontallPosBox->Pack(CalibrationEntrylPosY);
		CalibrationEntrylPosZ->SetDigits(4);
		horizontallPosBox->Pack(CalibrationEntrylPosZ);
		verticalBox->Pack(horizontallPosBox);

		auto horizontallRotBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);
		horizontallRotBox->Pack(CalibrationlRotLabel);
		CalibrationEntrylRotX->SetDigits(4);
		horizontallRotBox->Pack(CalibrationEntrylRotX);
		CalibrationEntrylRotY->SetDigits(4);
		horizontallRotBox->Pack(CalibrationEntrylRotY);
		CalibrationEntrylRotZ->SetDigits(4);
		horizontallRotBox->Pack(CalibrationEntrylRotZ);
		verticalBox->Pack(horizontallRotBox);

		auto horizontalhPosBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);
		horizontalhPosBox->Pack(CalibrationhPosLabel);
		CalibrationEntryhPosX->SetDigits(4);
		horizontalhPosBox->Pack(CalibrationEntryhPosX);
		CalibrationEntryhPosY->SetDigits(4);
		horizontalhPosBox->Pack(CalibrationEntryhPosY);
		CalibrationEntryhPosZ->SetDigits(4);
		horizontalhPosBox->Pack(CalibrationEntryhPosZ);
		verticalBox->Pack(horizontalhPosBox);

		auto horizontalhRotBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);
		horizontalhRotBox->Pack(CalibrationhRotLabel);
		CalibrationEntryhRotX->SetDigits(4);
		horizontalhRotBox->Pack(CalibrationEntryhRotX);
		CalibrationEntryhRotY->SetDigits(4);
		horizontalhRotBox->Pack(CalibrationEntryhRotY);
		CalibrationEntryhRotZ->SetDigits(4);
		horizontalhRotBox->Pack(CalibrationEntryhRotZ);
		verticalBox->Pack(horizontalhRotBox);

		verticalBox->Pack(sfg::Label::Create(" "));

		verticalBox->Pack(CalibrationSaveButton);

		verticalBox->Pack(sfg::Label::Create(" "));

		verticalBox->Pack(ActivateVRSceneTypeButton);

		calibrationBox->Pack(verticalBox);
	}

	void setBonesListItems()
	{
		using namespace KVR;
		for (int i = 0; i < KinectJointCount; ++i)
		{
			BonesList->AppendItem(KinectJointName[i]);
		}
		// Set as default - to prevent garbage additions
		BonesList->SelectItem(0);
	}

	void setDeviceListItems(sfg::ComboBox::Ptr comboBox)
	{
		comboBox->Clear();
		for (int i = 0; i < TrackingPoolManager::count(); ++i)
		{
			if (kinectJointDevicesHiddenFromList && TrackingPoolManager::trackerIdInKinectRange(i))
			{
				continue;
			}
			comboBox->AppendItem(TrackingPoolManager::deviceGuiString(i));
		}
		// Set as default - to prevent garbage additions
		comboBox->SelectItem(0);
	}

	void setRolesListItems(sfg::ComboBox::Ptr comboBox)
	{
		for (int i = 0; i < static_cast<int>(KVR::KinectDeviceRole::Count); ++i)
		{
			comboBox->AppendItem(KVR::KinectDeviceRoleName[i]);
		}
		// Set as default - to prevent garbage additions
		comboBox->SelectItem(0);
	}

	void updateKinectStatusLabel(KinectHandlerBase& kinect)
	{
		HRESULT status = kinect.getStatusResult();
		if (kinect.isInitialised())
		{
			if (status == lastKinectStatus)
				return; // No need to waste time updating it;
			if (kinect.isPSMS)
			{
				KinectStatusLabel->SetText("PSMoveService Mode!");
			}
			else
			{
				switch (status)
				{
				case S_OK:
					{
						KinectStatusLabel->SetText("Kinect Status: Success!");
						break;
					}
				default:
					KinectStatusLabel->SetText("Kinect Status: ERROR " + kinect.statusResultString(status));
					break;
				}
			}
		}
		else
			updateKinectStatusLabelDisconnected();
		if (status != lastKinectStatus)
		{
			LOG(INFO) << "Kinect Status changed to: " << KinectStatusLabel->GetText().toAnsiString();
			lastKinectStatus = status;
		}
	}

	void updateEmuStatusLabelError(vrinputemulator::vrinputemulator_connectionerror e)
	{
		//InputEmulatorStatusLabel->SetText("Input Emu Status: NOT Connected! Error " + std::to_string(e.errorcode) + " " + e.what() + "\n\n Is SteamVR open and InputEmulator installed?");
	}

	void updateEmuStatusLabelSuccess()
	{
		//InputEmulatorStatusLabel->SetText("Input Emu Status: Success!");
	}

	void updateVRStatusLabel(vr::EVRInitError eError)
	{
		if (eError == vr::VRInitError_None)
			SteamVRStatusLabel->SetText("SteamVR Status: Success!");
		else
			SteamVRStatusLabel->SetText(
				"SteamVR Status: ERROR " + std::to_string(eError) +
				"\nPlease restart K2VR with SteamVR successfully running!");
	}

	void setTrackingMethodsReference(std::vector<std::unique_ptr<TrackingMethod>>& ref)
	{
		v_trackingMethodsRef = &ref;
	}

	void setDeviceHandlersReference(std::vector<std::unique_ptr<DeviceHandler>>& ref)
	{
		v_deviceHandlersRef = &ref;
	}

	void updateWithNewWindowSize(sf::Vector2f size)
	{
		guiWindow->SetAllocation(sf::FloatRect(0.f, 0.f, .4f * size.x, .4f * size.y));
		//setScale();
		//guiWindow->SetAllocation(sf::FloatRect(size.x - width, 0.f, width, size.y));
		//mGUI.SideBar->SetAllocation(sf::FloatRect(0.f, 0.f, width, size.y));
	}

	using PointSet = Eigen::Matrix<float, 3, Eigen::Dynamic>;

	auto rigid_transform_3D(const PointSet& A, const PointSet& B) -> std::tuple<Eigen::Matrix3f, Eigen::Vector3f>
	{
		static_assert(PointSet::RowsAtCompileTime == 3);
		assert(A.cols() == B.cols());

		// find mean column wise
		const Eigen::Vector3f centroid_A = A.rowwise().mean();
		const Eigen::Vector3f centroid_B = B.rowwise().mean();

		// subtract mean
		PointSet Am = A.colwise() - centroid_A;
		PointSet Bm = B.colwise() - centroid_B;

		PointSet H = Am * Bm.transpose();

		//
		//# sanity check
		//#if linalg.matrix_rank(H) < 3:
		//	#    raise ValueError("rank of H = {}, expecting 3".format(linalg.matrix_rank(H)))
		//

		// find rotation
		Eigen::JacobiSVD<Eigen::Matrix3Xf> svd = H.jacobiSvd(
			Eigen::DecompositionOptions::ComputeFullU | Eigen::DecompositionOptions::ComputeFullV);
		const Eigen::Matrix3f& U = svd.matrixU();
		Eigen::MatrixXf V = svd.matrixV();
		Eigen::Matrix3f R = V * U.transpose();

		// special reflection case
		if (R.determinant() < 0.0f)
		{
			V.col(2) *= -1.0f;
			R = V * U.transpose();
		}

		const Eigen::Vector3f t = -R * centroid_A + centroid_B;

		return std::make_tuple(R, t);
	}

	void killProcessByName(const char* filename)
	{
		USES_CONVERSION;
		HANDLE hSnapShot = CreateToolhelp32Snapshot(TH32CS_SNAPALL, NULL);
		PROCESSENTRY32 pEntry;
		pEntry.dwSize = sizeof(pEntry);
		BOOL hRes = Process32First(hSnapShot, &pEntry);
		while (hRes)
		{
			if (strcmp(W2A(pEntry.szExeFile), filename) == 0)
			{
				HANDLE hProcess = OpenProcess(PROCESS_TERMINATE, 0,
				                              static_cast<DWORD>(pEntry.th32ProcessID));
				if (hProcess != nullptr)
				{
					TerminateProcess(hProcess, 9);
					CloseHandle(hProcess);
				}
			}
			hRes = Process32Next(hSnapShot, &pEntry);
		}
		CloseHandle(hSnapShot);
	}

	void setVirtualHipsBoxSignals()
	{
		using namespace VirtualHips;

		VirtualHipHeightFromHMDButton->SetDigits(2);
		DegreeButton->SetDigits(2);
		TDegreeButton->SetDigits(0);
		VirtualHipSittingThreshold->SetDigits(2);
		VirtualHipLyingThreshold->SetDigits(2);

		arduhx->SetDigits(2);
		arduhy->SetDigits(2);
		arduhz->SetDigits(2);

		ardumx->SetDigits(2);
		ardumy->SetDigits(2);
		ardumz->SetDigits(2);

		VirtualHipUseHMDYawButton->GetSignal(sfg::ToggleButton::OnToggle).Connect([this]
		{
			settings.followHmdYawRotation = (VirtualHipUseHMDYawButton->IsActive());
		});
		VirtualHipUseHMDPitchButton->GetSignal(sfg::ToggleButton::OnToggle).Connect([this]
		{
			settings.followHmdPitchRotation = (VirtualHipUseHMDPitchButton->IsActive());
		});
		VirtualHipUseHMDRollButton->GetSignal(sfg::ToggleButton::OnToggle).Connect([this]
		{
			settings.followHmdRollRotation = (VirtualHipUseHMDRollButton->IsActive());
		});

		VirtualHipLockToHeadButton->GetSignal(sfg::RadioButton::OnToggle).Connect([this]
			{
				settings.positionAccountsForFootTrackers = !VirtualHipLockToHeadButton->IsActive();
			}
		);
		VirtualHipLockToFeetButton->GetSignal(sfg::RadioButton::OnToggle).Connect([this]
			{
				settings.positionAccountsForFootTrackers = VirtualHipLockToFeetButton->IsActive();
			}
		);

		AutoStartTrackers->GetSignal(sfg::Widget::OnLeftClick).Connect([this]
		{
			settings.astartt = !settings.astartt;
			if (settings.astartt)
			{
				AutoStartTrackers->SetLabel("Initialise trackers automatically CURRENT: YES");
			}
			else
			{
				AutoStartTrackers->SetLabel("Initialise trackers automatically CURRENT: NO");
			}
		});

		AutoStartKinectToVR->GetSignal(sfg::Widget::OnLeftClick).Connect([this]
		{
			settings.astartk = !settings.astartk;
			if (settings.astartk)
			{
				AutoStartKinectToVR->SetLabel("Launch K2 automatically with SteamVR CURRENT: YES");
			}
			else
			{
				AutoStartKinectToVR->SetLabel("Launch K2 automatically with SteamVR CURRENT: NO");
			}
		});

		AutoStartHeadTracking->GetSignal(sfg::Widget::OnLeftClick).Connect([this]
		{
			settings.astarth = !settings.astarth;
			if (settings.astarth)
			{
				AutoStartHeadTracking->SetLabel("Start head tracking on launch CURRENT: YES");
			}
			else
			{
				AutoStartHeadTracking->SetLabel("Start head tracking on launch CURRENT: NO");
			}
		});

		AutoStartControllers->GetSignal(sfg::Widget::OnLeftClick).Connect([this]
		{
			settings.astarta = !settings.astarta;
			if (settings.astarta)
			{
				AutoStartControllers->SetLabel("Start Controllers on launch CURRENT: YES");
			}
			else
			{
				AutoStartControllers->SetLabel("Start Controllers on launch CURRENT: NO");
			}
		});

		startControllers->GetSignal(sfg::Widget::OnLeftClick).Connect([this]
		{
			KinectSettings::conActivated = true;

			std::thread* activate = new std::thread([]
			{
				try
				{
					using namespace boost::asio;
					using ip::tcp;
					using std::string;

					io_service io_service;
					//socket creation
					tcp::socket socket(io_service);
					//connection
					socket.connect(tcp::endpoint(ip::address::from_string("127.0.0.1"), 5741));
					// request/message from client

					const string msg = "Hello from Client!\n";
					boost::system::error_code error;
					boost::asio::write(socket, buffer(msg), error);
					if (!error)
					{
						LOG(INFO) << "Client sent message!";
					}
					else
					{
						LOG(INFO) << "(Ignore) send failed: " << error.message();
					}

					// getting response from server
					streambuf receive_buffer;
					boost::asio::read(socket, receive_buffer, transfer_all(), error);
					if (error && error != error::eof)
					{
						LOG(INFO) << "(Ignore) receive failed: " << error.message();
					}
					else
					{
						const char* data = boost::asio::buffer_cast<const char*>(receive_buffer.data());
					}
				}
				catch (boost::exception const& e)
				{
				}
				catch (std::exception e)
				{
				}
			});

			std::thread* pscon = new std::thread([this]
			{
				while (true)
				{
					vr::HmdVector3d_t mpose = psMoveHandler.getMovePosition(0);
					vr::HmdVector3d_t hpose = psMoveHandler.getMovePosition(1);

					vr::HmdQuaternion_t mrotation = psMoveHandler.getMoveOrientation(0);
					vr::HmdQuaternion_t hrotation = psMoveHandler.getMoveOrientation(1);
				}
			});

			/*ShellExecute(NULL, _T("open"), _T("avr_hhost.exe"), s2ws(comportbox1->GetSelectedText()).c_str(), NULL, SW_HIDE);
			ShellExecute(NULL, _T("open"), _T("avr_mhost.exe"), s2ws(comportbox2->GetSelectedText()).c_str(), NULL, SW_HIDE);*/

			//settings.comph = comportbox1->GetSelectedText();
			//settings.compm = comportbox2->GetSelectedText();

			saveSettings();
			stopControllers->SetState(sfg::Widget::State::NORMAL);
			startControllers->SetState(sfg::Widget::State::INSENSITIVE);
			//comportbox1->SetState(sfg::Widget::State::INSENSITIVE);
			//comportbox2->SetState(sfg::Widget::State::INSENSITIVE);
			//refreshcomports->SetState(sfg::Widget::State::INSENSITIVE);
		});

		stopControllers->GetSignal(sfg::Widget::OnLeftClick).Connect([this]
		{
			//killProcessByName("avr_hhost.exe");
			//killProcessByName("avr_mhost.exe");

			KinectSettings::conActivated = false;

			stopControllers->SetState(sfg::Widget::State::INSENSITIVE);
			startControllers->SetState(sfg::Widget::State::NORMAL);
			//comportbox1->SetState(sfg::Widget::State::NORMAL);
			//comportbox2->SetState(sfg::Widget::State::NORMAL);
			//refreshcomports->SetState(sfg::Widget::State::NORMAL);
		});

		if (settings.astartt)
		{
			AutoStartTrackers->SetLabel("Initialise trackers automatically CURRENT: YES");
		}
		else
		{
			AutoStartTrackers->SetLabel("Initialise trackers automatically CURRENT: NO");
		}
		if (settings.astartk)
		{
			AutoStartKinectToVR->SetLabel("Launch K2 automatically with SteamVR CURRENT: YES");
		}
		else
		{
			AutoStartKinectToVR->SetLabel("Launch K2 automatically with SteamVR CURRENT: NO");
		}
		if (settings.astarth)
		{
			AutoStartHeadTracking->SetLabel("Start head tracking on launch CURRENT: YES");
		}
		else
		{
			AutoStartHeadTracking->SetLabel("Start head tracking on launch CURRENT: NO");
		}
		if (settings.astarta)
		{
			AutoStartControllers->SetLabel("Start Controllers on launch CURRENT: YES");
		}
		else
		{
			AutoStartControllers->SetLabel("Start Controllers on launch CURRENT: NO");
		}

		VirtualHipHeightFromHMDButton->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				settings.heightFromHMD = VirtualHipHeightFromHMDButton->GetValue();
				KinectSettings::huoffsets.v[0] = VirtualHipHeightFromHMDButton->GetValue();
			}
		);

		arduhx->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				settings.hauoffset_s(0) = arduhx->GetValue();
				KinectSettings::hauoffset.v[0] = arduhx->GetValue();
				saveSettings();
			}
		);
		arduhy->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				settings.hauoffset_s(1) = arduhy->GetValue();
				KinectSettings::hauoffset.v[1] = arduhy->GetValue();
				saveSettings();
			}
		);
		arduhz->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				settings.hauoffset_s(2) = arduhz->GetValue();
				KinectSettings::hauoffset.v[2] = arduhz->GetValue();
				saveSettings();
			}
		);

		ardumx->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				settings.mauoffset_s(0) = ardumx->GetValue();
				KinectSettings::mauoffset.v[0] = ardumx->GetValue();
				saveSettings();
			}
		);
		ardumy->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				settings.mauoffset_s(1) = ardumy->GetValue();
				KinectSettings::mauoffset.v[1] = ardumy->GetValue();
				saveSettings();
			}
		);
		ardumz->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				settings.mauoffset_s(2) = ardumz->GetValue();
				KinectSettings::mauoffset.v[2] = ardumz->GetValue();
				saveSettings();
			}
		);

		DegreeButton->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				settings.hmdegree = DegreeButton->GetValue();
				KinectSettings::hroffset = DegreeButton->GetValue();
			}
		);
		TDegreeButton->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
			{
				settings.tdegree = TDegreeButton->GetValue();
				KinectSettings::cpoints = TDegreeButton->GetValue();
			}
		);
		VirtualHipFollowHMDLean->GetSignal(sfg::ToggleButton::OnToggle).Connect([this]
		{
			settings.positionFollowsHMDLean = (VirtualHipFollowHMDLean->IsActive());
		});

		VirtualHipSittingThreshold->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
		{
			settings.sittingMaxHeightThreshold = VirtualHipSittingThreshold->GetValue();
			KinectSettings::huoffsets.v[1] = VirtualHipSittingThreshold->GetValue();
		});

		VirtualHipLyingThreshold->GetSignal(sfg::SpinButton::OnValueChanged).Connect([this]
		{
			settings.lyingMaxHeightThreshold = VirtualHipLyingThreshold->GetValue();
			KinectSettings::huoffsets.v[2] = VirtualHipLyingThreshold->GetValue();
		});

		VirtualHipConfigSaveButton->GetSignal(sfg::Button::OnLeftClick).Connect([this]
			{
				saveSettings();
			}
		);

		TrackersConfigSaveButton->GetSignal(sfg::Button::OnLeftClick).Connect([this]
			{
				saveSettings();
			}
		);

		expcalibbutton->GetSignal(sfg::ToggleButton::OnToggle).Connect([this]
		{
			KinectSettings::expcalib = !KinectSettings::expcalib;
		});

		TrackersCalibButton->GetSignal(sfg::Button::OnLeftClick).Connect([this]
		{
			vr::EVRInitError error;
			vr::IVRSystem* csystem = VR_Init(&error, vr::VRApplication_Background);
			vr::HmdMatrix34_t cposeMatrix;
			vr::HmdVector3_t cposition;
			vr::HmdQuaternion_t cquaternion;
			vr::TrackedDevicePose_t ctrackedControllerPose;
			vr::VRControllerState_t ccontrollerState;
			vr::VRControllerState_t cstate;
			bool calibrationAbort = false;

			if (!KinectSettings::isCalibrating)
			{
				KinectSettings::isCalibrating = true;

				if (!KinectSettings::expcalib)
				{
					std::thread* t1 = new std::thread([this, &calibrationAbort]()
					{
						KinectSettings::rtcalibrated = true;
						KinectSettings::jcalib = true;

						Eigen::AngleAxisd rollAngle(0.f, Eigen::Vector3d::UnitZ());
						Eigen::AngleAxisd yawAngle(0.f, Eigen::Vector3d::UnitY());
						Eigen::AngleAxisd pitchAngle(0.f, Eigen::Vector3d::UnitX());
						Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

						Eigen::Matrix3d rotationMatrix = q.matrix();
						KinectSettings::R_matT = rotationMatrix.cast<float>();

						bool firstTime = true;
						double yawtmp = 0, pitchtmp = 0;
						while (!KinectSettings::isTriggerPressed[0] || !KinectSettings::isTriggerPressed[1])
						{
							std::this_thread::sleep_for(std::chrono::milliseconds(300));
							/******************************************************************************/
							TrackersCalibButton->SetLabel(
								std::string(
									"Adjust position with Thumbsticks (LGrip: Fine adjust, RGrip: Switch to Rotation, Triggers: Confirm)")
								.c_str());
							/******************************************************************************/

							while (!KinectSettings::isGripPressed[0] && (!KinectSettings::isTriggerPressed[0] || !
								KinectSettings::isTriggerPressed[1]))
							{
								if (!KinectSettings::isGripPressed[1])
								{
									KinectSettings::T_matT(0) += VRInput::trackpadpose[1].x * .01f;
									KinectSettings::T_matT(1) += VRInput::trackpadpose[0].y * .01f;
									KinectSettings::T_matT(2) += -VRInput::trackpadpose[1].y * .01f;
								}
								else
								{
									KinectSettings::T_matT(0) += VRInput::trackpadpose[1].x * .001f;
									KinectSettings::T_matT(1) += VRInput::trackpadpose[0].y * .001f;
									KinectSettings::T_matT(2) += -VRInput::trackpadpose[1].y * .001f;
								}

								std::this_thread::sleep_for(std::chrono::milliseconds(5));
								if (!KinectSettings::isCalibrating) {
									calibrationAbort = true;
									break;
								}
							}

							if (firstTime)
								KinectSettings::calorigin = Eigen::Vector3f(
									KinectSettings::mposes[2].v[0], KinectSettings::mposes[2].v[1],
									KinectSettings::mposes[2].v[2]);
							firstTime = false;

							std::this_thread::sleep_for(std::chrono::milliseconds(300));
							/******************************************************************************/
							TrackersCalibButton->SetLabel(
								std::string(
									"Adjust rotation with Thumbsticks (LGrip: Fine adjust, RGrip: Switch to Position, Triggers: Confirm)")
								.c_str());
							/******************************************************************************/

							while (!KinectSettings::isGripPressed[0] && (!KinectSettings::isTriggerPressed[0] || !
								KinectSettings::isTriggerPressed[1]))
							{
								if (!KinectSettings::isGripPressed[1])
								{
									yawtmp += VRInput::trackpadpose[1].x * M_PI / 280.f;
									pitchtmp += VRInput::trackpadpose[0].y * M_PI / 280.f;
								}
								else
								{
									yawtmp += (VRInput::trackpadpose[1].x * M_PI / 280.f) * .1f;
									pitchtmp += (VRInput::trackpadpose[0].y * M_PI / 280.f) * .1f;
								}

								Eigen::AngleAxisd rollAngle(0.f, Eigen::Vector3d::UnitZ());
								Eigen::AngleAxisd yawAngle(yawtmp, Eigen::Vector3d::UnitY());
								Eigen::AngleAxisd pitchAngle(pitchtmp, Eigen::Vector3d::UnitX());
								Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

								Eigen::Matrix3d rotationMatrix = q.matrix();
								KinectSettings::R_matT = rotationMatrix.cast<float>();

								std::this_thread::sleep_for(std::chrono::milliseconds(5));
								KinectSettings::tryaw = glm::degrees(yawtmp);
								if (!KinectSettings::isCalibrating) {
									calibrationAbort = true;
									break;
								}
							}

							if (!KinectSettings::isCalibrating)
							{
								KinectSettings::calorigin = settings.caliborigin;
								KinectSettings::R_matT = settings.rcR_matT;
								KinectSettings::T_matT = settings.rcT_matT;
								KinectSettings::tryaw = settings.tryawst;

								KinectSettings::kinpitch = settings.kinpitchst;
								calibrationAbort = true;
								break;
							}
						}

						std::this_thread::sleep_for(std::chrono::seconds(1));
						
						if (!calibrationAbort)
						{
							settings.caliborigin = KinectSettings::calorigin;
							settings.rcR_matT = KinectSettings::R_matT;
							settings.rcT_matT = KinectSettings::T_matT;
							settings.tryawst = glm::degrees(yawtmp);
							
							KinectSettings::kinpitch = 
								(glm::degrees(eulerAngles(KinectSettings::trackerRoth).x) + glm::degrees(eulerAngles(KinectSettings::trackerRoth).y)) / 2;
							settings.kinpitchst = KinectSettings::kinpitch;
						}

						KinectSettings::rtcalibrated = true;
						settings.rtcalib = true;

						TrackersCalibButton->SetLabel(
							std::string(!KinectSettings::isCalibrating
								            ? "Calibration aborted! Hit me to re-calibrate!"
								            : "Done! Hit me to re-calibrate!").c_str());
						TrackersCalibButton->SetState(sfg::Widget::State::NORMAL);

						saveSettings();
					});
				}
				else
				{
					std::thread* t1 = new std::thread([this, &calibrationAbort]()
					{
						vr::EVRInitError error;
						vr::IVRSystem* system = VR_Init(&error, vr::VRApplication_Background);
						vr::TrackedDevicePose_t trackedDevicePose;
						vr::TrackedDevicePose_t trackedControllerPose;
						vr::VRControllerState_t controllerState;
						vr::HmdMatrix34_t poseMatrix;
						vr::HmdVector3_t position;
						vr::HmdQuaternion_t quaternion;

						std::vector<vr::DriverPose_t> spose;
						std::vector<vr::HmdVector3d_t> hpose;

						KinectSettings::ismatrixcalibrated = false;
						KinectSettings::rtcalibrated = false;
						KinectSettings::calorigin = Eigen::Vector3f(0, 0, 0);
						settings.caliborigin = KinectSettings::calorigin;

						for (int ipoint = 1; ipoint <= KinectSettings::cpoints; ipoint++)
						{
							if (!KinectSettings::isCalibrating) break;
							vr::DriverPose_t ispose;
							vr::HmdVector3d_t ihpose;

							std::this_thread::sleep_for(std::chrono::seconds(3));

							TrackersCalibButton->SetLabel(
								std::string(
									"Get ready to calibrate: Point " + boost::lexical_cast<std::string>(ipoint) + "").
								c_str());
							std::this_thread::sleep_for(std::chrono::seconds(1));
							for (auto i = 3; i >= 0; i--)
							{
								TrackersCalibButton->SetLabel(
									std::string(
										"Point " + boost::lexical_cast<std::string>(ipoint) +
										": Stand somewhere... Time left: " + boost::lexical_cast<std::string>(i) + "s").
									c_str());
								std::this_thread::sleep_for(std::chrono::seconds(1));
								if (!KinectSettings::isCalibrating) break;
							}
							if (!KinectSettings::isCalibrating) break;

							vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(
								vr::TrackingUniverseStanding, 0, &trackedDevicePose, 1);
							position = GetPosition(trackedDevicePose.mDeviceToAbsoluteTracking);

							ispose.vecPosition[0] = position.v[0] - KinectSettings::trackingOriginPosition.v[0];
							ispose.vecPosition[1] = position.v[1] - KinectSettings::trackingOriginPosition.v[1];
							ispose.vecPosition[2] = position.v[2] - KinectSettings::trackingOriginPosition.v[2];

							Eigen::AngleAxisd rollAngle(0.f, Eigen::Vector3d::UnitZ());
							Eigen::AngleAxisd yawAngle(-KinectSettings::svrhmdyaw, Eigen::Vector3d::UnitY());
							Eigen::AngleAxisd pitchAngle(0.f, Eigen::Vector3d::UnitX());

							Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

							Eigen::Vector3d in(ispose.vecPosition[0], ispose.vecPosition[1], ispose.vecPosition[2]);
							Eigen::Vector3d out = q * in;

							ispose.vecPosition[0] = out(0);
							ispose.vecPosition[1] = out(1);
							ispose.vecPosition[2] = out(2);

							for (auto i = 0; i < 3; i++)ihpose.v[i] = KinectSettings::mposes[0].v[i];
							TrackersCalibButton->SetLabel(
								std::string("Position captured: Point " + boost::lexical_cast<std::string>(ipoint) + "")
								.c_str());
							std::this_thread::sleep_for(std::chrono::seconds(2));

							spose.push_back(ispose);
							hpose.push_back(ihpose);
							if (!KinectSettings::isCalibrating) break;
						}
						if (!KinectSettings::isCalibrating)
						{
							KinectSettings::calorigin = settings.caliborigin;
							KinectSettings::R_matT = settings.rcR_matT;
							KinectSettings::T_matT = settings.rcT_matT;
							KinectSettings::tryaw = settings.tryawst;

							KinectSettings::kinpitch = settings.kinpitchst;
							calibrationAbort = true;
						}

						if (!calibrationAbort)
						{
							Eigen::Matrix<float, 3, Eigen::Dynamic> spoints(3, KinectSettings::cpoints), hpoints(
								                                        3, KinectSettings::cpoints);

							for (int ipoint = 0; ipoint < KinectSettings::cpoints; ipoint++)
							{
								spoints(0, ipoint) = spose.at(ipoint).vecPosition[0];
								spoints(1, ipoint) = spose.at(ipoint).vecPosition[1];
								spoints(2, ipoint) = spose.at(ipoint).vecPosition[2];

								hpoints(0, ipoint) = hpose.at(ipoint).v[0];
								hpoints(1, ipoint) = hpose.at(ipoint).v[1];
								hpoints(2, ipoint) = hpose.at(ipoint).v[2];
								if (!KinectSettings::isCalibrating) break;
							}

							PointSet A = hpoints, B = spoints;

							const auto [ret_R, ret_t] = rigid_transform_3D(A, B);

							std::cout << "\nHead points\n" << A << "\nSteamvr points\n" << B << "\nTranslation\n" <<
								ret_t << "\nRotation\n" << ret_R << '\n';

							PointSet B2 = (ret_R * A).colwise() + ret_t;

							PointSet err = B2 - B;
							err = err.cwiseProduct(err);
							const float rmse = std::sqrt(err.sum() / static_cast<float>(3));

							/*if (rmse < 0.01f)
								std::cout << "\nEverything looks good!\n";
							else
								std::cout << "\nHmm something doesn't look right ...\n";*/

							std::cout << "\nOrginal points\n" << B << "\nMy result\n" << B2 << '\n';

							/*Eigen::Matrix<float, 3, 1> xht;
							xht << 0, 0, 3;
							Eigen::Matrix<float, 3, 1> xht2 = (ret_R * xht).colwise() + ret_t;*/

							KinectSettings::R_matT = ret_R;
							KinectSettings::T_matT = ret_t;

							settings.rcR_matT = ret_R;
							settings.rcT_matT = ret_t;
						}

						TrackersCalibButton->SetLabel(
							std::string("Prepare to calibration: Tracker Orientation").c_str());
						std::this_thread::sleep_for(std::chrono::seconds(1));
						for (auto i = 3; i >= 0; i--)
						{
							if (!KinectSettings::isCalibrating) break;
							TrackersCalibButton->SetLabel(
								std::string(
									"Tracker Orientation: Look at Kinect... Time left: " + boost::lexical_cast<
										std::string>(i) + "s").c_str());
							std::this_thread::sleep_for(std::chrono::seconds(1));
						}
						if (!KinectSettings::isCalibrating)
						{
							KinectSettings::calorigin = settings.caliborigin;
							KinectSettings::R_matT = settings.rcR_matT;
							KinectSettings::T_matT = settings.rcT_matT;
							KinectSettings::tryaw = settings.tryawst;

							KinectSettings::kinpitch = settings.kinpitchst;
							calibrationAbort = true;
						}

						if (!calibrationAbort)
						{
							vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(
								vr::TrackingUniverseStanding, 0, &trackedDevicePose, 1);
							quaternion = GetRotation(trackedDevicePose.mDeviceToAbsoluteTracking);
							double yaw = std::atan2(trackedDevicePose.mDeviceToAbsoluteTracking.m[0][2],
							                        trackedDevicePose.mDeviceToAbsoluteTracking.m[2][2]),
							       yawRaw = std::atan2(trackedDevicePose.mDeviceToAbsoluteTracking.m[0][2],
							                           trackedDevicePose.mDeviceToAbsoluteTracking.m[2][2]);

							Eigen::Quaternion q(quaternion.w, quaternion.x, quaternion.y, quaternion.z);

							if (yawRaw < 0.0f)
							{
								yawRaw += 2 * M_PI;
							}
							if (yaw < 0.0)
							{
								yaw = 2 * M_PI + yaw;
							}

							KinectSettings::tryaw = glm::degrees(yaw);
							settings.tryawst = glm::degrees(yaw);

							KinectSettings::kinpitch =
								(glm::degrees(eulerAngles(KinectSettings::trackerRoth).x) + glm::degrees(eulerAngles(KinectSettings::trackerRoth).y)) / 2;
							settings.kinpitchst = KinectSettings::kinpitch;

							KinectSettings::calorigin = Eigen::Vector3f(0, 0, 0);
							settings.caliborigin = KinectSettings::calorigin;
						}

						KinectSettings::rtcalibrated = true;
						settings.rtcalib = true;

						TrackersCalibButton->SetLabel(
							std::string(calibrationAbort
								            ? "Calibration aborted! Hit me to re-calibrate!"
								            : "Done! Hit me to re-calibrate!").c_str());
						TrackersCalibButton->SetState(sfg::Widget::State::NORMAL);

						saveSettings();
					});
				}
			}
			else
			{
				KinectSettings::isCalibrating = false;
				calibrationAbort = true;
			}

			saveSettings();
		});

		TrackersCalibSButton->GetSignal(sfg::Button::OnLeftClick).Connect([this]
			{
				KinectSettings::headtracked = true;

				saveSettings();
			}
		);
	}

	void loadVirtualHipSettingsIntoGUIElements()
	{
		// Retrieve the values from config
		using namespace VirtualHips;
		retrieveSettings();

		VirtualHipUseHMDYawButton->SetActive(settings.followHmdYawRotation);
		VirtualHipUseHMDPitchButton->SetActive(settings.followHmdPitchRotation);
		VirtualHipUseHMDRollButton->SetActive(settings.followHmdRollRotation);

		VirtualHipLockToHeadButton->SetActive(!settings.positionAccountsForFootTrackers);

		VirtualHipHeightFromHMDButton->SetValue(settings.heightFromHMD);
		DegreeButton->SetValue(settings.hmdegree);
		TDegreeButton->SetValue(settings.tdegree);

		arduhx->SetValue(settings.hauoffset_s(0));
		arduhy->SetValue(settings.hauoffset_s(1));
		arduhz->SetValue(settings.hauoffset_s(2));

		ardumx->SetValue(settings.mauoffset_s(0));
		ardumy->SetValue(settings.mauoffset_s(1));
		ardumz->SetValue(settings.mauoffset_s(2));

		VirtualHipFollowHMDLean->SetActive(settings.positionFollowsHMDLean);

		VirtualHipSittingThreshold->SetValue(settings.sittingMaxHeightThreshold);
		VirtualHipLyingThreshold->SetValue(settings.lyingMaxHeightThreshold);
	}

	// Virtual Hips Menu
	void packElementsIntoVirtualHipsBox()
	{
		loadVirtualHipSettingsIntoGUIElements();
		setVirtualHipsBoxSignals();

		virtualHipsBox->SetSpacing(0.5f);

		auto modeTitleBox1 = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);
		modeTitleBox1->Pack(sfg::Label::Create("Offsets are working only when Head Tracking is activated"));

		virtualHipsBox->Pack(modeTitleBox1);

		sfg::Box::Ptr selectoptionbox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL, 5.f);
		selectoptionbox->Pack(sfg::Label::Create("Choose tracking option for HMD (Position only)"));
		selectoptionbox->Pack(headtrackingselectbox);
		virtualHipsBox->Pack(selectoptionbox);

		headtrackingselectbox->AppendItem("PSMove head tracking");
		headtrackingselectbox->AppendItem("Kinect head tracking");
		headtrackingselectbox->SelectItem(VirtualHips::settings.headTrackingOption);

		psmidbox1->Pack(sfg::Label::Create("Head PSMove ID"));
		psmidbox1->Pack(psmoveboxa);
		psmidbox1->Pack(refreshpsmovesbuton11);

		virtualHipsBox->Pack(psmidbox1);

		auto modeTitleBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);
		auto standingBox = sfg::Box::Create(sfg::Box::Orientation::VERTICAL);
		modeTitleBox->Pack(sfg::Label::Create("X Offset (meters)"));
		standingBox->Pack(VirtualHipHeightFromHMDButton);
		standingBox->Pack(sfg::Label::Create(" "));

		auto sittingBox = sfg::Box::Create(sfg::Box::Orientation::VERTICAL);
		modeTitleBox->Pack(sfg::Label::Create("Y Offsets (meters)"));
		sittingBox->Pack(VirtualHipSittingThreshold);
		sittingBox->Pack(sfg::Label::Create(" "));

		auto lyingBox = sfg::Box::Create(sfg::Box::Orientation::VERTICAL);
		modeTitleBox->Pack(sfg::Label::Create("Z Offset (meters)"));
		lyingBox->Pack(VirtualHipLyingThreshold);
		lyingBox->Pack(sfg::Label::Create(" "));

		auto modeTitleBox10 = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);
		auto standingBox10 = sfg::Box::Create(sfg::Box::Orientation::VERTICAL);
		modeTitleBox10->Pack(sfg::Label::Create("World Offset (degrees)"));
		modeTitleBox10->Pack(DegreeButton);

		auto modeBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);
		modeBox->Pack(standingBox);
		modeBox->Pack(sittingBox);
		modeBox->Pack(lyingBox);

		virtualHipsBox->Pack(modeTitleBox);
		virtualHipsBox->Pack(modeBox);

		virtualHipsBox->Pack(modeTitleBox10);

		virtualHipsBox->Pack(sfg::Label::Create(" "));
		virtualHipsBox->Pack(VirtualHipConfigSaveButton);
		virtualHipsBox->Pack(sfg::Label::Create(""));
		virtualHipsBox->Pack(sfg::Label::Create(""));
		virtualHipsBox->Pack(HeadTrackingStartButton);
		virtualHipsBox->Pack(sfg::Label::Create(""));
		virtualHipsBox->Pack(sfg::Label::Create(""));

		auto nicebox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);

		HeadTrackingCalibButton->Show(false);
		nicebox->Pack(HeadTrackingCalibButton);

		nicebox->Pack(AutoStartHeadTracking);
		AutoStartHeadTracking->Show(false);

		virtualHipsBox->Pack(nicebox);
	}

private:
	sf::Font mainGUIFont;
	sfg::SFGUI sfguiRef;
	sfg::Window::Ptr guiWindow = sfg::Window::Create();
	sfg::Notebook::Ptr mainNotebook = sfg::Notebook::Create();

	std::vector<std::unique_ptr<DeviceHandler>>* v_deviceHandlersRef;
	std::vector<std::unique_ptr<TrackingMethod>>* v_trackingMethodsRef;

	// All the device handlers
	PSMoveHandler psMoveHandler;

	HRESULT lastKinectStatus = E_FAIL;

	sfg::Desktop guiDesktop;

	sfg::Box::Ptr mainGUIBox = sfg::Box::Create(sfg::Box::Orientation::VERTICAL, 5.f);
	sfg::Box::Ptr calibrationBox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL, 5.f);
	sfg::Box::Ptr advancedTrackerBox = sfg::Box::Create(sfg::Box::Orientation::VERTICAL, 5.f);
	sfg::Box::Ptr controllersBox = sfg::Box::Create(sfg::Box::Orientation::VERTICAL, 5.f);
	sfg::Box::Ptr virtualHipsBox = sfg::Box::Create(sfg::Box::Orientation::VERTICAL, 5.f);

	sfg::ComboBox::Ptr comportbox1 = sfg::ComboBox::Create();
	sfg::ComboBox::Ptr comportbox2 = sfg::ComboBox::Create();

	sfg::Adjustment::Ptr fontSizeAdjustment = sfg::Adjustment::Create();
	sfg::Label::Ptr FontSizeScaleLabel = sfg::Label::Create("(WARNING, LAGS ON CHANGE) Font Size: ");
	sfg::SpinButton::Ptr FontSizeScale = sfg::SpinButton::Create(
		sfg::Adjustment::Create(SFMLsettings::globalFontSize, 5.f, 100.f, .5f));
	float lastFontSizeValue = SFMLsettings::globalFontSize;

	sfg::Box::Ptr modeTitleBox110 = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL);

	//Statuses
	sfg::Label::Ptr KinectStatusLabel = sfg::Label::Create();
	sfg::Label::Ptr SteamVRStatusLabel = sfg::Label::Create();
	sfg::Label::Ptr InputEmulatorStatusLabel = sfg::Label::Create();

	sfg::Button::Ptr reconKinectButton = sfg::Button::Create("Reconnect Kinect");
	sfg::Button::Ptr refreshpsmovesbuton = sfg::Button::Create("Refresh");
	sfg::Button::Ptr refreshpsmovesbuton1 = sfg::Button::Create("Refresh");
	sfg::Button::Ptr refreshpsmovesbuton11 = sfg::Button::Create("Refresh");

	sfg::Button::Ptr TrackerLastInitButton = sfg::Button::Create(
		"**Please be in VR before hitting me!** Spawn same trackers as last session");

	sfg::Button::Ptr ShowSkeletonButton = sfg::CheckButton::Create("Show/Hide Skeleton Tracking");
	sfg::Button::Ptr AutoStartTrackers = sfg::Button::Create("Initialise trackers automatically");
	sfg::Button::Ptr AutoStartHeadTracking = sfg::Button::Create("Start head tracking on launch");
	sfg::Button::Ptr AutoStartKinectToVR = sfg::Button::Create("Launch K2 automatically with SteamVR");
	sfg::Button::Ptr AutoStartControllers = sfg::Button::Create("Start Controllers with SteamVR");
	sfg::Box::Ptr horcombox = sfg::Box::Create(sfg::Box::Orientation::HORIZONTAL, 5.f);

	sfg::Button::Ptr refreshcomports = sfg::Button::Create("Refresh");
	sfg::Button::Ptr startControllers = sfg::Button::Create("Re/Initialise Controllers controllers");
	sfg::Button::Ptr stopControllers = sfg::Button::Create("Stop Polling input for controllers");

	//Zeroing
	sfg::Label::Ptr KinectRotLabel = sfg::Label::Create(
		"Calibrate the rotation of the Kinect sensor with the controller thumbsticks. Press the trigger to confirm.");
	sfg::CheckButton::Ptr KinectRotButton = sfg::CheckButton::Create("Enable Kinect Rotation Calibration");

	//Position Adjust
	sfg::Label::Ptr KinectPosLabel = sfg::Label::Create(
		"Calibrate the position of the Kinect sensor with the controller thumbsticks. Press the trigger to confirm.");
	sfg::CheckButton::Ptr KinectPosButton = sfg::CheckButton::Create("Enable Kinect Position Calibration");

	// Controllers
	sfg::CheckButton::Ptr EnableGamepadButton = sfg::CheckButton::Create("Enable Gamepad Calibration Controls");
	sfg::Label::Ptr ReconControllersLabel = sfg::Label::Create(
		"If controller input isn't working, press this to reconnect them.\n Make sure both are on, and not in standby.");
	sfg::Button::Ptr ReconControllersButton = sfg::Button::Create("Reconnect VR Controllers");

	sfg::Label::Ptr InferredLabel = sfg::Label::Create(
		"Checking this stops the trackers if it's not absolutely 100% sure where they are. Leaving this disabled may cause better tracking in poorly lit environments, but at the cost of slight jerks aside sometimes.");
	sfg::CheckButton::Ptr IgnoreInferredCheckButton = sfg::CheckButton::Create("Disable Raw Positional Tracking");

	sfg::Button::Ptr SetJointsToFootRotationButton = sfg::Button::Create("Enable (buggy) foot rotation for 360 Kinect");
	sfg::Button::Ptr SetJointsToAnkleRotationButton = sfg::Button::Create(
		"Disable (buggy) foot rotation for 360 Kinect");

	sfg::Button::Ptr SetAllJointsRotUnfiltered = sfg::Button::Create(
		"Disable rotation smoothing for ALL joints (Rotation smoothing is in development!!!)");
	sfg::Button::Ptr SetAllJointsRotFiltered = sfg::Button::Create(
		"Enable rotation smoothing for ALL joints (Rotation smoothing is in development!!!)");
	sfg::Button::Ptr SetAllJointsRotHead = sfg::Button::Create(
		"Use Head orientation for ALL joints - may fix issues with jumping trackers at cost of limited rotation");

	sfg::Label::Ptr InstructionsLabel = sfg::Label::Create(
		"Stand in front of the Kinect sensor.\n If the trackers don't update, then try crouching slightly until they move.\n\n Calibration: The arrow represents the position and rotation of the Kinect - match it as closely to real life as possible for the trackers to line up.\n\n The arrow pos/rot is set with the thumbsticks on the controllers, and confirmed with the trigger.");
	//Blegh - There has to be a better way than this, maybe serialization?

	sfg::Label::Ptr CalibrationSettingsLabel = sfg::Label::Create(
		"This tab allows you to (manually) offset every one of your trackers. It will aeffect real calibration values, but it will not change them directly.\nYou may need it for example when your right foot will be upper than left, etc.\nRotation is in degrees and position is declared in meters.");
	sfg::Label::Ptr CalibrationrPosLabel = sfg::Label::Create("Right Foot Position x, y, z");
	sfg::SpinButton::Ptr CalibrationEntryrPosX = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::moffsets[0][0].v[0], -1000.f, 1000.f, .01f, .2f));
	sfg::SpinButton::Ptr CalibrationEntryrPosY = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::moffsets[0][0].v[1], -1000.f, 1000.f, .01f, .2f));
	sfg::SpinButton::Ptr CalibrationEntryrPosZ = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::moffsets[0][0].v[2], -1000.f, 1000.f, .01f, .2f));

	sfg::Label::Ptr CalibrationrRotLabel = sfg::Label::Create("Right Foot Rotation x, y, z");
	sfg::SpinButton::Ptr CalibrationEntryrRotX = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::moffsets[1][0].v[0], -360.f, 360.f, .01f, .2f));
	sfg::SpinButton::Ptr CalibrationEntryrRotY = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::moffsets[1][0].v[1], -360.f, 360.f, .01f, .2f));
	sfg::SpinButton::Ptr CalibrationEntryrRotZ = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::moffsets[1][0].v[2], -360.f, 360.f, .01f, .2f));

	sfg::Label::Ptr CalibrationlPosLabel = sfg::Label::Create("Left Foot Position x, y, z ");
	sfg::SpinButton::Ptr CalibrationEntrylPosX = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::moffsets[0][1].v[0], -1000.f, 1000.f, .01f, .2f));
	sfg::SpinButton::Ptr CalibrationEntrylPosY = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::moffsets[0][1].v[1], -1000.f, 1000.f, .01f, .2f));
	sfg::SpinButton::Ptr CalibrationEntrylPosZ = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::moffsets[0][1].v[2], -1000.f, 1000.f, .01f, .2f));

	sfg::Label::Ptr CalibrationlRotLabel = sfg::Label::Create("Left Foot Rotation x, y, z ");
	sfg::SpinButton::Ptr CalibrationEntrylRotX = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::moffsets[1][1].v[0], -360.f, 360.f, .01f, .2f));
	sfg::SpinButton::Ptr CalibrationEntrylRotY = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::moffsets[1][1].v[1], -360.f, 360.f, .01f, .2f));
	sfg::SpinButton::Ptr CalibrationEntrylRotZ = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::moffsets[1][1].v[2], -360.f, 360.f, .01f, .2f));

	sfg::Label::Ptr CalibrationhPosLabel = sfg::Label::Create("Hips Position x, y, z       ");
	sfg::SpinButton::Ptr CalibrationEntryhPosX = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::moffsets[0][2].v[0], -1000.f, 1000.f, .01f, .2f));
	sfg::SpinButton::Ptr CalibrationEntryhPosY = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::moffsets[0][2].v[1], -1000.f, 1000.f, .01f, .2f));
	sfg::SpinButton::Ptr CalibrationEntryhPosZ = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::moffsets[0][2].v[2], -1000.f, 1000.f, .01f, .2f));

	sfg::Label::Ptr CalibrationhRotLabel = sfg::Label::Create("Hips Rotation x, y, z       ");
	sfg::SpinButton::Ptr CalibrationEntryhRotX = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::moffsets[1][2].v[0], -360.f, 360.f, .01f, .2f));
	sfg::SpinButton::Ptr CalibrationEntryhRotY = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::moffsets[1][2].v[1], -360.f, 360.f, .01f, .2f));
	sfg::SpinButton::Ptr CalibrationEntryhRotZ = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::moffsets[1][2].v[2], -360.f, 360.f, .01f, .2f));

	sfg::Button::Ptr CalibrationSaveButton = sfg::Button::Create("Save Calibration Values");

	sfg::Button::Ptr ActivateVRSceneTypeButton = sfg::Button::Create("Show K2VR in the VR Bindings Menu!");

	//Adv Trackers
	sfg::Button::Ptr calibrateOffsetButton = sfg::Button::Create("Calibrate VR Offset");
	sfg::Button::Ptr AddHandControllersToList = sfg::Button::Create("Add Hand Controllers");
	sfg::Button::Ptr AddLowerTrackersToList = sfg::Button::Create("Add Lower Body Trackers");

	bool userSelectedDeviceRotIndex = false;
	bool userSelectedDevicePosIndex = false;
	sfg::Box::Ptr TrackerList = sfg::Box::Create(sfg::Box::Orientation::VERTICAL, 5);
	sfg::Label::Ptr TrackerListLabel = sfg::Label::Create("Trackers to be spawned:");

	sfg::Box::Ptr TrackerListOptionsBox = sfg::Box::Create(sfg::Box::Orientation::VERTICAL, 5);
	sfg::SpinButton::Ptr HipScale = sfg::SpinButton::Create(
		sfg::Adjustment::Create(KinectSettings::hipRoleHeightAdjust, -1.f, 1.f, .01f));
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
	sfg::Label::Ptr TrackingMethodLabel = sfg::Label::Create(
		"Click the corresponding button for the devices you wish to use, and K2VR will try its best to connect to them. (Go to the 'Adv. Trackers' tab once these are connected.");

	sfg::Button::Ptr StartPSMoveHandler = sfg::Button::Create("Run PS Move Handler");
	sfg::Button::Ptr StopPSMoveHandler = sfg::Button::Create("Stop PS Move Handler");
	sfg::Label::Ptr PSMoveHandlerLabel = sfg::Label::Create("Status: Off");

	// Virtual Hips Box
	sfg::CheckButton::Ptr VirtualHipUseHMDYawButton = sfg::CheckButton::Create("Yaw");
	sfg::CheckButton::Ptr VirtualHipUseHMDPitchButton = sfg::CheckButton::Create("Pitch");
	sfg::CheckButton::Ptr VirtualHipUseHMDRollButton = sfg::CheckButton::Create("Roll");

	sfg::RadioButton::Ptr VirtualHipLockToHeadButton = sfg::RadioButton::Create("Head");
	sfg::RadioButton::Ptr VirtualHipLockToFeetButton = sfg::RadioButton::Create("Feet");

	sfg::SpinButton::Ptr VirtualHipHeightFromHMDButton = sfg::SpinButton::Create(
		sfg::Adjustment::Create(VirtualHips::settings.heightFromHMD, -1000.f, 1000.f, 0.01f));
	sfg::CheckButton::Ptr VirtualHipFollowHMDLean = sfg::CheckButton::Create("Follow HMD Lean");

	sfg::SpinButton::Ptr arduhx = sfg::SpinButton::Create(
		sfg::Adjustment::Create(VirtualHips::settings.heightFromHMD, -1000.f, 1000.f, 0.01f));
	sfg::SpinButton::Ptr arduhy = sfg::SpinButton::Create(
		sfg::Adjustment::Create(VirtualHips::settings.heightFromHMD, -1000.f, 1000.f, 0.01f));
	sfg::SpinButton::Ptr arduhz = sfg::SpinButton::Create(
		sfg::Adjustment::Create(VirtualHips::settings.heightFromHMD, -1000.f, 1000.f, 0.01f));

	sfg::SpinButton::Ptr ardumx = sfg::SpinButton::Create(
		sfg::Adjustment::Create(VirtualHips::settings.heightFromHMD, -1000.f, 1000.f, 0.01f));
	sfg::SpinButton::Ptr ardumy = sfg::SpinButton::Create(
		sfg::Adjustment::Create(VirtualHips::settings.heightFromHMD, -1000.f, 1000.f, 0.01f));
	sfg::SpinButton::Ptr ardumz = sfg::SpinButton::Create(
		sfg::Adjustment::Create(VirtualHips::settings.heightFromHMD, -1000.f, 1000.f, 0.01f));

	sfg::SpinButton::Ptr DegreeButton = sfg::SpinButton::Create(
		sfg::Adjustment::Create(VirtualHips::settings.heightFromHMD, -360.f, 360.f, 0.01f));
	sfg::SpinButton::Ptr TDegreeButton = sfg::SpinButton::Create(
		sfg::Adjustment::Create(VirtualHips::settings.heightFromHMD, 2, 11, 1.f));

	sfg::SpinButton::Ptr VirtualHipSittingThreshold = sfg::SpinButton::Create(
		sfg::Adjustment::Create(VirtualHips::settings.sittingMaxHeightThreshold, -1000.f, 1000.f, 0.01f));

	sfg::SpinButton::Ptr VirtualHipLyingThreshold = sfg::SpinButton::Create(
		sfg::Adjustment::Create(VirtualHips::settings.lyingMaxHeightThreshold, -1000.f, 1000.f, 0.01f));

	sfg::Button::Ptr VirtualHipConfigSaveButton = sfg::Button::Create("Save Settings");
	sfg::Button::Ptr TrackersConfigSaveButton = sfg::Button::Create("Save Settings");

	sfg::Button::Ptr HeadTrackingStartButton = sfg::Button::Create("Start Head Tracking");
	sfg::Button::Ptr HeadTrackingCalibButton = sfg::Button::Create("Calibration: Look at Kinect and stand still");
	sfg::Button::Ptr TrackersCalibSButton = sfg::Button::Create("Begin Calibration");
	sfg::Button::Ptr TrackersCalibButton = sfg::Button::Create("Begin Calibration");
	sfg::CheckButton::Ptr expcalibbutton = sfg::CheckButton::Create("Enable Automatic Calibration");

	void updateKinectStatusLabelDisconnected()
	{
		KinectStatusLabel->SetText("Kinect Status: ERROR KINECT NOT DETECTED");
	}

	void showPostTrackerInitUI(bool show = true)
	{
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
		calibrateOffsetButton->Show(show);

		//calibrationBox->Show(show);
	}

	void hidePostTrackerInitUI()
	{
		showPostTrackerInitUI(false);
	}
};
