#include <boost/asio.hpp>
#include "stdafx.h"
#include "KinectToVR.h"
#include "VRHelper.h"

#include "KinectSettings.h"
#include "VRController.h"
#include "VRHelper.h"
#include "GamepadController.h"
#include "GUIHandler.h"
#include "ManualCalibrator.h"
#include "HeadAndHandsAutoCalibrator.h"
#include "TrackingMethod.h"
#include "ColorTracker.h"
#include "SkeletonTracker.h"
#include "IMU_PositionMethod.h"
#include "IMU_RotationMethod.h"
#include "VRDeviceHandler.h"
#include "PSMoveHandler.h"
#include "DeviceHandler.h"
#include "TrackingPoolManager.h"
#include <boost/thread.hpp>
#include <SFML/Audio.hpp>

#include <locale>
#include <codecvt>
#include <iostream>
#include <string>
#include <thread>
//GUI
#include <SFGUI/SFGUI.hpp>
#include <SFGUI/Widgets.hpp>

//OpenCV
#include <opencv2/opencv.hpp>

// Windows last because of the great and holy Microsoft
// ... and their ability to cause compiler errors with macros
#include "wtypes.h"

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

#pragma comment(lib, "Ws2_32.lib")

using namespace KVR;

std::string log_get_timestamp_prefix()
{
	// From PSMoveService ServerLog.cpp
	auto now = std::chrono::system_clock::now();
	auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);
	auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now - seconds);
	time_t in_time_t = std::chrono::system_clock::to_time_t(now);

	std::stringstream ss;
	ss << "[" << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %H:%M:%S") << "." << milliseconds.count() << "]: ";

	return ss.str();
}

void processKeyEvents(sf::Event event)
{
	switch (event.key.code)
	{
	case sf::Keyboard::A:
		toggle(KinectSettings::isKinectDrawn);
		break;
	default:
		break;
	}
}

void toggle(bool& b)
{
	b = !b;
}

// Get the horizontal and vertical screen sizes in pixel
//  https://stackoverflow.com/questions/8690619/how-to-get-screen-resolution-in-c
void getDesktopResolution(int& horizontal, int& vertical)
{
	RECT desktop;
	// Get a handle to the desktop window
	const HWND hDesktop = GetDesktopWindow();
	// Get the size of screen to the variable desktop
	GetWindowRect(hDesktop, &desktop);
	// The top left corner will have coordinates (0,0)
	// and the bottom right corner will have coordinates
	// (horizontal, vertical)
	horizontal = desktop.right;
	vertical = desktop.bottom;
}

sf::VideoMode getScaledWindowResolution()
{
	int h;
	int v;
	getDesktopResolution(h, v);

	sf::VideoMode mode = sf::VideoMode(SFMLsettings::windowScale * static_cast<float>(h),
	                                   SFMLsettings::windowScale * static_cast<float>(v));
	//std::cerr << "desktop: " << h << ", " << v << '\n';
	//std::cerr << "scaled: " << mode.width << ", " << mode.height << '\n';
	return mode;
}

void updateKinectWindowRes(const sf::RenderWindow& window)
{
	SFMLsettings::m_window_width = window.getSize().x;
	SFMLsettings::m_window_height = window.getSize().y;
	LOG(INFO) << "Stored window size: " << SFMLsettings::m_window_width << " x " << SFMLsettings::m_window_height;
	//std::cerr << "w: " << SFMLsettings::m_window_width << " h: " << SFMLsettings::m_window_height << "\n";
}

bool filePathIsNonASCII(const std::wstring& filePath)
{
	for (auto c : filePath)
	{
		if (static_cast<unsigned char>(c) > 127)
		{
			return true;
		}
	}
	return false;
}

void verifyDefaultFilePath()
{
	// Warn about non-english file path, as openvr can only take ASCII chars
	// If this isn't checked, unfortunately, most of the bindings won't load
	// Unless OpenVR's C API adds support for non-english filepaths, K2VR can't either
	bool filePathInvalid = filePathIsNonASCII(SFMLsettings::fileDirectoryPath);
	if (filePathInvalid)
	{
		LOG(ERROR) << "K2VR File Path NONASCII (Invalid)!";
		auto message = L"WARNING: NON-ENGLISH FILEPATH DETECTED: "
			+ SFMLsettings::fileDirectoryPath
			+ L"\n It's possible that OpenVR bindings won't work correctly"
			+ L"\n Please move the K2VR directory to a location with ASCII only"
			+ L"\n e.g. C:/KinectToVR will be fine";
		auto result = MessageBox(nullptr, message.c_str(), L"WARNING!!!", MB_ABORTRETRYIGNORE + MB_ICONWARNING);
		if (result = IDABORT)
		{
			SFMLsettings::keepRunning = false;
		}
	}
	else
		LOG(INFO) << "K2VR File Path ASCII (Valid)";
}

void updateFilePath()
{
	HMODULE module = GetModuleHandleW(nullptr);
	WCHAR exeFilePath[MAX_PATH];
	GetModuleFileNameW(module, exeFilePath, MAX_PATH);

	//Get rid of exe from name
	WCHAR directory[MAX_PATH];
	WCHAR drive[_MAX_DRIVE];
	WCHAR dir[_MAX_DIR];
	WCHAR fname[_MAX_FNAME];
	WCHAR ext[_MAX_EXT];
	_wsplitpath_s(exeFilePath, drive, _MAX_DRIVE, dir, _MAX_DIR, fname,
	              _MAX_FNAME, ext, _MAX_EXT);

	WCHAR filename[_MAX_FNAME]{};
	WCHAR extension[_MAX_EXT]{};
	WCHAR directoryFilePath[MAX_PATH];
	_wmakepath_s(directoryFilePath, _MAX_PATH, drive, dir, filename, extension);
	std::wstring filePathString(directoryFilePath);
	SFMLsettings::fileDirectoryPath = filePathString;

	LOG(INFO) << "File Directory Path Set to " << filePathString;
}

void attemptInitialiseDebugDisplay(sf::Font& font, sf::Text& debugText)
{
	// Global Debug Font
#if _DEBUG
	auto fontFileName = "arial.ttf";
	LOG(DEBUG) << "Attemping Debug Font Load: " << fontFileName << '\n';
	font.loadFromFile(fontFileName);
	debugText.setFont(font);
#endif
	debugText.setString("");
	debugText.setCharacterSize(40);
	debugText.setFillColor(sf::Color::Red);

	debugText.setString(SFMLsettings::debugDisplayTextStream.str());
}

vr::HmdQuaternion_t kinectQuaternionFromRads()
{
	return vrmath::quaternionFromYawPitchRoll(KinectSettings::kinectRadRotation.v[1],
	                                          KinectSettings::kinectRadRotation.v[0],
	                                          KinectSettings::kinectRadRotation.v[2]);
}

void updateTrackerInitGuiSignals(GUIHandler& guiRef, std::vector<KinectTrackedDevice>& v_trackers,
                                 vr::IVRSystem*& m_VRsystem)
{
	if constexpr (true)
	{
		guiRef.setTrackerButtonSignals(v_trackers, m_VRsystem);
		guiRef.updateEmuStatusLabelSuccess();
	}
	else
	{
		guiRef.updateTrackerInitButtonLabelFail();
	}
}

void limitVRFramerate(double& endFrameMilliseconds)
{
	static unsigned int FPS = 90;

	double deltaDeviationMilliseconds;
	int maxMillisecondsToCompensate = 30;

	deltaDeviationMilliseconds = 1000.0 / FPS - endFrameMilliseconds;

	if (floor(deltaDeviationMilliseconds) > 0) // TODO: Handle -ve (slow) frames
		Sleep(deltaDeviationMilliseconds);
	if (deltaDeviationMilliseconds < -maxMillisecondsToCompensate)
	{
		endFrameMilliseconds -= maxMillisecondsToCompensate;
	}
	else
	{
		endFrameMilliseconds += deltaDeviationMilliseconds;
	}
}

void updatenormaltrackers()
{
}

void processLoop(KinectHandlerBase& kinect)
{
	LOG(INFO) << "~~~New logging session for main process begins here!~~~";
	LOG(INFO) << "Kinect version is V" << static_cast<int>(kinect.kVersion);
	updateFilePath();
	//sf::RenderWindow renderWindow(getScaledWindowResolution(), "KinectToVR: " + KinectSettings::KVRversion, sf::Style::Titlebar | sf::Style::Close);
	sf::RenderWindow renderWindow(sf::VideoMode(1280, 768, 32), "KinectToVR: " + KinectSettings::KVRversion,
	                              sf::Style::Titlebar | sf::Style::Close | sf::Style::Resize);
	auto mGUIView = sf::View(renderWindow.getDefaultView());
	auto mGridView = sf::View(sf::FloatRect(0, 0, 1280, 768));

	updateKinectWindowRes(renderWindow);
	int windowFrameLimit = 90;
	renderWindow.setFramerateLimit(windowFrameLimit);
	//Prevents ridiculous overupdating and high CPU usage - plus 90Hz is the recommended refresh rate for most VR panels 
	//renderWindow.setVerticalSyncEnabled(true);

	sf::Clock frameClock;
	sf::Clock timingClock;

	sf::Time time_lastKinectStatusUpdate = timingClock.getElapsedTime();
	sf::Time time_lastGuiDesktopUpdate = timingClock.getElapsedTime();

	//Initialise Settings
	KinectSettings::serializeKinectSettings();
	sf::Font font;
	sf::Text debugText;
	// Global Debug Font
	attemptInitialiseDebugDisplay(font, debugText);

	//SFGUI Handling -------------------------------------- 
	GUIHandler guiRef;
	// ----------------------------------------------------

	//Initialise Kinect
	KinectSettings::kinectRepRotation = kinectQuaternionFromRads();
	kinect.update();

	guiRef.updateKinectStatusLabel(kinect);
	KinectSettings::isKinectPSMS = kinect.isPSMS;
	// Reconnect Kinect Event Signal
	guiRef.setKinectButtonSignal(kinect);

	//Initialise InputEmu and Trackers
	std::vector<KinectTrackedDevice> v_trackers{};

	VRcontroller rightController(vr::TrackedControllerRole_RightHand);
	VRcontroller leftController(vr::TrackedControllerRole_LeftHand);


	LOG(INFO) << "Attempting connection to vrsystem.... "; // DEBUG
	vr::EVRInitError eError = vr::VRInitError_None;
	vr::IVRSystem* m_VRSystem = VR_Init(&eError, vr::VRApplication_Overlay);

	LOG_IF(eError != vr::VRInitError_None, ERROR) << "IVRSystem could not be initialised: EVRInitError Code " <<
 static_cast<int>(eError);

	// INPUT BINDING TEMPORARY --------------------------------
	// Warn about non-english file path, as openvr can only take ASCII chars
	verifyDefaultFilePath();

	if (eError == vr::VRInitError_None)
	{
		// Set origins so that proper offsets for each coordinate system can be found
		KinectSettings::trackingOrigin = m_VRSystem->GetRawZeroPoseToStandingAbsoluteTrackingPose();
		KinectSettings::trackingOriginPosition = GetVRPositionFromMatrix(KinectSettings::trackingOrigin);
		double yaw = std::atan2(KinectSettings::trackingOrigin.m[0][2], KinectSettings::trackingOrigin.m[2][2]);
		if (yaw < 0.0)
		{
			yaw = 2 * M_PI + yaw;
		}
		KinectSettings::svrhmdyaw = yaw;

		LOG(INFO) << "SteamVR Tracking Origin for Driver Relative: " <<
			KinectSettings::trackingOriginPosition.v[0] << ", " <<
			KinectSettings::trackingOriginPosition.v[1] << ", " <<
			KinectSettings::trackingOriginPosition.v[2] << ", " <<
			KinectSettings::svrhmdyaw << "RAD";

		guiRef.setVRSceneChangeButtonSignal(m_VRSystem);
		updateTrackerInitGuiSignals(guiRef, v_trackers, m_VRSystem);
		setTrackerRolesInVRSettings();
		//VRInput::initialiseVRInput();

		leftController.Connect(m_VRSystem);
		rightController.Connect(m_VRSystem);

		// Todo: implement binding system
		guiRef.loadK2VRIntoBindingsMenu(m_VRSystem);
	}
	// Function pointer for the currently selected calibration method, which can be swapped out for the others
	// Only one calibration method can be active at a time
	std::function<void
			(double deltaT,
			 KinectHandlerBase& kinect,
			 vr::VRActionHandle_t& h_horizontalPos,
			 vr::VRActionHandle_t& h_verticalPos,
			 vr::VRActionHandle_t& h_confirmPos,
			 GUIHandler& guiRef)>
		currentCalibrationMethod = ManualCalibrator::Calibrate;
	guiRef.updateVRStatusLabel(eError);


	KinectSettings::userChangingZero = true;

	//Default tracking methods
	std::vector<std::unique_ptr<TrackingMethod>> v_trackingMethods;
	guiRef.setTrackingMethodsReference(v_trackingMethods);

	SkeletonTracker mainSkeletalTracker;
	if (kinect.kVersion != KinectVersion::INVALID)
	{
		mainSkeletalTracker.initialise();
		kinect.initialiseSkeleton();
		v_trackingMethods.push_back(std::make_unique<SkeletonTracker>(mainSkeletalTracker));
	}

	IMU_PositionMethod posMethod;
	v_trackingMethods.push_back(std::make_unique<IMU_PositionMethod>(posMethod));

	IMU_RotationMethod rotMethod;
	v_trackingMethods.push_back(std::make_unique<IMU_RotationMethod>(rotMethod));

	VRDeviceHandler vrDeviceHandler(m_VRSystem);
	if (eError == vr::VRInitError_None)
		vrDeviceHandler.initialise();

	std::vector<std::unique_ptr<DeviceHandler>> v_deviceHandlers;
	v_deviceHandlers.push_back(std::make_unique<VRDeviceHandler>(vrDeviceHandler));
	guiRef.setDeviceHandlersReference(v_deviceHandlers);
	guiRef.initialisePSMoveHandlerIntoGUI(); // Needs the deviceHandlerRef to be set

	guiRef.coptbox->SelectItem(VirtualHips::settings.footOption);
	guiRef.coptbox1->SelectItem(VirtualHips::settings.hipsOption);
	guiRef.foptbox->SelectItem(VirtualHips::settings.posOption);
	guiRef.contrackingselectbox->SelectItem(VirtualHips::settings.conOption);
	guiRef.bodytrackingselectbox->SelectItem(VirtualHips::settings.bodyTrackingOption);
	guiRef.headtrackingselectbox->SelectItem(VirtualHips::settings.headTrackingOption);
	guiRef.refreshpsms();

	//select last psm ids

	boost::thread* ipcThread = new boost::thread(KinectSettings::sendipc);
	ipcThread->detach();

	while (renderWindow.isOpen() && SFMLsettings::keepRunning)
	{
		//Clear the debug text display
		SFMLsettings::debugDisplayTextStream.str(std::string());
		SFMLsettings::debugDisplayTextStream.clear();

		double currentTime = frameClock.restart().asSeconds();
		double deltaT = currentTime;
		SFMLsettings::debugDisplayTextStream << "FPS Start = " << 1.0 / deltaT << '\n';
		//std::cout << SFMLsettings::debugDisplayTextStream.str() << std::endl;

		if (timingClock.getElapsedTime() > time_lastGuiDesktopUpdate + sf::milliseconds(33))
		{
			sf::Event event;

			while (renderWindow.pollEvent(event))
			{
				guiRef.desktopHandleEvents(event);
				if (event.type == sf::Event::Closed)
				{
					SFMLsettings::keepRunning = false;
					KinectSettings::initialised = false;
					renderWindow.close();
					break;
				}
				if (event.type == sf::Event::KeyPressed)
				{
					processKeyEvents(event);
				}
				if (event.type == sf::Event::Resized)
				{
					std::cerr << "HELP I AM RESIZING!\n";
					//sf::Vector2f size = static_cast<sf::Vector2f>(renderWindow.getSize());
					sf::Vector2f size = sf::Vector2f(event.size.width, event.size.height);
					// Minimum size
					if (size.x < 800)
						size.x = 800;
					if (size.y < 600)
						size.y = 600;

					// Apply possible size changes
					renderWindow.setSize(static_cast<sf::Vector2u>(size));

					// Reset grid view
					mGridView.setCenter(size / 2.f);
					mGridView.setSize(size);
					// = sf::View(sf::FloatRect(mGridView.getCenter().x, mGridView.getCenter().y, mGridView.getSize().x+(mGridView.getSize().x - size.x), mGridView.getSize().y+(mGridView.getSize().y - size.y)));

					// Reset  GUI view
					mGUIView = sf::View(sf::FloatRect(0.f, 0.f, size.x, size.y));
					//mGUIView.setCenter(size / 2.f);
					renderWindow.setView(mGUIView);

					// Resize widgets
					updateKinectWindowRes(renderWindow);
					guiRef.updateWithNewWindowSize(size);
				}
			}
			if (!(renderWindow.isOpen() && SFMLsettings::keepRunning))
			{
				// Possible for window to be closed mid-loop, in which case, instead of using goto's
				// this is used to avoid glErrors that crash the program, and prevent proper
				// destruction and cleaning up
				break;
			}

			//Clear ---------------------------------------
			renderWindow.clear(); /////////////////////////////////////////////////////
			renderWindow.setView(mGridView); //////////////////////////////////////////
			renderWindow.setView(mGUIView); ///////////////////////////////////////////

			//Process -------------------------------------
			//Update GUI

			guiRef.updateDesktop(deltaT);
			time_lastGuiDesktopUpdate = timingClock.getElapsedTime();
		}

		//Update VR Components
		if (eError == vr::VRInitError_None)
		{
			rightController.update(deltaT);
			leftController.update(deltaT);

			updateHMDPosAndRot(m_VRSystem);

			/*std::cout << "X: " << float(int(rightController.GetControllerAxisValue(vr::k_EButton_SteamVR_Touchpad).x*10))/10.f <<
				" Y: " << float(int(rightController.GetControllerAxisValue(vr::k_EButton_SteamVR_Touchpad).y*10))/10.f << 
				" T: " << rightController.GetTrigger() << '\n';*/

			/***********************************************************************************************/
			std::cout <<
				"Left: " << KinectSettings::hidariashimove.Pose.Position.x << ' ' << KinectSettings::hidariashimove.Pose
				.Position.y << ' ' << KinectSettings::hidariashimove.Pose.Position.z << '\n' <<
				"Right: " << KinectSettings::migiashimove.Pose.Position.x << ' ' << KinectSettings::migiashimove.Pose.
				Position.y << ' ' << KinectSettings::migiashimove.Pose.Position.z << '\n' <<
				"Waist: " << KinectSettings::yobumove.Pose.Position.x << ' ' << KinectSettings::yobumove.Pose.Position.y
				<< ' ' << KinectSettings::yobumove.Pose.Position.z << "\n\n";
			/***********************************************************************************************/

			VRInput::trackpadpose[0].x = rightController.GetControllerAxisValue(vr::k_EButton_SteamVR_Touchpad).x;
			VRInput::trackpadpose[0].y = rightController.GetControllerAxisValue(vr::k_EButton_SteamVR_Touchpad).y;
			VRInput::trackpadpose[1].x = leftController.GetControllerAxisValue(vr::k_EButton_SteamVR_Touchpad).x;
			VRInput::trackpadpose[1].y = leftController.GetControllerAxisValue(vr::k_EButton_SteamVR_Touchpad).y;
			VRInput::confirmdatapose.bState = leftController.GetTriggerDown() || rightController.GetTriggerDown();

			KinectSettings::isGripPressed[0] = rightController.GetGripDown();
			KinectSettings::isGripPressed[1] = leftController.GetGrip();
			KinectSettings::isTriggerPressed[0] = rightController.GetTrigger();
			KinectSettings::isTriggerPressed[1] = leftController.GetTrigger();

			// EWWWWWWWWW -------------
			if (VRInput::legacyInputModeEnabled)
			{
				using namespace VRInput;
				moveHorizontallyData.bActive = true;
				auto leftStickValues = leftController.GetControllerAxisValue(vr::k_EButton_SteamVR_Touchpad);
				moveHorizontallyData.x = leftStickValues.x;
				moveHorizontallyData.y = leftStickValues.y;

				moveVerticallyData.bActive = true;
				auto rightStickValues = rightController.GetControllerAxisValue(vr::k_EButton_SteamVR_Touchpad);
				moveVerticallyData.x = rightStickValues.x;
				moveVerticallyData.y = rightStickValues.y;

				confirmCalibrationData.bActive = true;
				auto triggerDown = leftController.GetTriggerDown() || rightController.GetTriggerDown();
				confirmCalibrationData.bState = triggerDown;
			}
			// -------------------------
		}

		for (auto& device_ptr : v_deviceHandlers)
		{
			if (device_ptr->active) device_ptr->run();
		}

		renderWindow.clear(); //////////////////////////////////////////////////////

		// Update Kinect Status
		// Only needs to be updated sparingly
		if (timingClock.getElapsedTime() > time_lastKinectStatusUpdate + sf::seconds(2.0))
		{
			guiRef.updateKinectStatusLabel(kinect);
			time_lastKinectStatusUpdate = timingClock.getElapsedTime();
		}

		if (kinect.isInitialised())
		{
			kinect.update();
			if (KinectSettings::adjustingKinectRepresentationPos
				|| KinectSettings::adjustingKinectRepresentationRot)
				currentCalibrationMethod(
					deltaT,
					kinect,
					VRInput::moveHorizontallyHandle,
					VRInput::moveVerticallyHandle,
					VRInput::confirmCalibrationHandle,
					guiRef);

			//kinect.updateTrackersWithSkeletonPosition(v_trackers);

			for (auto& method_ptr : v_trackingMethods)
			{
				method_ptr->update(kinect, v_trackers);
				method_ptr->updateTrackers(kinect, v_trackers);
			}
			for (auto& tracker : v_trackers)
			{
				tracker.update();
			}

			//renderWindow.clear();
			kinect.drawKinectData(renderWindow);
		}

		if (footOrientationFilterOption.filterOption != static_cast<footRotationFilterOption>(guiRef.coptbox->
			GetSelectedItem()))
		{
			footOrientationFilterOption.filterOption = static_cast<footRotationFilterOption>(guiRef.coptbox->
				GetSelectedItem());
			VirtualHips::settings.footOption = guiRef.coptbox->GetSelectedItem();

			VirtualHips::saveSettings();
		}
		if (hipsOrientationFilterOption.filterOption != static_cast<hipsRotationFilterOption>(guiRef.coptbox1->
			GetSelectedItem()))
		{
			hipsOrientationFilterOption.filterOption = static_cast<hipsRotationFilterOption>(guiRef.coptbox1->
				GetSelectedItem());
			VirtualHips::settings.hipsOption = guiRef.coptbox1->GetSelectedItem();

			VirtualHips::saveSettings();
		}
		if (positionFilterOption.filterOption != static_cast<positionalFilterOption>(guiRef.foptbox->GetSelectedItem()))
		{
			positionFilterOption.filterOption = static_cast<positionalFilterOption>(guiRef.foptbox->GetSelectedItem());
			VirtualHips::settings.posOption = guiRef.foptbox->GetSelectedItem();

			VirtualHips::saveSettings();
		}
		if (kontororaTorakkinguOpushon.trackingOption != static_cast<kontororaTorakkinguOpu>(guiRef.contrackingselectbox
			->GetSelectedItem()))
		{
			kontororaTorakkinguOpushon.trackingOption = static_cast<kontororaTorakkinguOpu>(guiRef.contrackingselectbox
				->GetSelectedItem());
			VirtualHips::settings.conOption = guiRef.contrackingselectbox->GetSelectedItem();

			VirtualHips::saveSettings();
		}
		if (bodiTorakkinguOpushon.trackingOption != static_cast<bodiTorakkinguOpu>(guiRef.bodytrackingselectbox->
			GetSelectedItem()))
		{
			bodiTorakkinguOpushon.trackingOption = static_cast<bodiTorakkinguOpu>(guiRef.bodytrackingselectbox->
				GetSelectedItem());
			VirtualHips::settings.bodyTrackingOption = guiRef.bodytrackingselectbox->GetSelectedItem();

			VirtualHips::saveSettings();
		}
		if (atamaTorakkinguOpushon.trackingOption != static_cast<atamaTorakkinguOpu>(guiRef.headtrackingselectbox->
			GetSelectedItem()))
		{
			atamaTorakkinguOpushon.trackingOption = static_cast<atamaTorakkinguOpu>(guiRef.headtrackingselectbox->
				GetSelectedItem());
			VirtualHips::settings.headTrackingOption = guiRef.headtrackingselectbox->GetSelectedItem();

			VirtualHips::saveSettings();
		}

		if (VirtualHips::settings.bodyTrackingOption == k_PSMoveFullTracking)
			guiRef.psmidbox->Show(true);
		else
			guiRef.psmidbox->Show(false);

		if (VirtualHips::settings.headTrackingOption == k_PSMoveTracking)
			guiRef.psmidbox1->Show(true);
		else
			guiRef.psmidbox1->Show(false);

		KinectSettings::footOption = VirtualHips::settings.footOption;
		KinectSettings::hipsOption = VirtualHips::settings.hipsOption;
		KinectSettings::posOption = VirtualHips::settings.posOption;
		KinectSettings::conOption = VirtualHips::settings.conOption;
		KinectSettings::bodytrackingoption = VirtualHips::settings.bodyTrackingOption;
		KinectSettings::headtrackingoption = VirtualHips::settings.headTrackingOption;

		//KinectSettings::footRotationFilterOption::k_EnableOrientationFilter;

		// Draw GUI
		updateHMDPosAndRot(m_VRSystem);

		//renderWindow.clear(); //////////////////////////////////////////////////////

		renderWindow.setActive(true);
		renderWindow.setView(mGUIView);
		guiRef.display(renderWindow);

		//Draw debug font
		double endTimeMilliseconds = frameClock.getElapsedTime().asMilliseconds();
		SFMLsettings::debugDisplayTextStream << "endTimeMilli: " << endTimeMilliseconds << '\n';

		//limitVRFramerate(endTimeMilliseconds);
		debugText.setString(SFMLsettings::debugDisplayTextStream.str());
		renderWindow.draw(debugText);

		renderWindow.resetGLStates();
		//End Frame
		renderWindow.display();
	}
	for (auto& device_ptr : v_deviceHandlers)
	{
		device_ptr->shutdown();
	}
	for (KinectTrackedDevice d : v_trackers)
	{
		d.destroy();
	}
	KinectSettings::writeKinectSettings();
	VirtualHips::saveSettings();

	//playspaceMovementAdjuster.resetPlayspaceAdjustments();
	if (eError == vr::EVRInitError::VRInitError_None)
	{
		removeTrackerRolesInVRSettings();
		vr::VR_Shutdown();
	}
}

void spawnDefaultLowerBodyTrackers()
{
	std::thread* activate = new std::thread([]
	{
		HANDLE pingPipe = CreateFile(
			TEXT("\\\\.\\pipe\\TrackersInitPipe"), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
		DWORD Written;

		std::string InitS = "Initialize Trackers!";

		char InitD[1024];
		strcpy_s(InitD, InitS.c_str());

		WriteFile(pingPipe, InitD, sizeof(InitD), &Written, nullptr);
		CloseHandle(pingPipe);
	});
}
