#include "stdafx.h"
#include "KinectToVR.h"
#include "VRHelper.h"

#include "wtypes.h"
#include <Windows.h>

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
#include "PSMoveHandler.h"
#include "DeviceHandler.h"
#include "TrackingPoolManager.h"

#include <SFML\Audio.hpp>

#include <locale>
#include <codecvt>
#include <iostream>
#include <string>
#include <thread>
//GUI
#include <SFGUI\SFGUI.hpp>
#include <SFGUI/Widgets.hpp>

//OpenCV
#include <opencv2\opencv.hpp>

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

void toEulerAngle(vr::HmdQuaternion_t q, double& roll, double& pitch, double& yaw)
{
    // roll (x-axis rotation)
    double sinr = +2.0 * (q.w * q.x + q.y * q.z);
    double cosr = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    roll = atan2(sinr, cosr);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny = +2.0 * (q.w * q.z + q.x * q.y);
    double cosy = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    yaw = atan2(siny, cosy);
}



void processKeyEvents(sf::Event event) {
    switch (event.key.code) {
    case sf::Keyboard::A:
        toggle(KinectSettings::isKinectDrawn);
        break;
    default:
        break;
    }
}
void toggle(bool &b) {
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
sf::VideoMode getScaledWindowResolution() {
    int h;
    int v;
    getDesktopResolution(h, v);
    
    sf::VideoMode mode = sf::VideoMode(SFMLsettings::windowScale*float(h), SFMLsettings::windowScale*float(v));
    //std::cerr << "desktop: " << h << ", " << v << '\n';
    //std::cerr << "scaled: " << mode.width << ", " << mode.height << '\n';
    return mode;
}
void updateKinectWindowRes(const sf::RenderWindow& window) {
    SFMLsettings::m_window_width = window.getSize().x;
    SFMLsettings::m_window_height = window.getSize().y;
    //std::cerr << "w: " << SFMLsettings::m_window_width << " h: " << SFMLsettings::m_window_height << "\n";
}

bool filePathIsNonASCII(const std::wstring& filePath) {
    for (auto c : filePath) {
        if (static_cast<unsigned char>(c) > 127) {
            return true;
        }
    }
    return false;
}

void verifyDefaultFilePath() {
    // Warn about non-english file path, as openvr can only take ASCII chars
    // If this isn't checked, unfortunately, most of the bindings won't load
    // Unless OpenVR's C API adds support for non-english filepaths, K2VR can't either
    if (filePathIsNonASCII(SFMLsettings::fileDirectoryPath)) {
        auto message = L"WARNING: NON-ENGLISH FILEPATH DETECTED: "
            + SFMLsettings::fileDirectoryPath
            + L"\n It's possible that OpenVR bindings won't work correctly"
            + L"\n Please move the K2VR directory to a location with ASCII only"
            + L"\n e.g. C:/KinectToVR will be fine";
        auto result = MessageBox(NULL, message.c_str(), L"WARNING!!!", MB_ABORTRETRYIGNORE + MB_ICONWARNING);
        if (result = IDABORT) {
            SFMLsettings::keepRunning = false;
        }
    }
}
void updateFilePath() {
    
    HMODULE module = GetModuleHandleW(NULL);
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
}
void attemptInitialiseDebugDisplay(sf::Font &font, sf::Text &debugText) {
    // Global Debug Font
#if _DEBUG
    auto fontFileName = "arial.ttf";
    std::cout << "Attemping Debug Font Load: " << fontFileName << '\n';
    font.loadFromFile(fontFileName);
    debugText.setFont(font);
#endif
    debugText.setString("");
    debugText.setCharacterSize(40);
    debugText.setFillColor(sf::Color::Red);

    debugText.setString(SFMLsettings::debugDisplayTextStream.str());
}
vr::HmdQuaternion_t kinectQuaternionFromRads() {
    return vrmath::quaternionFromYawPitchRoll(KinectSettings::kinectRadRotation.v[1], KinectSettings::kinectRadRotation.v[0], KinectSettings::kinectRadRotation.v[2]);
}
void attemptIEmulatorConnection(vrinputemulator::VRInputEmulator & inputEmulator, GUIHandler & guiRef) {
    try {
        inputEmulator.connect();
    }
    catch (vrinputemulator::vrinputemulator_connectionerror e) {
        guiRef.updateEmuStatusLabelError(e);
        std::cerr << "Attempted connection to Input Emulator" << std::to_string(e.errorcode) + " " + e.what() + "\n\n Is SteamVR open and InputEmulator installed?" << std::endl;
    }
}
void updateTrackerInitGuiSignals(vrinputemulator::VRInputEmulator &inputEmulator, GUIHandler &guiRef, std::vector<KVR::KinectTrackedDevice> & v_trackers) {
    if (inputEmulator.isConnected()) {
        guiRef.setTrackerButtonSignals(inputEmulator, v_trackers);
        guiRef.updateEmuStatusLabelSuccess();
    }
    else {
        guiRef.updateTrackerInitButtonLabelFail();
    }
}

void limitVRFramerate(double &endFrameMilliseconds)
{
    // Framerate limiting - as SFML only has 90 FPS when window in focus X-X
    // Strange bugs occurred before, with the render window framerate being
    // *sometimes* tied to the VR update. With this, it fixes it at 90,
    // and 30 for the GUI - as it should be (GUI uses a lot of CPU to update)
    static unsigned int FPS = 90;

    double deltaDeviationMilliseconds;
    int maxMillisecondsToCompensate = 30;

    deltaDeviationMilliseconds = 1000.0 / FPS - endFrameMilliseconds;

    if (floor(deltaDeviationMilliseconds) > 0) // TODO: Handle -ve (slow) frames
        Sleep(deltaDeviationMilliseconds);
    if (deltaDeviationMilliseconds < -maxMillisecondsToCompensate) {
        endFrameMilliseconds -= maxMillisecondsToCompensate;
    }
    else {
        endFrameMilliseconds += deltaDeviationMilliseconds;
    }
    //SFMLsettings::debugDisplayTextStream << "deviateMilli: " << deltaDeviationMilliseconds << '\n';
    //SFMLsettings::debugDisplayTextStream << "POST endTimeMilli: " << endFrameMilliseconds << '\n';
    //SFMLsettings::debugDisplayTextStream << "FPS End = " << 1000.0 / endFrameMilliseconds << '\n';
}

void processLoop(KinectHandlerBase& kinect) {
    updateFilePath();
    sf::RenderWindow renderWindow(getScaledWindowResolution(), "KinectToVR: " + KinectSettings::KVRversion, sf::Style::Titlebar | sf::Style::Close);
    updateKinectWindowRes(renderWindow);
    //renderWindow.setFramerateLimit(90);   //Prevents ridiculous overupdating and high CPU usage - plus 90Hz is the recommended refresh rate for most VR panels 
    renderWindow.setVerticalSyncEnabled(true);

    sf::Clock clock;

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
    // Reconnect Kinect Event Signal
    guiRef.setKinectButtonSignal(kinect);

    //Initialise InputEmu and Trackers
    std::vector<KVR::KinectTrackedDevice> v_trackers{};
    vrinputemulator::VRInputEmulator inputEmulator;
    attemptIEmulatorConnection(inputEmulator, guiRef);
    updateTrackerInitGuiSignals(inputEmulator, guiRef, v_trackers);

    // Function pointer for the currently selected calibration method, which can be swapped out for the others
    // Only one calibration method can be active at a time
    std::function<void
    (double deltaT,
        KinectHandlerBase &kinect,
        vr::VRActionHandle_t &h_horizontalPos,
        vr::VRActionHandle_t &h_verticalPos,
        vr::VRActionHandle_t &h_confirmPos,
        GUIHandler &guiRef)>
        currentCalibrationMethod = ManualCalibrator::Calibrate;


    VRcontroller rightController(vr::TrackedControllerRole_RightHand);
    VRcontroller leftController(vr::TrackedControllerRole_LeftHand);

    std::cerr << "Attempting connection to vrsystem.... " << std::endl;    // DEBUG
    vr::EVRInitError eError = vr::VRInitError_None;
    vr::IVRSystem *m_VRSystem = vr::VR_Init(&eError, vr::VRApplication_Background);
    guiRef.setVRSceneChangeButtonSignal(m_VRSystem);

    // INPUT BINDING TEMPORARY --------------------------------
    // Warn about non-english file path, as openvr can only take ASCII chars
    verifyDefaultFilePath();

    setTrackerRolesInVRSettings();


    vr::EVRInputError iError = vr::VRInput()->SetActionManifestPath(KVR::inputDirForOpenVR("action-manifest.json"));

    vr::VRActionHandle_t moveHorizontallyHandle;
    vr::VRActionHandle_t moveVerticallyHandle;
    vr::VRActionHandle_t confirmCalibrationHandle;
    iError = vr::VRInput()->GetActionHandle("/actions/calibration/in/MoveHorizontally", &moveHorizontallyHandle);
    iError = vr::VRInput()->GetActionHandle("/actions/calibration/in/MoveVertically", &moveVerticallyHandle);
    iError = vr::VRInput()->GetActionHandle("/actions/calibration/in/ConfirmCalibration", &confirmCalibrationHandle);

    vr::VRActionSetHandle_t calibrationSetHandle;
    iError = vr::VRInput()->GetActionSetHandle("/actions/calibration", &calibrationSetHandle);

    vr::VRActiveActionSet_t activeActionSet;
    activeActionSet.ulActionSet = calibrationSetHandle;
    activeActionSet.ulRestrictedToDevice = vr::k_ulInvalidInputValueHandle;
    activeActionSet.unPadding;
    activeActionSet.nPriority = 0;
    iError = vr::VRInput()->UpdateActionState(&activeActionSet, sizeof(activeActionSet), 1);

    vr::InputDigitalActionData_t confirmCalibrationData{};
    iError = vr::VRInput()->GetDigitalActionData(confirmCalibrationHandle, &confirmCalibrationData, sizeof(confirmCalibrationData), activeActionSet.ulRestrictedToDevice);
    vr::InputAnalogActionData_t moveHorizontallyData{};
    iError = vr::VRInput()->GetAnalogActionData(moveHorizontallyHandle, &moveHorizontallyData, sizeof(moveHorizontallyData), activeActionSet.ulRestrictedToDevice);
    vr::InputAnalogActionData_t moveVerticallyData{};
    vr::VRInput()->GetAnalogActionData(moveVerticallyHandle, &moveVerticallyData, sizeof(moveVerticallyData), activeActionSet.ulRestrictedToDevice);
    // --------------------------------------------------------






    if (eError == vr::VRInitError_None) {
        std::cerr << "Attempting connection to controllers.... " << std::endl;    // DEBUG
        leftController.Connect(m_VRSystem);
        rightController.Connect(m_VRSystem);
        std::cerr << "Attempted connection to controllers! " << std::endl;    // DEBUG

        // Todo: implement binding system
        guiRef.loadK2VRIntoBindingsMenu(m_VRSystem);
    }
    guiRef.updateVRStatusLabel(eError);
    std::cerr << "Attempted connection to vrsystem! " << eError << std::endl;    // DEBUG

    guiRef.setReconnectControllerButtonSignal(leftController, rightController, m_VRSystem);

    KinectSettings::userChangingZero = true;

    //Default tracking methods
    std::vector<std::unique_ptr<TrackingMethod>> v_trackingMethods;
    guiRef.setTrackingMethodsReference(v_trackingMethods);

    SkeletonTracker mainSkeletalTracker;
    kinect.initialiseSkeleton();
    v_trackingMethods.push_back(std::make_unique<SkeletonTracker>(mainSkeletalTracker));

    IMU_PositionMethod posMethod;
    v_trackingMethods.push_back(std::make_unique<IMU_PositionMethod>(posMethod));

    IMU_RotationMethod rotMethod;
    v_trackingMethods.push_back(std::make_unique<IMU_RotationMethod>(rotMethod));
    /*
    ColorTracker mainColorTracker(KinectSettings::kinectV2Width, KinectSettings::kinectV2Height);
    v_trackingMethods.push_back(mainColorTracker);
    */

    // Physical Device Handlers
    // Ideally, nothing should be spawned in code, and everything done by user input
    // This means that these Handlers are spawned in the GuiHandler, and each updated in the vector automatically
    std::vector<std::unique_ptr<DeviceHandler>> v_deviceHandlers;
    guiRef.setDeviceHandlersReference(v_deviceHandlers);



    while (renderWindow.isOpen() && SFMLsettings::keepRunning)
    {
        //Clear the debug text display
        SFMLsettings::debugDisplayTextStream.str(std::string());
        SFMLsettings::debugDisplayTextStream.clear();

        double currentTime = clock.restart().asSeconds();
        double deltaT = currentTime;
        SFMLsettings::debugDisplayTextStream << "FPS Start = " << 1.0 / deltaT << '\n';

        if (1.0 / deltaT > 60) { // DEBUG FOR PERF. BUG
            printf("Framerate rose\n");
        }

        // ----------------------------------------------------------------------


        updateKinectWindowRes(renderWindow);

        sf::Event event;

        while (renderWindow.pollEvent(event))
        {
            guiRef.desktopHandleEvents(event);
            if (event.type == sf::Event::Closed) {
                SFMLsettings::keepRunning = false;
                renderWindow.close();
            }
            if (event.type == sf::Event::KeyPressed) {
                processKeyEvents(event);
            }
        }

        //Clear ---------------------------------------
        renderWindow.clear();

        //Process -------------------------------------
        //Update GUI
        guiRef.updateDesktop(deltaT);

        /*
        // Framerate limiting - as SFML only has 90 FPS when window in focus X-X
        static unsigned int FPS = 90;

        double deltaDeviation;
        int maxMillisecondsToCompensate = 30;

        deltaDeviation = 1.0 / FPS - deltaT;

        if (floor(deltaDeviation) > 0)
            Sleep(deltaDeviation);

        if (deltaDeviation < -maxMillisecondsToCompensate) {
            deltaT -= maxMillisecondsToCompensate;
        }
        else {
            deltaT += deltaDeviation;
        }
        */

        //Update VR Components
        if (eError == vr::VRInitError_None) {
            //vr::VRInput()->UpdateActionState();
            rightController.update(deltaT);
            leftController.update(deltaT);
            updateHMDPosAndRot(m_VRSystem);

            // UPDATE INPUT PER FRAME ----------------------
            iError = vr::VRInput()->UpdateActionState(&activeActionSet, sizeof(activeActionSet), 1);
            // ----------------------------------------------
        }
        else {
            std::cerr << "Error updating controllers: Could not connect to the SteamVR system! OpenVR init error-code " << std::to_string(eError) << std::endl;
        }

        for (auto & device_ptr : v_deviceHandlers) {
            if (device_ptr->active) device_ptr->run();
        }

        // Update Kinect Status
        guiRef.updateKinectStatusLabel(kinect);
        if (kinect.isInitialised()) {
            kinect.update();
            if (KinectSettings::adjustingKinectRepresentationPos
                || KinectSettings::adjustingKinectRepresentationRot)
                currentCalibrationMethod(
                    deltaT,
                    kinect,
                    moveHorizontallyHandle,
                    moveVerticallyHandle,
                    confirmCalibrationHandle,
                    guiRef);

            kinect.updateTrackersWithSkeletonPosition(v_trackers);
            //std::vector<KVR::TrackedDeviceInputData> v_inputData = psMoveHandler.extractVRTrackingPoses();
            /*
            std::vector<KVR::TrackedDeviceInputData> v_inputData{}; // DEBUG
            for (auto & method_ptr : v_trackingMethods) {
                method_ptr->update(kinect, v_trackers);
                method_ptr->updateTrackers(kinect, v_trackers, v_inputData);
            }
            for (auto & tracker : v_trackers) {
                tracker.update();
            }
            */
            kinect.drawKinectData(renderWindow);
        }
        //std::vector<uint32_t> virtualDeviceIndexes;
        //for (KinectTrackedDevice d : v_trackers) {
        //    vrinputemulator::VirtualDeviceInfo info = inputEmulator.getVirtualDeviceInfo(d.deviceId);
        //    virtualDeviceIndexes.push_back(info.openvrDeviceId); // needs to be converted into openvr's id - as inputEmulator has it's own Id's starting from zero
        //}


        //playspaceMovementAdjuster.update(leftController, rightController, virtualDeviceIndexes);

        renderWindow.pushGLStates();



        // Draw GUI
        renderWindow.setActive(true);

        guiRef.display(renderWindow);

        //Draw debug font
        double endTimeMilliseconds = clock.getElapsedTime().asMilliseconds();
        SFMLsettings::debugDisplayTextStream << "endTimeMilli: " << endTimeMilliseconds << '\n';

        //limitVRFramerate(endTimeMilliseconds);
        debugText.setString(SFMLsettings::debugDisplayTextStream.str());
        renderWindow.draw(debugText);


        renderWindow.popGLStates();

        //End Frame
        renderWindow.display();

    }
    for (auto & device_ptr : v_deviceHandlers) {
        device_ptr->shutdown();
    }
    for (KinectTrackedDevice d : v_trackers) {
        d.destroy();
    }
    KinectSettings::writeKinectSettings();

    //playspaceMovementAdjuster.resetPlayspaceAdjustments();
    removeTrackerRolesInVRSettings();
    vr::VR_Shutdown();
}

void spawnAndConnectTracker(vrinputemulator::VRInputEmulator & inputE, std::vector<KVR::KinectTrackedDevice>& v_trackers, uint32_t posDevice_gId,
    uint32_t rotDevice_gId, KVR::KinectDeviceRole role)
{
    KVR::KinectTrackedDevice device(inputE, posDevice_gId, rotDevice_gId, role);
    device.init(inputE);
    v_trackers.push_back(device);
}
void spawnAndConnectTracker(vrinputemulator::VRInputEmulator & inputE, std::vector<KVR::KinectTrackedDevice>& v_trackers, KVR::KinectJointType mainJoint, KVR::KinectJointType secondaryJoint, KVR::KinectDeviceRole role)
{
    KVR::KinectTrackedDevice device(inputE, mainJoint, secondaryJoint, role);
    device.init(inputE);
    v_trackers.push_back(device);
}

void spawnAndConnectHandTrackers(vrinputemulator::VRInputEmulator & inputE, std::vector<KVR::KinectTrackedDevice>& v_trackers) {
    spawnAndConnectTracker(inputE, v_trackers, KVR::KinectJointType::WristLeft, KVR::KinectJointType::HandLeft, KVR::KinectDeviceRole::LeftHand);
    spawnAndConnectTracker(inputE, v_trackers, KVR::KinectJointType::WristRight, KVR::KinectJointType::HandRight, KVR::KinectDeviceRole::RightHand);
}
void spawnDefaultLowerBodyTrackers(vrinputemulator::VRInputEmulator & inputE, std::vector<KVR::KinectTrackedDevice>& v_trackers)
{
    spawnAndConnectTracker(inputE, v_trackers, KinectSettings::leftFootJointWithRotation, KinectSettings::leftFootJointWithoutRotation, KVR::KinectDeviceRole::LeftFoot);
    spawnAndConnectTracker(inputE, v_trackers, KinectSettings::rightFootJointWithRotation, KinectSettings::rightFootJointWithoutRotation, KVR::KinectDeviceRole::RightFoot);
    spawnAndConnectTracker(inputE, v_trackers, KVR::KinectJointType::SpineBase, KVR::KinectJointType::SpineMid, KVR::KinectDeviceRole::Hip);
}

void spawnAndConnectKinectTracker(vrinputemulator::VRInputEmulator &inputE, std::vector<KVR::KinectTrackedDevice> &v_trackers)
{
    KVR::KinectTrackedDevice kinectTrackerRef(inputE, KVR::KinectJointType::Head, KVR::KinectJointType::Head, KVR::KinectDeviceRole::KinectSensor);
    kinectTrackerRef.init(inputE);
    setKinectTrackerProperties(inputE, kinectTrackerRef.deviceId);
    v_trackers.push_back(kinectTrackerRef);
}
