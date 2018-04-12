#include "stdafx.h"
#include "KinectToVR.h"
#include "VRHelper.h"

#include "wtypes.h"

#include "KinectSettings.h"
#include "VRController.h"
#include "GamepadController.h"
#include "GUIHandler.h"
#include <SFML\Audio.hpp>
#include <iostream>
//GUI
#include <SFGUI\SFGUI.hpp>
#include <SFGUI/Widgets.hpp>


using namespace KVR;

/*
void updateKinectTracker(vrinputemulator::VRInputEmulator &emulator, KinectTrackedDevice device)
{
    auto pose = emulator.getVirtualDevicePose(device.deviceId);
    // The Kinect Tracker position must be rotated, as otherwise the tracker is oriented to the wrong direction
    
    double kRelativeX =  - kinectZero.x;
    double kRelativeY =  - kinectZero.y;
    double kRelativeZ = - kinectZero.z;
    double rawPositionX = hmdZero.v[0] + kRelativeX;
    double rawPositionZ =  hmdZero.v[2] + kRelativeZ;

    pose.vecPosition[0] = kinectToVRScale * rawPositionX;
    pose.vecPosition[1] = kinectToVRScale * kRelativeY;
    pose.vecPosition[2] = kinectToVRScale * rawPositionZ;
    

    double kRelativeX = -kinectZero.x;
    double kRelativeY = -kinectZero.y;
    double kRelativeZ = -kinectZero.z;
    double rawPositionX = hmdZero.v[0] + kRelativeX;
    double rawPositionZ = hmdZero.v[2] + kRelativeZ;

    pose.vecPosition[0] = rawPositionX;
    pose.vecPosition[1] = kinectToVRScale * kRelativeY;
    pose.vecPosition[2] = rawPositionZ;

    pose.poseIsValid = true;
    pose.result = vr::TrackingResult_Running_OK;
    emulator.setVirtualDevicePose(device.deviceId, pose);
}
*/
double deltaScaled(double valuePerSecond, double delta) {
    return valuePerSecond * delta;
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
    case sf::Keyboard::S:
        toggle(KinectSettings::isSkeletonDrawn);
        break;
    case sf::Keyboard::Q:
        KinectSettings::userChangingZero = true;
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

void PlayspaceMovementUpdate(sf::Vector3f& lastLeftPosition, sf::Vector3f& lastRightPosition) {
	// Grab controllers
	vr::TrackedDevicePose_t devicePoses[vr::k_unMaxTrackedDeviceCount];
	vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0.0f, devicePoses, vr::k_unMaxTrackedDeviceCount);
	auto leftId = vr::VRSystem()->GetTrackedDeviceIndexForControllerRole(vr::TrackedControllerRole_LeftHand);
	if (leftId == vr::k_unTrackedDeviceIndexInvalid) {
		return;
	}
	auto rightId = vr::VRSystem()->GetTrackedDeviceIndexForControllerRole(vr::TrackedControllerRole_RightHand);
	if (rightId == vr::k_unTrackedDeviceIndexInvalid) {
		return;
	}
	// Grab controller positions
	vr::TrackedDevicePose_t* leftPose = devicePoses+leftId;
	vr::TrackedDevicePose_t* rightPose = devicePoses+rightId;

	if (!leftPose->bPoseIsValid || !leftPose->bDeviceIsConnected ) {
		return;
	}
	if (!rightPose->bPoseIsValid || !rightPose->bDeviceIsConnected ) {
		return;
	}
	vr::HmdMatrix34_t* leftMat = &(leftPose->mDeviceToAbsoluteTracking);
	vr::HmdMatrix34_t* rightMat = &(leftPose->mDeviceToAbsoluteTracking);
	sf::Vector3f leftPos = sf::Vector3f(leftMat->m[0][3], leftMat->m[1][3], leftMat->m[2][3]);
	sf::Vector3f rightPos = sf::Vector3f(rightMat->m[0][3], rightMat->m[1][3], rightMat->m[2][3]);

	// Grab controller buttons
	vr::VRControllerState_t leftButtons;
	vr::VRControllerState_t rightButtons;
	vr::VRSystem()->GetControllerState(rightId, &rightButtons, sizeof(vr::VRControllerState_t));
	vr::VRSystem()->GetControllerState(leftId, &leftButtons, sizeof(vr::VRControllerState_t));

	// Check if left hand button is pressed
	sf::Vector3f ldelta = leftPos - lastLeftPosition;
	lastLeftPosition = leftPos;

	sf::Vector3f rdelta = rightPos - lastRightPosition;
	lastLeftPosition = leftPos;

	int button = KinectSettings::leftHandPlayspaceMovementButton;
	if (button && (leftButtons.ulButtonPressed & (1 << (button - 1)))) {
		MoveUniverseOrigin(vr::TrackingUniverseStanding, ldelta);
	}
	button = KinectSettings::rightHandPlayspaceMovementButton;
	if (button && (rightButtons.ulButtonPressed & (1 << (button - 1)))) {
		MoveUniverseOrigin(vr::TrackingUniverseStanding, rdelta);
	}
}

void processLoop(KinectHandlerBase& kinect) {
    sf::RenderWindow renderWindow(getScaledWindowResolution(), "KinectToVR: " + KinectSettings::KVRversion, sf::Style::Titlebar | sf::Style::Close);
    updateKinectWindowRes(renderWindow);
    renderWindow.setFramerateLimit(45);   //Prevents ridiculous overupdating and high CPU usage - plus 90Hz is the recommended refresh rate for most VR panels 

    sf::Clock clock;

    //Initialise Settings
    KinectSettings::serializeKinectSettings();
    sf::Font font;
    sf::Text debugText;
    // Global Debug Font
    //font.loadFromFile("arial.ttf");
    //debugText.setFont(font);
    debugText.setString("");
    debugText.setCharacterSize(40);
    debugText.setFillColor(sf::Color::Red);

    debugText.setString(SFMLsettings::debugDisplayTextStream.str());

    //SFGUI Handling -------------------------------------- 
    GUIHandler guiRef;
    // ----------------------------------------------------

    //Initialise Kinect
    KinectSettings::kinectRepRotation = vrmath::quaternionFromYawPitchRoll(KinectSettings::kinectRadRotation.v[1], KinectSettings::kinectRadRotation.v[0], KinectSettings::kinectRadRotation.v[2]);
    kinect.update();

    guiRef.updateKinectStatusLabel(kinect);
    // Reconnect Kinect Event Signal
    guiRef.setKinectButtonSignal(kinect);

    //Initialise InputEmu and Trackers
    std::vector<KVR::KinectTrackedDevice> v_trackers{};
    vrinputemulator::VRInputEmulator inputEmulator;
    try {
        inputEmulator.connect();
    }
    catch (vrinputemulator::vrinputemulator_connectionerror e) {
        guiRef.updateEmuStatusLabelError(e);
        std::cerr << "Attempted connection to Input Emulator" << std::to_string(e.errorcode) + " " + e.what() + "\n\n Is SteamVR open and InputEmulator installed?" << std::endl;   // DEBUG
    }

    // Tracker Initialisation Lambda
    if (inputEmulator.isConnected()) {
        guiRef.setTrackerInitButtonSignal(inputEmulator, v_trackers);
        guiRef.updateEmuStatusLabelSuccess();
    }
    else {
        guiRef.updateTrackerInitButtonLabelFail();
    }

    //v_trackers.push_back(kinectTrackerRef);
    std::cerr << "Attempting connection to vrsystem.... " << std::endl;    // DEBUG
                                                                           //Initialise VR System
    VRcontroller rightController(vr::TrackedControllerRole_RightHand);
    VRcontroller leftController(vr::TrackedControllerRole_LeftHand);
    
    GamepadController gamepad;

    vr::EVRInitError eError = vr::VRInitError_None;
    vr::IVRSystem *m_VRSystem = vr::VR_Init(&eError, vr::VRApplication_Utility);
    if (eError == vr::VRInitError_None) {
        std::cerr << "Attempting connection to controllers.... " << std::endl;    // DEBUG
        leftController.Connect(m_VRSystem);
        rightController.Connect(m_VRSystem);
        std::cerr << "Attempted connection to controllers! " << std::endl;    // DEBUG
    }
    guiRef.updateVRStatusLabel(eError);
    std::cerr << "Attempted connection to vrsystem! " << eError << std::endl;    // DEBUG

    guiRef.setReconnectControllerButtonSignal(leftController, rightController, m_VRSystem);

    KinectSettings::userChangingZero = true;
	// FIXME: frame 0 to 1, delta position will be wrong, if they're holding the movement buttons, they'll get zipped into space.
	sf::Vector3f lastLeftPosition = sf::Vector3f(0, 0, 0);
	sf::Vector3f lastRightPosition = sf::Vector3f(0, 0, 0);
    while (renderWindow.isOpen())
    {
        //Clear the debug text display
        SFMLsettings::debugDisplayTextStream.str(std::string());
        SFMLsettings::debugDisplayTextStream.clear();

        std::stringstream ss;
        double currentTime = clock.restart().asSeconds();
        double deltaT = currentTime;
        ss << "FPS = " << 1.0 / deltaT << '\n';

        updateKinectWindowRes(renderWindow);

        sf::Event event;
        while (renderWindow.pollEvent(event))
        {
            guiRef.desktopHandleEvents(event);

            if (event.type == sf::Event::Closed)
                renderWindow.close();
            if (event.type == sf::Event::KeyPressed) {
                processKeyEvents(event);
            }
        }

        //Clear ---------------------------------------
        renderWindow.clear();


        //Process -------------------------------------
        //Update GUI
        guiRef.updateDesktop(deltaT);
        // Update Kinect Status
		PlayspaceMovementUpdate(lastLeftPosition, lastRightPosition);
        rightController.Connect(m_VRSystem);
        leftController.Connect(m_VRSystem);
        if (eError == vr::VRInitError_None) {
            rightController.update(deltaT);
            leftController.update(deltaT);
        }
        else {
            std::cerr << "Error updating controllers: Could not connect to the SteamVR system! OpenVR init error-code " << std::to_string(eError) << std::endl;
        }

        guiRef.updateKinectStatusLabel(kinect);
        if (kinect.isInitialised()) {
            kinect.update();
            
            if (KinectSettings::adjustingKinectRepresentationPos) { //TEMP FOR TESTING IMPLMENTATION- Warning Gross af
                if (SFMLsettings::usingGamepad) {
                    sf::Vector2f axis = gamepad.leftThumbstickValue();
                    KinectSettings::kinectRepPosition.v[0] += deltaScaled(1.0, deltaT) * axis.x;
                    KinectSettings::kinectRepPosition.v[2] += deltaScaled(1.0, deltaT) * axis.y;

                    axis = gamepad.rightThumbstickValue();
                    KinectSettings::kinectRepPosition.v[1] += deltaScaled(1.0, deltaT) * axis.y;

                    if (gamepad.pressedRightTrigger()) {
                        KinectSettings::adjustingKinectRepresentationPos = false;
                        guiRef.togglePosButton();
                        rightController.setHapticPulse(.15, 1000, 0);
                    }
                }
                else {
                    if (leftController.GetTouch(vr::EVRButtonId::k_EButton_SteamVR_Touchpad)) {
                        sf::Vector2f axis = leftController.GetControllerAxisValue(vr::EVRButtonId::k_EButton_SteamVR_Touchpad);
                        KinectSettings::kinectRepPosition.v[0] += deltaScaled(1.0, deltaT) * axis.x;
                        KinectSettings::kinectRepPosition.v[2] += deltaScaled(1.0, deltaT) * axis.y;
                    }
                    if (rightController.GetTouch(vr::EVRButtonId::k_EButton_SteamVR_Touchpad)) {
                        sf::Vector2f axis = rightController.GetControllerAxisValue(vr::EVRButtonId::k_EButton_SteamVR_Touchpad);
                        KinectSettings::kinectRepPosition.v[1] += deltaScaled(1.0, deltaT) * axis.y;
                    }
                    if (rightController.GetTrigger()) {
                        KinectSettings::adjustingKinectRepresentationPos = false;
                        guiRef.togglePosButton();
                    }
                }
                
            }
            else if (KinectSettings::adjustingKinectRepresentationRot) { //TEMP FOR TESTING IMPLMENTATION
                KinectSettings::kinectRepRotation = vrmath::quaternionFromYawPitchRoll(KinectSettings::kinectRadRotation.v[1], KinectSettings::kinectRadRotation.v[0], KinectSettings::kinectRadRotation.v[2]);
                if (SFMLsettings::usingGamepad) {
                    sf::Vector2f axis = gamepad.leftThumbstickValue();
                    KinectSettings::kinectRadRotation.v[1] += deltaScaled(3.0, deltaT) * axis.x;

                    axis = gamepad.rightThumbstickValue();
                    KinectSettings::kinectRadRotation.v[0] += deltaScaled(3.0, deltaT) * axis.y;

                    if (gamepad.pressedRightTrigger()) {
                        KinectSettings::adjustingKinectRepresentationRot = false;
                        guiRef.toggleRotButton();
                    }
                }
                else {
                    
                    //std::cerr << isRotating << '\n';
                    if (leftController.GetTouch(vr::EVRButtonId::k_EButton_SteamVR_Touchpad)) {
                        sf::Vector2f axis = leftController.GetControllerAxisValue(vr::EVRButtonId::k_EButton_SteamVR_Touchpad);
                        KinectSettings::kinectRadRotation.v[1] += deltaScaled(3.0, deltaT) * axis.x;
                        //std::cerr << "ROT ADJUSTED L\n";
                    }
                    if (rightController.GetTouch(vr::EVRButtonId::k_EButton_SteamVR_Touchpad)) {
                        sf::Vector2f axis = rightController.GetControllerAxisValue(vr::EVRButtonId::k_EButton_SteamVR_Touchpad);
                        KinectSettings::kinectRadRotation.v[0] += deltaScaled(3.0, deltaT) * axis.y;
                        //std::cerr << "ROT ADJUSTED R\n";
                    }
                    if (rightController.GetTrigger()) {
                        KinectSettings::adjustingKinectRepresentationRot = false;
                        guiRef.toggleRotButton();
                        rightController.setHapticPulse(.15, 1000, 0);
                    }
                }
            }


            kinect.updateTrackersWithSkeletonPosition(inputEmulator, v_trackers);

            //Draw
            kinect.drawKinectData(renderWindow);
        }

        
        renderWindow.pushGLStates();

        //Draw debug font
        debugText.setString(SFMLsettings::debugDisplayTextStream.str());
        renderWindow.draw(debugText);

        // Draw GUI
        renderWindow.setActive(true);

        guiRef.display(renderWindow);

        renderWindow.popGLStates();
        //End Frame
        renderWindow.display();

    }
    for (KinectTrackedDevice d : v_trackers) {
        d.destroy();
    }
    KinectSettings::writeKinectSettings();
    vr::VR_Shutdown();
}

void spawnAndConnectHandTrackers(vrinputemulator::VRInputEmulator & inputE, std::vector<KinectTrackedDevice>& v_trackers) {
    spawnAndConnectTracker(inputE, v_trackers, KVR::KinectJointType::WristLeft, KVR::KinectJointType::HandLeft, KinectDeviceRole::LeftHand);
    spawnAndConnectTracker(inputE, v_trackers, KVR::KinectJointType::WristRight, KVR::KinectJointType::HandRight, KinectDeviceRole::RightHand);
}

void spawnAndConnectTracker(vrinputemulator::VRInputEmulator & inputE, std::vector<KinectTrackedDevice>& v_trackers, KVR::KinectJointType mainJoint, KVR::KinectJointType secondaryJoint, KinectDeviceRole role)
{
    KinectTrackedDevice device(inputE, mainJoint, secondaryJoint, role);
	device.init(inputE);
    v_trackers.push_back(device);
}

void spawnDefaultLowerBodyTrackers(vrinputemulator::VRInputEmulator & inputE, std::vector<KinectTrackedDevice>& v_trackers)
{
    spawnAndConnectTracker(inputE, v_trackers, KVR::KinectJointType::AnkleLeft, KVR::KinectJointType::FootLeft, KinectDeviceRole::LeftFoot);
    spawnAndConnectTracker(inputE, v_trackers, KVR::KinectJointType::AnkleRight, KVR::KinectJointType::FootRight, KinectDeviceRole::RightFoot);
    spawnAndConnectTracker(inputE, v_trackers, KVR::KinectJointType::SpineBase, KVR::KinectJointType::SpineMid, KinectDeviceRole::Hip);
}

void spawnAndConnectKinectTracker(vrinputemulator::VRInputEmulator &inputE, std::vector<KinectTrackedDevice> &v_trackers)
{
    KinectTrackedDevice kinectTrackerRef(inputE, KVR::KinectJointType::Head, KVR::KinectJointType::Head, KinectDeviceRole::KinectSensor);
	kinectTrackerRef.init(inputE);
    setKinectTrackerProperties(kinectTrackerRef.deviceId);
    v_trackers.push_back(kinectTrackerRef);
}
