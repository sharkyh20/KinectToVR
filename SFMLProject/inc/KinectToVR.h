#pragma once
#include "stdafx.h"
//OpenGl and SFML
#include <SFML/Window/Event.hpp>
#include "KinectHandlerBase.h"
#include "GUIHandler.h"
//VR
#include <openvr.h>

//void updateKinectTracker(vrinputemulator::VRInputEmulator &emulator, KinectTrackedDevice device);
void toEulerAngle(vr::HmdQuaternion_t q, double& roll, double& pitch, double& yaw);

void processKeyEvents(sf::Event event);
void toggle(bool &b);


void processLoop(KinectHandlerBase& kinect);

void updateFilePath();


void attemptInitialiseDebugDisplay(sf::Font font, sf::Text debugText) {
    // Global Debug Font
#if _DEBUG
    auto fontFileName = "arial.ttf";
    std::cout << "Attemping Debug Font Load: " << KVR::fileToDirPath(fontFileName) << '\n';
    font.loadFromFile(KVR::fileToDirPath(fontFileName));
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
void updateTrackerInitGuiSignals(vrinputemulator::VRInputEmulator &inputEmulator, GUIHandler &guiRef, std::vector<KVR::KinectTrackedDevice> v_trackers) {
    if (inputEmulator.isConnected()) {
        guiRef.setTrackerButtonSignals(inputEmulator, v_trackers);
        guiRef.updateEmuStatusLabelSuccess();
    }
    else {
        guiRef.updateTrackerInitButtonLabelFail();
    }
}