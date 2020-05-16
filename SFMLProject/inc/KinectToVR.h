#pragma once
#include "stdafx.h"
//OpenGl and SFML
#include <SFML/Window/Event.hpp>
#include "KinectHandlerBase.h"
//VR
#include <openvr.h>

//void updateKinectTracker(vrinputemulator::VRInputEmulator &emulator, KinectTrackedDevice device);
void processKeyEvents(sf::Event event);
void toggle(bool &b);


void limitVRFramerate(double &endTimeMilliseconds, std::stringstream &ss);

void processLoop(KinectHandlerBase& kinect);

void updateFilePath();

void spawnAndConnectTracker(vrinputemulator::VRInputEmulator & inputE, std::vector<KVR::KinectTrackedDevice>& v_trackers, uint32_t posDevice_gId,
    uint32_t rotDevice_gId, KVR::KinectDeviceRole role);
void spawnAndConnectTracker(vrinputemulator::VRInputEmulator & inputE, std::vector<KVR::KinectTrackedDevice>& v_trackers, KVR::KinectJointType mainJoint, KVR::KinectJointType secondaryJoint, KVR::KinectDeviceRole role);
void spawnAndConnectHandTrackers(vrinputemulator::VRInputEmulator & inputE, std::vector<KVR::KinectTrackedDevice>& v_trackers);
void spawnDefaultLowerBodyTrackers();
void spawnAndConnectKinectTracker(vrinputemulator::VRInputEmulator &inputE, std::vector<KVR::KinectTrackedDevice> &v_trackers);
