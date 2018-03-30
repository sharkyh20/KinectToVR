#pragma once
#include "stdafx.h"
//OpenGl and SFML
#include <SFML/Window/Event.hpp>
#include "KinectHandlerBase.h"
//VR
#include <openvr.h>

//void updateKinectTracker(vrinputemulator::VRInputEmulator &emulator, KinectTrackedDevice device);
void toEulerAngle(vr::HmdQuaternion_t q, double& roll, double& pitch, double& yaw);

void processKeyEvents(sf::Event event);
void toggle(bool &b);


void processLoop(KinectHandlerBase& kinect);


void spawnAndConnectHandTrackers(vrinputemulator::VRInputEmulator & inputE, std::vector<KVR::KinectTrackedDevice>& v_trackers);
void spawnAndConnectTracker(vrinputemulator::VRInputEmulator &inputE, std::vector<KVR::KinectTrackedDevice> &v_trackers, KVR::KinectJointType mainJoint, KVR::KinectJointType secondaryJoint, KVR::KinectDeviceRole role);
void spawnDefaultLowerBodyTrackers(vrinputemulator::VRInputEmulator &inputE, std::vector<KVR::KinectTrackedDevice> &v_trackers);
void spawnAndConnectKinectTracker(vrinputemulator::VRInputEmulator &inputE, std::vector<KVR::KinectTrackedDevice> &v_trackers);