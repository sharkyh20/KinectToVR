#pragma once
#include "stdafx.h"
//OpenGl and SFML
#include <SFML/Window/Event.hpp>
#include "KinectHandlerBase.h"
//VR
#include <openvr.h>

//void updateKinectTracker(vrinputemulator::VRInputEmulator &emulator, KinectTrackedDevice device);
void toEulerAngle(vr::HmdQuaternion_t q, double& roll, double& pitch, double& yaw);

void setKinectTrackerProperties(uint32_t deviceId); //TODO: TEMP HOLD BEFORE GETTING MULTIPLE ORIENTATIONS TO WORK

void processKeyEvents(sf::Event event);
void toggle(bool &b);

double deltaScaled(double valuePerSecond, double delta);

void processLoop(KinectHandlerBase& kinect);

void spawnAndConnectTracker(vrinputemulator::VRInputEmulator &inputE, std::vector<KinectTrackedDevice> &v_trackers, KinectJointType mainJoint, KinectJointType secondaryJoint);
void spawnDefaultFullBodyTrackers(vrinputemulator::VRInputEmulator &inputE, std::vector<KinectTrackedDevice> &v_trackers);
void spawnAndConnectKinectTracker(vrinputemulator::VRInputEmulator &inputE, std::vector<KinectTrackedDevice> &v_trackers);