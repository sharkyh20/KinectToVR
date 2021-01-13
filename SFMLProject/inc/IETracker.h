#pragma once
#include "stdafx.h"
#include <vrinputemulator.h>
//VR Tracking
uint32_t initTracker(vrinputemulator::VRInputEmulator& inputEmulator, bool connected);


void setTrackerDefaultProperties(vrinputemulator::VRInputEmulator& ie, uint32_t& deviceId);
void setDeviceProperty(vrinputemulator::VRInputEmulator& ie, uint32_t deviceId, int dProp, std::string type,
                       std::string value);
void removeAllTrackerProperties(vrinputemulator::VRInputEmulator& ie, uint32_t& deviceId);
void removeDeviceProperty(vrinputemulator::VRInputEmulator& ie, uint32_t deviceId, int dProp);
void destroyTracker(vrinputemulator::VRInputEmulator& inputEmulator, uint32_t ID);
void setKinectTrackerProperties(vrinputemulator::VRInputEmulator& ie, uint32_t deviceId);
