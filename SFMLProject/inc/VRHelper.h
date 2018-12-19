#pragma once
#include "stdafx.h"
#include "openvr.h"
#include <SFML/System/Vector3.hpp>
#include <vrinputemulator.h>


void setTrackerRolesInVRSettings();
void removeTrackerRolesInVRSettings();

void toEulerAngle(vr::HmdQuaternion_t q, double& pitch, double& yaw, double& roll);

vr::DriverPose_t defaultReadyDriverPose();
vr::DriverPose_t trackedDeviceToDriverPose(vr::TrackedDevicePose_t tPose);
vr::HmdVector3d_t getWorldPositionFromDriverPose(vr::DriverPose_t pose);

vr::HmdVector3d_t updateHMDPosAndRot(vr::IVRSystem* &m_sys);

// Get the quaternion representing the rotation
vr::HmdQuaternion_t GetVRRotationFromMatrix(vr::HmdMatrix34_t matrix);
// Get the vector representing the position
vr::HmdVector3d_t GetVRPositionFromMatrix(vr::HmdMatrix34_t matrix);
void translateAllDevicesWorldFromDriver(vrinputemulator::VRInputEmulator& inputEmulator, vr::HmdVector3d_t vec);
bool deviceIsVirtual(uint32_t deviceIndex, std::vector<uint32_t> virtualDeviceIndexes);
void translateRealDevicesWorldFromDriver(vrinputemulator::VRInputEmulator& inputEmulator, vr::HmdVector3d_t vec, std::vector<uint32_t> virtualDeviceIndexes);
void SetUniverseOrigin(const vr::HmdMatrix34_t& curPos, sf::Vector3f pos, vrinputemulator::VRInputEmulator& inputEmulator , std::vector<uint32_t> virtualDeviceIndexes);
void MoveUniverseOrigin(vr::HmdMatrix34_t& curPos, sf::Vector3f delta, vrinputemulator::VRInputEmulator& inputEmulator , std::vector<uint32_t> virtualDeviceIndexes);