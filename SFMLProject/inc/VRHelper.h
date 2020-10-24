#pragma once
#include "stdafx.h"
#include "openvr.h"
#include <openvr_math.h>
#include <SFML/System/Vector3.hpp>
#include <vrinputemulator.h>

namespace vrmath
{
	double length_sq(vr::HmdVector3d_t v);
	double length(vr::HmdVector3d_t v);
	double length(vr::HmdQuaternion_t q);
	vr::HmdQuaternion_t normalized(vr::HmdQuaternion_t a);
	float norm_squared(vr::HmdQuaternion_t x);
	vr::HmdQuaternion_t divide(const vr::HmdQuaternion_t& x, float k);
	vr::HmdQuaternion_t inverse(vr::HmdQuaternion_t x);

	vr::HmdVector3d_t cross(vr::HmdVector3d_t v1, vr::HmdVector3d_t v2);
	double dot(vr::HmdVector3d_t v1, vr::HmdVector3d_t v2);
	vr::HmdQuaternion_t get_rotation_between(vr::HmdVector3d_t u, vr::HmdVector3d_t v);
}

void setTrackerRolesInVRSettings();
void removeTrackerRolesInVRSettings();

void toEulerAngle(vr::HmdQuaternion_t q, double& pitch, double& yaw, double& roll);

vr::DriverPose_t defaultReadyDriverPose();
vr::DriverPose_t trackedDeviceToDriverPose(vr::TrackedDevicePose_t tPose);
vr::HmdVector3d_t getWorldPositionFromDriverPose(vr::DriverPose_t pose);

vr::HmdVector3d_t updateHMDPosAndRot(vr::IVRSystem* & m_sys);

// Get the quaternion representing the rotation
vr::HmdQuaternion_t GetVRRotationFromMatrix(vr::HmdMatrix34_t matrix);
// Get the vector representing the position
vr::HmdVector3d_t GetVRPositionFromMatrix(vr::HmdMatrix34_t matrix);
void translateAllDevicesWorldFromDriver(vrinputemulator::VRInputEmulator& inputEmulator, vr::HmdVector3d_t vec);
bool deviceIsVirtual(uint32_t deviceIndex, std::vector<uint32_t> virtualDeviceIndexes);
void translateRealDevicesWorldFromDriver(vrinputemulator::VRInputEmulator& inputEmulator, vr::HmdVector3d_t vec,
                                         std::vector<uint32_t> virtualDeviceIndexes);
void SetUniverseOrigin(const vr::HmdMatrix34_t& curPos, sf::Vector3f pos,
                       vrinputemulator::VRInputEmulator& inputEmulator, std::vector<uint32_t> virtualDeviceIndexes);
void MoveUniverseOrigin(vr::HmdMatrix34_t& curPos, sf::Vector3f delta, vrinputemulator::VRInputEmulator& inputEmulator,
                        std::vector<uint32_t> virtualDeviceIndexes);
