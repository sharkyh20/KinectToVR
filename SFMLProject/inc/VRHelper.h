#pragma once
#include "stdafx.h"
#include "openvr.h"
#include <SFML/System/Vector3.hpp>

vr::HmdVector3_t getHMDPosition(vr::IVRSystem* &m_sys);

// Get the quaternion representing the rotation
vr::HmdQuaternion_t GetVRRotationFromMatrix(vr::HmdMatrix34_t matrix);
// Get the vector representing the position
vr::HmdVector3_t GetVRPositionFromMatrix(vr::HmdMatrix34_t matrix);

void MoveUniverseOrigin(vr::ETrackingUniverseOrigin universe, sf::Vector3f delta);