#include "stdafx.h"

#include "VRHelper.h"
#include <openvr_math.h>
#include "KinectSettings.h"

vr::HmdVector3_t updateHMDPosAndRot(vr::IVRSystem* &m_system) {
    //Gets the HMD location for relative position setting
    // Use the head joint for the zero location!
    vr::HmdVector3_t position{};
    const int HMD_INDEX = 0;

    vr::TrackedDevicePose_t hmdPose;
    vr::TrackedDevicePose_t devicePose[vr::k_unMaxTrackedDeviceCount];
    m_system->GetDeviceToAbsoluteTrackingPose(vr::ETrackingUniverseOrigin::TrackingUniverseStanding, 0, devicePose, 1);
    if (devicePose[HMD_INDEX].bPoseIsValid) {
        if (vr::VRSystem()->GetTrackedDeviceClass(HMD_INDEX) == vr::TrackedDeviceClass_HMD) {
            hmdPose = devicePose[HMD_INDEX];
            position = GetVRPositionFromMatrix(hmdPose.mDeviceToAbsoluteTracking);
            vr::HmdQuaternion_t quaternion = GetVRRotationFromMatrix(hmdPose.mDeviceToAbsoluteTracking);
            KinectSettings::hmdPosition = position;
            KinectSettings::hmdRotation = quaternion;
        }
    }
    return position;
}
// Get the quaternion representing the rotation
vr::HmdQuaternion_t GetVRRotationFromMatrix(vr::HmdMatrix34_t matrix) {
    // Credit to Omnifinity https://github.com/Omnifinity/OpenVR-Tracking-Example/
    vr::HmdQuaternion_t q;

    q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
    q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
    q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);
    return q;
}
// Get the vector representing the position
vr::HmdVector3_t GetVRPositionFromMatrix(vr::HmdMatrix34_t matrix) {
    // Credit to Omnifinity https://github.com/Omnifinity/OpenVR-Tracking-Example/
    vr::HmdVector3_t vector;

    vector.v[0] = matrix.m[0][3];
    vector.v[1] = matrix.m[1][3];
    vector.v[2] = matrix.m[2][3];

    return vector;
}

//void ResetUniverseOrigin(vr::ETrackingUniverseOrigin universe) {

//}

bool MoveUniverseOrigin(vr::HmdMatrix34_t& curPos, sf::Vector3f delta) {
	// Adjust direction of delta to match the universe forward direction.
	sf::Vector3f universeDelta = sf::Vector3f(
		curPos.m[0][0] * delta.x + curPos.m[0][1] * delta.y + curPos.m[0][2] * delta.z,
		curPos.m[1][0] * delta.x + curPos.m[1][1] * delta.y + curPos.m[1][2] * delta.z,
		curPos.m[2][0] * delta.x + curPos.m[2][1] * delta.y + curPos.m[2][2] * delta.z
	);
	curPos.m[0][3] += universeDelta.x;
	curPos.m[1][3] += universeDelta.y;
	curPos.m[2][3] += universeDelta.z;
	return true;
}