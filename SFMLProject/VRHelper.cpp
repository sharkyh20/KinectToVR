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
            KinectSettings::hmdAbsoluteTracking = hmdPose.mDeviceToAbsoluteTracking;
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
void translateAllDevicesWorldFromDriver(vrinputemulator::VRInputEmulator& inputEmulator, vr::HmdVector3d_t vec) {
    vr::TrackedDevicePose_t devicePoses[vr::k_unMaxTrackedDeviceCount];
    vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, devicePoses, vr::k_unMaxTrackedDeviceCount);
    for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
        if (!devicePoses[i].bDeviceIsConnected) {
            continue;
        }
        inputEmulator.enableDeviceOffsets(i, true);
        inputEmulator.setWorldFromDriverTranslationOffset(i, vec);
    }
}
bool deviceIsVirtual(uint32_t deviceIndex, std::vector<uint32_t> virtualDeviceIndexes) {
    if (virtualDeviceIndexes.empty()) return false;
    return std::find(virtualDeviceIndexes.begin(), virtualDeviceIndexes.end(), deviceIndex) != virtualDeviceIndexes.end();
}
void translateRealDevicesWorldFromDriver(vrinputemulator::VRInputEmulator& inputEmulator, vr::HmdVector3d_t vec, std::vector<uint32_t> virtualDeviceIndexes) {
    vr::TrackedDevicePose_t devicePoses[vr::k_unMaxTrackedDeviceCount];
    vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, devicePoses, vr::k_unMaxTrackedDeviceCount);
    for (uint32_t deviceIndex = 0; deviceIndex < vr::k_unMaxTrackedDeviceCount; deviceIndex++) {
        if (!devicePoses[deviceIndex].bDeviceIsConnected) {
            continue;
        }
        if (deviceIsVirtual(deviceIndex, virtualDeviceIndexes)) {
            //The virtual stuff is differently scaled than the physical stuff - may need to look into this, as the value might change with changes to the Kinect tracking
            vr::HmdVector3d_t adjustedVec;
            adjustedVec.v[0] = vec.v[0] * 0.5f;
            adjustedVec.v[1] = vec.v[1] * 0.5f;
            adjustedVec.v[2] = vec.v[2] * 0.5f;
            inputEmulator.enableDeviceOffsets(deviceIndex, true);
            inputEmulator.setWorldFromDriverTranslationOffset(deviceIndex, adjustedVec);
        }
        else {
            inputEmulator.enableDeviceOffsets(deviceIndex, true);
            inputEmulator.setWorldFromDriverTranslationOffset(deviceIndex, vec);
        }
    }
}

 
void SetUniverseOrigin(const vr::HmdMatrix34_t& curPos, sf::Vector3f pos, vrinputemulator::VRInputEmulator& inputEmulator, std::vector<uint32_t> virtualDeviceIndexes) {
    if (pos == sf::Vector3f(0, 0, 0)) {
        translateRealDevicesWorldFromDriver(inputEmulator, { 0,0,0 }, virtualDeviceIndexes);
    }
    else {
        sf::Vector3f universePos = sf::Vector3f(
            curPos.m[0][0] * pos.x + curPos.m[0][1] * pos.y + curPos.m[0][2] * pos.z,
            curPos.m[1][0] * pos.x + curPos.m[1][1] * pos.y + curPos.m[1][2] * pos.z,
            curPos.m[2][0] * pos.x + curPos.m[2][1] * pos.y + curPos.m[2][2] * pos.z
        );
        vr::HmdVector3d_t vec;
        vec.v[0] = -curPos.m[0][3];
        vec.v[1] = -curPos.m[1][3];
        vec.v[2] = -curPos.m[2][3];

        translateRealDevicesWorldFromDriver(inputEmulator, vec, virtualDeviceIndexes);
    }
}

void MoveUniverseOrigin(vr::HmdMatrix34_t& curPos, sf::Vector3f delta, vrinputemulator::VRInputEmulator& inputEmulator, std::vector<uint32_t> virtualDeviceIndexes) {
    // Adjust direction of delta to match the universe forward direction.
    sf::Vector3f universeDelta = sf::Vector3f(
        curPos.m[0][0] * delta.x + curPos.m[0][1] * delta.y + curPos.m[0][2] * delta.z,
        curPos.m[1][0] * delta.x + curPos.m[1][1] * delta.y + curPos.m[1][2] * delta.z,
        curPos.m[2][0] * delta.x + curPos.m[2][1] * delta.y + curPos.m[2][2] * delta.z
    );
    curPos.m[0][3] += universeDelta.x;
    curPos.m[1][3] += universeDelta.y;
    curPos.m[2][3] += universeDelta.z;
    vr::HmdVector3d_t vec;
    vec.v[0] = -curPos.m[0][3];
    vec.v[1] = -curPos.m[1][3];
    vec.v[2] = -curPos.m[2][3];

    translateRealDevicesWorldFromDriver(inputEmulator, vec, virtualDeviceIndexes);
}