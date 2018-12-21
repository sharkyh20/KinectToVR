#pragma once


#include <openvr.h>

#include "DeviceHandler.h"
#include "TrackingPoolManager.h"
#include "TrackedDeviceInputData.h"
#include "VRHelper.h"

#include <openvr_math.h>

#include <vrinputemulator.h>

namespace vrmath {
    double length_sq(vr::HmdVector3d_t v) {
        return 
            v.v[0] * v.v[0] +
            v.v[1] * v.v[1] +
            v.v[2] * v.v[2];
    }
    double length(vr::HmdVector3d_t v) {
        return sqrt(length_sq(v));
    }
    double length(vr::HmdQuaternion_t q) {
        return sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    }
    vr::HmdQuaternion_t normalized(vr::HmdQuaternion_t a) {
        vr::HmdQuaternion_t q = a;
        float magnitude = pow(length(q), 2);
        q.w /= magnitude;
        q.x /= magnitude;
        q.y /= magnitude;
        q.z /= magnitude;
        return q;
    }
        
    vr::HmdVector3d_t cross(vr::HmdVector3d_t v1, vr::HmdVector3d_t v2) {
        float x = (v1.v[1] * v2.v[2]) - (v1.v[2]*v2.v[1]);
        float y = -((v1.v[0]*v2.v[2]) - (v1.v[2]*v2.v[0]));
        float z = (v1.v[0]*v2.v[1]) - (v1.v[1]*v2.v[0]);
        return { x,y,z };
    }
    double dot(vr::HmdVector3d_t v1, vr::HmdVector3d_t v2) {
        return v1.v[0] * v2.v[0] + v1.v[1] * v2.v[1] + v1.v[2] * v2.v[2];
    }
    vr::HmdQuaternion_t get_rotation_between(vr::HmdVector3d_t u, vr::HmdVector3d_t v) {
        double k_cos_theta = dot(u, v);
        float k = sqrt(length_sq(u) * length_sq(v));

        if (k_cos_theta / k == -1)
        {
            // 180 degree rotation around any orthogonal vector
            return vr::HmdQuaternion_t{ 1, 0, 0, 0 };
        }
        auto vec = cross(u, v);
        return normalized(vr::HmdQuaternion_t{ k_cos_theta + k, vec.v[0], vec.v[1], vec.v[2] });
    }
    
}

enum class VirtualHipMode {
        Standing,
        Sitting,
        Lying
    };

struct VirtualHipSettings {
    bool followHmdYawRotation = true;
    bool followHmdRollRotation = false;
    bool followHmdPitchRotation = false;

    bool positionAccountsForFootTrackers = false; // If false, Hip tracker always stays bolted to directly under the HMD with no horizontal shift
    double positionLatency = 0.05; // Seconds. How far behind the head should the hips be so that they don't instantly follow every tiny movement of the HMD
    bool positionFollowsHMDLean = false; // Determines whether the virtual hips in standing mode will stay above the foot trackers, or interpolate between the HMD and foot trackers on a direct slant

    double heightFromHMD = 0.72; // Meters. Hips are by default projected downwards from the HMD, by 72cm (adjustable by user)
    VirtualHipMode hipMode = VirtualHipMode::Standing;
    double hipThickness = 0.03; // Meters. Essentially how wide the hips are, so that when lying down, they are put slightly above the ground

    double sittingMaxHeightThreshold = 0.95; // Meters. Under this height, mode is sitting
    double lyingMaxHeightThreshold = 0.5; // Meters. Under this height, mode is lying
};

class VRDeviceHandler : public DeviceHandler {
    // Updates the tracking pool with data from the 
    // non-IE SteamVR devices - e.g. head position/rotation
public:
    VRDeviceHandler(vr::IVRSystem* &g_VRSystem, vrinputemulator::VRInputEmulator &inputEmulator)
    : m_VRSystem(g_VRSystem),
        m_inputEmulator(inputEmulator)
    {

    }
    ~VRDeviceHandler() {}

    int initialise() {
        // Add all devices that aren't sensors or virtual

        vr::TrackedDevicePose_t devicePose[vr::k_unMaxTrackedDeviceCount];
        m_VRSystem->GetDeviceToAbsoluteTrackingPose(vr::ETrackingUniverseOrigin::TrackingUniverseStanding, 0, devicePose, vr::k_unMaxTrackedDeviceCount);

        updateVirtualDeviceList();

        for (int i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i) {
            // If device virtual, skip
            if (virtualDevices[i])
                continue;

            // If device has no useful pos/rot data, skip
            auto deviceClass = m_VRSystem->GetTrackedDeviceClass(i);
            if (deviceClass == vr::TrackedDeviceClass_Invalid)
                break; // No point iterating past the last device
            if (deviceClass == vr::TrackedDeviceClass_TrackingReference)
                continue;

            KVR::TrackedDeviceInputData data = defaultDeviceData(i);
            uint32_t globalID = k_invalidTrackerID;
            TrackingPoolManager::addDeviceToPool(data, globalID);

            vrDeviceToPoolIds[i].internalID = i;
            vrDeviceToPoolIds[i].globalID = globalID;
        }

        initVirtualHips();

        active = true;
        return 0;
    }
    int run() {
        
        vr::TrackedDevicePose_t devicePoses[vr::k_unMaxTrackedDeviceCount];
        m_VRSystem->GetDeviceToAbsoluteTrackingPose(vr::ETrackingUniverseOrigin::TrackingUniverseStanding, 0, devicePoses, vr::k_unMaxTrackedDeviceCount);

        for (int i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i) {
            vr::ETrackedDeviceClass deviceClass = m_VRSystem->GetTrackedDeviceClass(i);
            if (deviceClass == vr::TrackedDeviceClass_Invalid)
                break; // No point iterating past the last device

            if (devicePoses[i].bPoseIsValid && vrDeviceToPoolIds[i].globalID != k_invalidTrackerID) {
                vr::TrackedDevicePose_t pose = devicePoses[i];
                vr::HmdVector3d_t position{};
                vr::HmdQuaternion_t rotation{};
                position = GetVRPositionFromMatrix(pose.mDeviceToAbsoluteTracking);
                rotation = GetVRRotationFromMatrix(pose.mDeviceToAbsoluteTracking);

                KVR::TrackedDeviceInputData data = defaultDeviceData(i);
                data.deviceId = vrDeviceToPoolIds[i].globalID;
                data.position = position;
                data.rotation = rotation;

                data.pose = trackedDeviceToDriverPose(pose);
                data.pose.vecPosition[0] = position.v[0];
                data.pose.vecPosition[1] = position.v[1];
                data.pose.vecPosition[2] = position.v[2];
                data.pose.qRotation = rotation;

                TrackingPoolManager::updatePoolWithDevice(data, vrDeviceToPoolIds[i].globalID);
            }
        }

        updateVirtualHips();

        return 0;
    }

    void updateVirtualDeviceList() {
        int virtualDeviceCount = m_inputEmulator.getVirtualDeviceCount();
        memset(virtualDevices, false, sizeof virtualDevices);

        for (int i = 0; i < virtualDeviceCount; ++i) {
            vrinputemulator::VirtualDeviceInfo info = m_inputEmulator.getVirtualDeviceInfo(i);
            virtualDevices[info.openvrDeviceId] = true;
        }
    }
private:
    vr::IVRSystem* &m_VRSystem;
    vrinputemulator::VRInputEmulator & m_inputEmulator;

    int virtualDeviceCount = 0;
    bool virtualDevices[vr::k_unMaxTrackedDeviceCount]{ false };
    TrackerIDs vrDeviceToPoolIds[vr::k_unMaxTrackedDeviceCount]{};
    TrackerIDs virtualHipsIds{};
    uint32_t virtualHipsLocalId = 420;
    VirtualHipSettings hipSettings;
    
    void initVirtualHips() {
        KVR::TrackedDeviceInputData data = defaultDeviceData(virtualHipsLocalId);

        uint32_t globalID = k_invalidTrackerID;
        TrackingPoolManager::addDeviceToPool(data, globalID);

        virtualHipsIds.internalID = virtualHipsLocalId;
        virtualHipsIds.globalID = globalID;
    }
    bool footTrackersAvailable() {
        return
            TrackingPoolManager::leftFootDevicePosGID != k_invalidTrackerID &&
            TrackingPoolManager::rightFootDevicePosGID != k_invalidTrackerID &&
            TrackingPoolManager::leftFootDeviceRotGID != k_invalidTrackerID &&
            TrackingPoolManager::rightFootDeviceRotGID != k_invalidTrackerID;
    }
    void calculateHipMode() {
        // Determine user mode from HMD position
        if (KinectSettings::hmdPosition.v[1] <= hipSettings.lyingMaxHeightThreshold)
            hipSettings.hipMode = VirtualHipMode::Lying;
        else if (KinectSettings::hmdPosition.v[1] <= hipSettings.sittingMaxHeightThreshold)
            hipSettings.hipMode = VirtualHipMode::Sitting;
        else
            hipSettings.hipMode = VirtualHipMode::Standing;
    }
    vr::HmdVector3d_t getAverageFootPosition()
    {
        // Get average of feet controller positions
        // NEED TO TAKE INTO ACCOUNT DRIVER-WORLD OFFSET!
        auto leftData = TrackingPoolManager::getDeviceData(TrackingPoolManager::leftFootDevicePosGID);
        auto rightData = TrackingPoolManager::getDeviceData(TrackingPoolManager::rightFootDevicePosGID);

        vr::HmdVector3d_t leftPos = getWorldPositionFromDriverPose(leftData.pose);
        vr::HmdVector3d_t rightPos = getWorldPositionFromDriverPose(rightData.pose);

        return vr::HmdVector3d_t{
            (leftPos.v[0] + rightPos.v[0]) * 0.5,
            (leftPos.v[1] + rightPos.v[1]) * 0.5,
            (leftPos.v[2] + rightPos.v[2]) * 0.5
        };
    }
    void calculateStandingPosition(vr::HmdVector3d_t & hipPosition) {
        // Initially use head position, project downwards
        hipPosition = KinectSettings::hmdPosition;
        hipPosition.v[1] -= hipSettings.heightFromHMD; 

        if (hipSettings.positionAccountsForFootTrackers &&
            footTrackersAvailable()) {
            vr::HmdVector3d_t averageFeetPos = getAverageFootPosition();

            // Use their X and Z
            hipPosition.v[0] = averageFeetPos.v[0];
            hipPosition.v[2] = averageFeetPos.v[2];
        }
    }
    void calculateSittingPosition(vr::HmdVector3d_t & hipPosition) {
        // Initially use head position, project downwards
        hipPosition = KinectSettings::hmdPosition;
        hipPosition.v[1] -= hipSettings.heightFromHMD; 

        // Prevents sinking when head gets closer to ground
        if (hipPosition.v[1] <= hipSettings.hipThickness) {
            hipPosition.v[1] = hipSettings.hipThickness;
        }

        // Use the x,z average of all tracked points - for now, turned off as it didn't have the desired result
        /*
        if (hipSettings.positionAccountsForFootTrackers &&
            footTrackersAvailable()) {
            const vr::HmdVector3d_t & headPos = KinectSettings::hmdPosition;

            vr::HmdVector3d_t averageFeetPos = getAverageFootPosition();
            
            vr::HmdVector3d_t averageHipPos{
                (averageFeetPos.v[0] + headPos.v[0]) * 0.5,
                (averageFeetPos.v[1] + headPos.v[1]) * 0.5,
                (averageFeetPos.v[2] + headPos.v[2]) * 0.5
            };

            // Use their X and Z
            hipPosition.v[0] = averageFeetPos.v[0];
            hipPosition.v[2] = averageFeetPos.v[2];
        }
        else {
            // Stays only under the HMD
        }
        */
    }
    void calculateLyingPosition(vr::HmdVector3d_t & hipPosition) {
        // Initially use head position, project downwards
        hipPosition = KinectSettings::hmdPosition;
        hipPosition.v[1] -= hipSettings.heightFromHMD;

        // Prevents sinking when head gets closer to ground
        if (hipPosition.v[1] <= hipSettings.hipThickness) {
            hipPosition.v[1] = hipSettings.hipThickness;
        }

        // Move the tracker horizontally to simulate hip position when lying down
        // https://math.stackexchange.com/questions/83404/finding-a-point-along-a-line-in-three-dimensions-given-two-points

        if (hipSettings.positionAccountsForFootTrackers &&
            footTrackersAvailable()) {
            // Get the point 'd' units along the line from point 'A' to 'B'
            vr::HmdVector3d_t A = KinectSettings::hmdPosition;
            vr::HmdVector3d_t B = getAverageFootPosition();
            double ratio = 0.5; // Ratio for how far from the head to the feet the hips are
            double distanceAB = sqrt(
                (B.v[0] - A.v[0]) * (B.v[0] - A.v[0]) +
                (B.v[1] - A.v[1]) * (B.v[1] - A.v[1]) +
                (B.v[2] - A.v[2]) * (B.v[2] - A.v[2])
            );
            //double d = hipSettings.heightFromHMD
            double d = distanceAB * ratio;

            vr::HmdVector3d_t BA = B - A;

            // Normalise
            double length = sqrt(
                BA.v[0] * BA.v[0] +
                BA.v[1] * BA.v[1] +
                BA.v[2] * BA.v[2]);
            vr::HmdVector3d_t unitVector = (BA) / length;

            // Scale towards B
            vr::HmdVector3d_t desiredHipPosition = A + (unitVector * d);
            hipPosition.v[0] = desiredHipPosition.v[0];
            hipPosition.v[2] = desiredHipPosition.v[2];
        }
    }
    void updateVirtualHips() {
        // Has access to head point directly, (and controllers if necessary)
        // Needs feet points to be supplied in order to properly predict the hips

        calculateHipMode();

        // Calculate Position
        vr::HmdVector3d_t hipPosition{ 0 };

        switch (hipSettings.hipMode) {
        case VirtualHipMode::Standing: {
            calculateStandingPosition(hipPosition);
            //LOG(INFO) << "Standing";
            break;
        }
        case VirtualHipMode::Sitting: {
            calculateSittingPosition(hipPosition);
            //LOG(INFO) << "Sitting";
            break;
        }
        case VirtualHipMode::Lying: {
            calculateLyingPosition(hipPosition);
            //LOG(INFO) << "Lying";
            break;
        }
        default: {
            LOG(ERROR) << "Virtual Hip mode invalid!!!";
        }
        };

        // Calculate Rotation
        vr::HmdQuaternion_t rotation{ 1,0,0,0 };
        double yaw = 0;
        double pitch = 0;
        double roll = 0;
        toEulerAngle(KinectSettings::hmdRotation, pitch, yaw, roll);
        switch (hipSettings.hipMode) {
        case VirtualHipMode::Standing: {
            
            if (hipSettings.followHmdYawRotation) {
                rotation = vrmath::quaternionFromRotationY(yaw);
            }
            break;
        }
        case VirtualHipMode::Sitting: {
            // As the hip tracker sinks further into the ground, rotate the hip upwards
            // Apply yaw first, then pitch
            vr::HmdQuaternion_t yawRotation = { 1,0,0,0 };
            if (hipSettings.followHmdYawRotation) {
                yawRotation = vrmath::quaternionFromRotationY(yaw);
            }

            // Adjust up/down
            vr::HmdQuaternion_t pitchRotation = { 1,0,0,0 };
            const double maxRotation = M_PI / 4.0; // 45 degrees up

            double rotationRatio = (
                KinectSettings::hmdPosition.v[1] - hipSettings.heightFromHMD) 
                / (hipSettings.sittingMaxHeightThreshold - hipSettings.heightFromHMD);
            double radiansToRotatePitch = -( maxRotation * rotationRatio);
            pitchRotation = vrmath::quaternionFromRotationX(radiansToRotatePitch);
            
            rotation = pitchRotation * yawRotation; // Right side applied first
            break;
        }
        case VirtualHipMode::Lying: {
            // Follows HMD roll now, and yaw is based on lookat from HMD to feet
            vr::HmdQuaternion_t rollRotation = vrmath::quaternionFromRotationZ(roll);
            vr::HmdQuaternion_t pitchRotation = vrmath::quaternionFromRotationAxis(M_PI_2, 0, 0, 1);
            vr::HmdQuaternion_t yawRotation = { 1,0,0,0 };

            if (hipSettings.positionAccountsForFootTrackers &&
                footTrackersAvailable()) {
                // rotation between
                auto rawQ = vrmath::get_rotation_between(KinectSettings::hmdPosition, getAverageFootPosition());
                double lookAtYaw = 0;
                double lookAtPitch = 0;
                double lookAtRoll = 0;
                toEulerAngle(rawQ, lookAtPitch, lookAtYaw, lookAtRoll);

                yawRotation = vrmath::quaternionFromRotationY(lookAtYaw);
            }


            rotation = rollRotation * pitchRotation * yawRotation;
            break;
        }
        default: {
            LOG(ERROR) << "Virtual Hip mode invalid!!!";
        }
        };

        KVR::TrackedDeviceInputData data = defaultDeviceData(virtualHipsLocalId);
        data.deviceId = virtualHipsIds.globalID;
        data.position = hipPosition;
        data.rotation = rotation;

        data.pose = defaultReadyDriverPose();
        data.pose.vecPosition[0] = hipPosition.v[0];
        data.pose.vecPosition[1] = hipPosition.v[1];
        data.pose.vecPosition[2] = hipPosition.v[2];
        data.pose.qRotation = rotation;

        TrackingPoolManager::updatePoolWithDevice(data, virtualHipsIds.globalID);
    }

    KVR::TrackedDeviceInputData defaultDeviceData(uint32_t localID) {
        // Local ID is the same as SteamVR ID's considering it iterates over the tracked device array
        if (localID == virtualHipsLocalId) {
            KVR::TrackedDeviceInputData data;

            data.positionTrackingOption = KVR::JointPositionTrackingOption::IMU;
            data.rotationTrackingOption = KVR::JointRotationTrackingOption::IMU;
            data.parentHandler = dynamic_cast<DeviceHandler*>(this);

            data.deviceName = "Virtual Hips";
            data.deviceId = virtualHipsLocalId;
            data.customModelName = "vr_controller_01_mrhat";

            return data;
        }

        KVR::TrackedDeviceInputData data;
        data.positionTrackingOption = KVR::JointPositionTrackingOption::IMU;
        data.rotationTrackingOption = KVR::JointRotationTrackingOption::IMU;
        data.parentHandler = dynamic_cast<DeviceHandler*>(this);
        
        // Courtesy of https://steamcommunity.com/app/358720/discussions/0/1353742967802223832/
        std::string modelName = "";

        vr::ETrackedPropertyError peError;
        uint32_t unRequiredBufferLen = m_VRSystem->GetStringTrackedDeviceProperty(localID, vr::Prop_ModelNumber_String, nullptr, 0, &peError);
        if (unRequiredBufferLen == 0) {
            modelName = "";
        }
        else {
            char* pchBuffer = new char[unRequiredBufferLen];
            unRequiredBufferLen = m_VRSystem->GetStringTrackedDeviceProperty(localID, vr::Prop_ModelNumber_String, pchBuffer, unRequiredBufferLen, &peError);
            modelName = pchBuffer;
            delete[] pchBuffer;
        }

        data.deviceName = "SteamVR ID: " + std::to_string(localID) + " " + modelName;
        data.deviceId = localID;
        data.customModelName = "vr_controller_vive_1_5";
        return data;
    }
};