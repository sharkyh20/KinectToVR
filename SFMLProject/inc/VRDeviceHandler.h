#pragma once


#include <openvr.h>

#include "DeviceHandler.h"
#include "TrackingPoolManager.h"
#include "TrackedDeviceInputData.h"
#include "VRHelper.h"

#include <vrinputemulator.h>

enum class VirtualHipMode {
        Standing,
        Sitting,
        Lying
    };

struct VirtualHipSettings {
    bool followHmdYawRotation = true;
    bool followHmdRollRotation = false;
    bool followHmdPitchRotation = false;

    bool positionAccountsForFootTrackers = true; // If false, Hip tracker always stays bolted to directly under the HMD with no horizontal shift
    double positionLatency = 0.05; // Seconds. How far behind the head should the hips be so that they don't instantly follow every tiny movement of the HMD
    bool positionFollowsHMDLean = false; // Determines whether the virtual hips in standing mode will stay above the foot trackers, or interpolate between the HMD and foot trackers on a direct slant

    double heightFromHMD = -0.72; // Meters. Hips are by default projected downwards from the HMD, by 72cm (adjustable by user)
    VirtualHipMode hipMode = VirtualHipMode::Standing;
    double hipThickness = 0.1; // Meters. Essentially how wide the hips are, so that when lying down, they are put slightly above the ground

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
    void calculateStandingPosition(vr::HmdVector3d_t & hipPosition) {
        // Initially use head position, project downwards
        hipPosition = KinectSettings::hmdPosition;
        hipPosition.v[1] += hipSettings.heightFromHMD; // height -ve

        if (hipSettings.positionAccountsForFootTrackers &&
            footTrackersAvailable()) {
            // Get average of feet controller positions
            // NEED TO TAKE INTO ACCOUNT DRIVER-WORLD OFFSET!
            auto leftData = TrackingPoolManager::getDeviceData(TrackingPoolManager::leftFootDevicePosGID);
            auto rightData = TrackingPoolManager::getDeviceData(TrackingPoolManager::rightFootDevicePosGID);

            // Doesn't actually work properly here, considering the raw position is already in proper vr units, and the world from driver is literally just for IE
            vr::HmdVector3d_t leftPos = getWorldPositionFromDriverPose(leftData.pose);
            vr::HmdVector3d_t rightPos = getWorldPositionFromDriverPose(rightData.pose);
            
            /*
            vr::HmdVector3d_t leftPos = { leftData.pose.vecPosition[0], leftData.pose.vecPosition[1], leftData.pose.vecPosition[2] };
            vr::HmdVector3d_t rightPos = { rightData.pose.vecPosition[0], rightData.pose.vecPosition[1], rightData.pose.vecPosition[2] };
            */
            vr::HmdVector3d_t averageFeetPos{
                (leftPos.v[0] + rightPos.v[0]) * 0.5,
                (leftPos.v[1] + rightPos.v[1]) * 0.5,
                (leftPos.v[2] + rightPos.v[2]) * 0.5
            };

            // Use their X and Z
            hipPosition.v[0] = averageFeetPos.v[0];
            hipPosition.v[2] = averageFeetPos.v[2];
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
            break;
        }
        case VirtualHipMode::Sitting: {
            // TODO
            break;
        }
        case VirtualHipMode::Lying: {
            // TODO
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
            //TODO
            break;
        }
        case VirtualHipMode::Lying: {
            //TODO
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