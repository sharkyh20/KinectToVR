#pragma once


#include <openvr.h>

#include "DeviceHandler.h"
#include "TrackingPoolManager.h"
#include "TrackedDeviceInputData.h"
#include "VRHelper.h"

#include <vrinputemulator.h>

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
                // Needed because of IE's natural offsets
                
                data.pose.vecWorldFromDriverTranslation[0] -= KinectSettings::trackingOriginPosition.v[0];
                data.pose.vecWorldFromDriverTranslation[1] -= KinectSettings::trackingOriginPosition.v[1];
                data.pose.vecWorldFromDriverTranslation[2] -= KinectSettings::trackingOriginPosition.v[2];
                
                TrackingPoolManager::updatePoolWithDevice(data, vrDeviceToPoolIds[i].globalID);
            }
        }

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

    int update() {

    }

    KVR::TrackedDeviceInputData defaultDeviceData(uint32_t localID) {
        // Local ID is the same as SteamVR ID's considering it iterates over the tracked device array
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