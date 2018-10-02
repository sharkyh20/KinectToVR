#pragma once
#include "../stdafx.h"
#include <string>
#include <openvr.h>

#include "DeviceHandler.h"
namespace KVR {
    struct TrackedDeviceInputData {
        // Used by tracking methods to say what the desired position/rotation for this device should be
        // All data should be converted to VR coords within the device handler
        // Only data ready for tracking use is put into these structs
        bool clearedForReinit = false; // Flag for devices that like to clear themselves once a new one is added
        std::string deviceName = "UNSET_DEVICE_DATA";
        uint32_t deviceId = 0;

        vr::HmdQuaternion_t rotation = { 1, 0, 0, 0 };
        KVR::JointRotationTrackingOption rotationTrackingOption = KVR::JointRotationTrackingOption::IMU;

        vr::HmdVector3d_t position = { 0, 0, 0 };
        KVR::JointPositionTrackingOption positionTrackingOption = KVR::JointPositionTrackingOption::IMU;

        // Pose left invalid if not in use
        vr::DriverPose_t pose;

        DeviceHandler *parentHandler;
    };
}