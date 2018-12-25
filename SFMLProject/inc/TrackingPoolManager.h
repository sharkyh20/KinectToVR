#pragma once

#include "stdafx.h"

#include <vector>
#include <iostream>
#include <string>

#include "KinectTrackedDevice.h"
#include "TrackedDeviceInputData.h"

class TrackingPoolManager {
public:
    enum class TrackingPoolError {
        OK = 0,
        InsufficientSpace,
        OverwritingWrongDevice
    };

    static uint32_t leftFootDevicePosGID;
    static uint32_t rightFootDevicePosGID;
    static uint32_t leftFootDeviceRotGID;
    static uint32_t rightFootDeviceRotGID;

    static uint32_t locateGlobalDeviceID(std::string serial) {
        for (KVR::TrackedDeviceInputData & data : devicePool) {
            if (data.serial == serial) {
                return data.deviceId;
            }
        }
        return k_invalidTrackerID;
    }
    static TrackingPoolError addDeviceToPool(KVR::TrackedDeviceInputData & inputData, uint32_t & globalID) {
        globalID = devicePool.size(); // for the default case of adding instead of rebuilding

        // Some devices (PSMoves) rebuild the controller list, so essentially what's removed and readded needs to have that happen here too, but instead just zeroed out - otherwise the vector is screwed up.

        // Search for first available position to fill
        for (int i = 0; i < devicePool.size(); ++i) {
            if (devicePool[i].clearedForReinit) {
                globalID = i;
                inputData.deviceId = globalID;
                devicePool[i] = inputData;
                devicePool[i].clearedForReinit = false;
                return TrackingPoolError::OK;
            }
        }
        inputData.deviceId = globalID;
        devicePool.push_back(inputData);
        return TrackingPoolError::OK;
    }
    static TrackingPoolError clearDeviceInPool(uint32_t globalID) {
        // Should ideally be called directly before the devices are reinitialised in the pool
        devicePool[globalID].clearedForReinit = true;
        return TrackingPoolError::OK;
    }
    static TrackingPoolError updatePoolWithDevice(KVR::TrackedDeviceInputData inputData, uint32_t globalID) {
        if ((inputData.deviceName != devicePool[globalID].deviceName)
           || (inputData.deviceId != devicePool[globalID].deviceId)) {
            LOG(ERROR) << devicePool[globalID].deviceName << " IS BEING OVERWRITTEN BY " << inputData.deviceName << '\n';
            return TrackingPoolError::OverwritingWrongDevice;
        }
        devicePool[globalID] = inputData;
        return TrackingPoolError::OK;
    }
    static KVR::TrackedDeviceInputData getDeviceData(uint32_t globalID) {
        return devicePool[globalID];
    }
    static int count() {
        return devicePool.size();
    }
    static std::string deviceGuiString(uint32_t globalID) {
        // Have ID first, device name afters
        return "GID: " + std::to_string(devicePool[globalID].deviceId) + " " + devicePool[globalID].deviceName;
    }
private:
    // The global device tracking data pool - where every device allocates it's corresponding place by registering an id
    static std::vector<KVR::TrackedDeviceInputData> devicePool;
};