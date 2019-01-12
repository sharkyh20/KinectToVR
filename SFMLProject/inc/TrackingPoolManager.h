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

    static uint32_t kinectFirstId;
    static uint32_t kinectLastId;
    static uint32_t kinectJointToGlobalIDTable[KVR::KinectJointCount];

    static uint32_t kinectSensorGID;

    static bool findKinectGlobalIDRange() {
        static bool kinectIdLocated = false;
        static uint32_t firstId = k_invalidTrackerID;
        static uint32_t lastId = k_invalidTrackerID;
        if (!kinectIdLocated) {
            for (int i = 0; i < TrackingPoolManager::count(); ++i) {
                auto data = TrackingPoolManager::getDeviceData(i);
                if (data.positionTrackingOption == KVR::JointPositionTrackingOption::Skeleton) {
                    firstId = i;
                    // Kinect Trackers spawned all together, so no need to account for different devices's between this range
                    lastId = i + KVR::KinectJointCount - 1;

                    kinectIdLocated = true;
                    break;
                }
            }
        }
        kinectFirstId = firstId;
        kinectLastId = lastId;
        if (kinectIdLocated)
            return true;
        return false;
    }

    static bool trackerIdInKinectRange(uint32_t trackerId) {
        findKinectGlobalIDRange();
        return trackerId >= kinectFirstId && trackerId <= kinectLastId;
    }
    static uint32_t globalDeviceIDFromJoint(KVR::KinectJointType joint) {
        if (kinectFirstId == k_invalidTrackerID
            && kinectLastId == k_invalidTrackerID) {
            findKinectGlobalIDRange();
            for (int i = 0; i < KVR::KinectJointCount; ++i) {
                kinectJointToGlobalIDTable[i] = kinectFirstId + i;
            }
            return kinectJointToGlobalIDTable[(int)joint];
        } else
            return kinectJointToGlobalIDTable[(int)joint];
        return k_invalidTrackerID;
    }
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
        if (globalID >= 0 && globalID < devicePool.size())
            return devicePool[globalID];
        return KVR::TrackedDeviceInputData();
    }
    static int count() {
        return devicePool.size();
    }
    static std::string deviceGuiString(uint32_t globalID) {
        // Have ID first, device name afters
        if (globalID >= 0 && globalID < devicePool.size()) {
            return "GID: " + std::to_string(devicePool[globalID].deviceId) + " " + devicePool[globalID].deviceName;
        }
        return "ERROR: DEVICE ID OUTSIDE OF POOL RANGE";
    }
private:
    // The global device tracking data pool - where every device allocates it's corresponding place by registering an id
    static std::vector<KVR::TrackedDeviceInputData> devicePool;
};