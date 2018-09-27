#pragma once
#include "stdafx.h"
#include <vector>

#include "KinectTrackedDevice.h"


static const uint32_t k_invalidTrackerID = 80808080;
struct TrackerIDs {
    uint32_t internalID = k_invalidTrackerID; // DeviceHandler Specific
    uint32_t globalID = k_invalidTrackerID; // Relative to the Pool
};

class DeviceHandler {
    // Physical Devices are based off of this class
    // I.e. PSmove controllers, JoyCons, etc
    // The class produces std::vector<KVR::TrackedDeviceInputData>
    // which can then be sent to the Tracking Methods, where the data is
    // bewiggled around for the final tracker updating

public:
    DeviceHandler() {}
    virtual ~DeviceHandler() {}

    virtual int initialise() { return 13; } // Return 13 if unimplemented
    virtual int run() { return 13; }
    virtual void shutdown() {  }

    virtual std::vector<KVR::TrackedDeviceInputData> extractVRTrackingPoses() { return {}; }
};