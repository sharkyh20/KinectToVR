#pragma once
#include "stdafx.h"
#include <vector>

#include "KinectTrackedDevice.h"
#include "TrackedDeviceInputData.h"

static const uint32_t k_invalidTrackerID = 80808080;
struct TrackerIDs {
    uint32_t internalID = k_invalidTrackerID; // DeviceHandler Specific
    uint32_t globalID = k_invalidTrackerID; // Relative to the Pool
};


enum class TrackingInputCategory {
    KinectBone, // Left foot bone...
    PhysicalDevice, // PSMove, Joycon...
    VR_Device // Head, left/right controller...
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
    virtual void identify(int controllerId, bool on) { } // Shows the user which device is being referred to, i.e. flashing light, rumble for a short while

    bool active = false; // By default, device trackers should be off, and spawned into the vector, but disabled - initialising them flips the bool, and allows them to be updated

    std::string identifyDeviceString = "Identify Me!"; // String used for the button to show for the identify function in the GUI - can be overidden by a device handler to fit the method used

    //virtual std::vector<KVR::TrackedDeviceInputData> extractVRTrackingPoses() { return {}; }
};