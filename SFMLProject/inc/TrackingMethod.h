#pragma once

#include "stdafx.h"
#include <string>
#include <vector>

#include "KinectHandlerBase.h"
#include "KinectTrackedDevice.h"
#include "TrackedDeviceInputData.h"

class TrackingMethod {
public:
    TrackingMethod() {}
    virtual ~TrackingMethod() {}

    virtual void initialise() {}

    virtual void activate() {}

    virtual void terminate() {}

    virtual void update(
        KinectHandlerBase& kinect,
        std::vector<KVR::KinectTrackedDevice> & v_trackers
    ) {}

    virtual void updateTrackers(
        KinectHandlerBase& kinect, 
        std::vector<KVR::KinectTrackedDevice> & v_trackers
    ) {}


    bool isActive() { return active; }
protected:
    bool active = false;
};