#pragma once

#include "stdafx.h"
#include <string>
#include <vector>

#include "KinectHandlerBase.h"
#include "KinectTrackedDevice.h"

class TrackingMethod {
public:
    TrackingMethod() {}
    ~TrackingMethod() {}

    void initialise() {}

    void activate() {}

    void terminate() {}

    void update(
        KinectHandlerBase& kinect,
        std::vector<KVR::KinectTrackedDevice> & v_trackers
    ) {}

    void updateTrackers(
        KinectHandlerBase& kinect, 
        std::vector<KVR::KinectTrackedDevice> & v_trackers
    ) {}


    bool isActive() { return active; }
protected:
    bool active = false;
};