#pragma once

#include "TrackingMethod.h"
class SkeletonTracker : public TrackingMethod {
public:
    SkeletonTracker() {}
    ~SkeletonTracker() {}

    void initialise() {
        //TODO
    }

    void activate() {
        //TODO
    }

    void terminate() {
        //TODO
    }

    void update(
        KinectHandlerBase& kinect,
        std::vector<KVR::KinectTrackedDevice> & v_trackers
    ) {
        // TODO, take skeleton positions for each 
        
    }

    void updateTrackers(
        KinectHandlerBase& kinect,
        std::vector<KVR::KinectTrackedDevice> & v_trackers,
        std::vector<KVR::TrackedDeviceInputData> inputs
    ) {
        for (KVR::KinectTrackedDevice device : v_trackers) {
            if (device.positionTrackingOption == KVR::JointPositionTrackingOption::Skeleton) {
                if (device.isSensor()) {
                    device.update(KinectSettings::kinectRepPosition, { 0,0,0 }, KinectSettings::kinectRepRotation);
                }
                else {
                    vr::HmdVector3d_t jointPosition{ 0,0,0 };
                    vr::HmdQuaternion_t jointRotation{ 0,0,0,0 };
                    if (kinect.getFilteredJoint(device, jointPosition, jointRotation)) {

                        device.update(kinect.trackedPositionVROffset, jointPosition, jointRotation);
                    }
                }
            }
        }
    }
};