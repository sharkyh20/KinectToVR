#pragma once

#include "stdafx.h"

#include "KinectHandlerBase.h"
#include "TrackingMethod.h"

class IMU_PositionMethod : public TrackingMethod {
public:
    IMU_PositionMethod() {}
    ~IMU_PositionMethod() {}

    void initialise() {}

    void activate() {}

    void terminate() {}

    void update(
        KinectHandlerBase& kinect,
        std::vector<KVR::KinectTrackedDevice> & v_trackers
    ) {

    }

    void updateTrackers(
        KinectHandlerBase& kinect,
        std::vector<KVR::KinectTrackedDevice> & v_trackers,
        std::vector<KVR::TrackedDeviceInputData> inputs
    ) {
        for (int i = 0; i < v_trackers.size(); ++i) {
            auto & device = v_trackers[i];
            if (device.positionTrackingOption != KVR::JointPositionTrackingOption::IMU) {
                continue;
            }
            if (device.isSensor()) {
                // Does not handle kinect representation
            }
            else {
                auto deviceData = TrackingPoolManager::getDeviceData(device.positionDevice_gId);
                
                // For now, whether or not the trackers are set via pose is up for debate
                //if (inputs[i].pose.poseIsValid) { 
                //    device.update(inputs[i].pose);
                //}
                //else {
                //    device.update(kinect.trackedPositionVROffset, inputs[i].position, inputs[i].rotation);
                //}

                device.setPositionForNextUpdate(deviceData.position);
            }
        }
    }
};