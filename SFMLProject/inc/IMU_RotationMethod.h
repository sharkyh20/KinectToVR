#pragma once
#include "../stdafx.h"

#include "KinectTrackedDevice.h"
#include "TrackingMethod.h"
#include "TrackingPoolManager.h"

class IMU_RotationMethod : public TrackingMethod {
public:
    IMU_RotationMethod() {}
    ~IMU_RotationMethod() {}

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
        std::vector<KVR::KinectTrackedDevice> & v_trackers
    ) {
        for (int i = 0; i < v_trackers.size(); ++i) {
            auto & device = v_trackers[i];
            if (device.rotationTrackingOption != KVR::JointRotationTrackingOption::IMU) {
                continue;
            }
            if (device.isSensor()) {
                // Does not handle kinect representation
            }
            else {
                auto deviceData = TrackingPoolManager::getDeviceData(device.rotationDevice_gId);

                // For now, whether or not the trackers are set via pose is up for debate
                //if (inputs[i].pose.poseIsValid) { 
                //    device.update(inputs[i].pose);
                //}
                //else {
                //    device.update(kinect.trackedPositionVROffset, inputs[i].position, inputs[i].rotation);
                //}

                device.setRotationForNextUpdate(deviceData.rotation);
            }
        }
    }
};