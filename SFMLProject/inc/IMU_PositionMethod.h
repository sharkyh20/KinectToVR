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
            auto device = v_trackers[i];
            if (device.isSensor()) {
                device.update(KinectSettings::kinectRepPosition, { 0,0,0 }, KinectSettings::kinectRepRotation);
            }
            else {
                vr::HmdVector3d_t jointPosition{ 0,0,0 };
                vr::HmdQuaternion_t jointRotation{ 0,0,0,0 };

                if (true) {
                    if (inputs[i].pose.poseIsValid) {
                        //printf("ROTATION of %i: %f, %f, %f, %f\n", 0, inputs[0].pose.qRotation.w, inputs[0].pose.qRotation.x, inputs[0].pose.qRotation.y, inputs[0].pose.qRotation.z);
                        device.update(kinect.trackedPositionVROffset, { inputs[i].pose.vecPosition[0], inputs[i].pose.vecPosition[1] , inputs[i].pose.vecPosition[2] }, inputs[i].pose.qRotation);
                    }
                    else {
                        //device.update(kinect.trackedPositionVROffset, inputs[i].position, inputs[i].rotation);
                    }
                }
            }
        }
    }
};