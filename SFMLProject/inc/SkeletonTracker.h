#pragma once

#include "TrackingMethod.h"
class SkeletonTracker : public TrackingMethod {
    // For now, always register the kinect FIRST, until there's some structure which binds joints and global id's
public:
    SkeletonTracker() {}
    ~SkeletonTracker() {}

    void initialise() {
        for (int i = 0; i < KVR::KinectJointCount; ++i) {
            KVR::TrackedDeviceInputData data;
            //data.parentHandler = dynamic_cast<DeviceHandler*>(this);
            data.deviceName = "K: " + std::to_string(i) + KVR::KinectJointName[i];
            uint32_t gID = k_invalidTrackerID;
            data.deviceId = gID;
            TrackingPoolManager::addDeviceToPool(data, gID);
            //v_controllers[i].id.globalID = gID;
        }
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
        // TODO
        // Iterate over trackers
        // Determine if they use kinect bones, update those bones only
        // Set flag to make sure bones aren't updated multiple times
        for (KVR::KinectTrackedDevice device : v_trackers) {
            if (device.joint0.joint == KVR::KinectJointType::INVALID
                || device.joint1.joint == KVR::KinectJointType::INVALID) {
                continue;
            }
            KVR::TrackedDeviceInputData data;
            int jointId = (int)device.joint0.joint;
            data.deviceName = "Active K: " + std::to_string(jointId)  + KVR::KinectJointName[jointId];
            data.deviceId = jointId;

            vr::HmdVector3d_t jointPosition{ 0,0,0 };
            vr::HmdQuaternion_t jointRotation{ 1,0,0,0 };
            kinect.getFilteredJoint(device, jointPosition, jointRotation);
            data.position = jointPosition;
            data.rotation = jointRotation;

            TrackingPoolManager::updatePoolWithDevice(data, jointId);
        }
    }

    void updateTrackers(
        KinectHandlerBase& kinect,
        std::vector<KVR::KinectTrackedDevice> & v_trackers,
        std::vector<KVR::TrackedDeviceInputData> inputs
    ) {
        for (int i = 0; i < v_trackers.size(); ++i) {
            auto & device = v_trackers[i];
            if (device.isSensor()) {
                device.update(KinectSettings::kinectRepPosition, { 0,0,0 }, KinectSettings::kinectRepRotation);
            }
            if (device.positionTrackingOption == KVR::JointPositionTrackingOption::Skeleton) {
                auto deviceData = TrackingPoolManager::getDeviceData(device.positionDevice_gId);
                device.setPositionForNextUpdate(deviceData.position);
            }
            if (device.rotationTrackingOption == KVR::JointRotationTrackingOption::Skeleton) {
                auto deviceData = TrackingPoolManager::getDeviceData(device.rotationDevice_gId);
                device.setRotationForNextUpdate(deviceData.rotation);
            }
        }
    }
};