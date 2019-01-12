#pragma once

#include "TrackingMethod.h"
class SkeletonTracker : public TrackingMethod {
    // For now, always register the kinect FIRST, until there's some structure which binds joints and global id's
public:
    SkeletonTracker() {}
    ~SkeletonTracker() {}

    uint32_t kinectJointGIDs[KVR::KinectJointCount]{ k_invalidTrackerID };

    void initialise() {
        for (int i = 0; i < KVR::KinectJointCount; ++i) {
            KVR::TrackedDeviceInputData data = defaultDeviceData(i);
            uint32_t gID = k_invalidTrackerID;
            TrackingPoolManager::addDeviceToPool(data, gID);
            kinectJointGIDs[i] = gID;
        }
        // Kinect Sensor
        KVR::TrackedDeviceInputData data = defaultSensorDeviceData();
        uint32_t gID = k_invalidTrackerID;
        TrackingPoolManager::addDeviceToPool(data, gID);
        TrackingPoolManager::kinectSensorGID = gID;
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
            if (device.role == KVR::KinectDeviceRole::KinectSensor)
                updatePoolWithKinectSensor(device);
            else
                updatePoolWithKinectJoint(kinect, device);
        }
    }

    void updateTrackers(
        KinectHandlerBase& kinect,
        std::vector<KVR::KinectTrackedDevice> & v_trackers
    ) {
        for (int i = 0; i < v_trackers.size(); ++i) {
            auto & device = v_trackers[i];
            if (device.isSensor()) {
                auto deviceData = TrackingPoolManager::getDeviceData(device.positionDevice_gId);
                device.setPositionForNextUpdate(deviceData.position);
                device.setRotationForNextUpdate(deviceData.rotation);
                device.setPoseForNextUpdate(deviceData.pose);
                break;
            }
            if (device.positionTrackingOption == KVR::JointPositionTrackingOption::Skeleton) {
                auto deviceData = TrackingPoolManager::getDeviceData(device.positionDevice_gId);
                device.setPositionForNextUpdate(deviceData.position);
            }
            if (device.rotationTrackingOption == KVR::JointRotationTrackingOption::Skeleton) {
                auto deviceData = TrackingPoolManager::getDeviceData(device.rotationDevice_gId);
                device.setRotationForNextUpdate(deviceData.rotation);
            }
            if (deviceUsesJointPose(device)) {
                auto deviceData = TrackingPoolManager::getDeviceData(device.positionDevice_gId);
                device.setPoseForNextUpdate(deviceData.pose);
            }
        }
    }
    bool deviceUsesJointPose(KVR::KinectTrackedDevice & device)
    {
        return device.positionTrackingOption == KVR::JointPositionTrackingOption::Skeleton
            &&
            device.rotationTrackingOption == KVR::JointRotationTrackingOption::Skeleton
            &&
            device.positionDevice_gId == device.rotationDevice_gId;
    }
private:
    KVR::TrackedDeviceInputData defaultDeviceData(uint32_t localID) {
        KVR::TrackedDeviceInputData data;
        data.deviceName = "KID: " + std::to_string(localID) + " " + KVR::KinectJointName[localID];
        data.deviceId = kinectJointGIDs[localID];
        data.serial = "KINECT_JOINT_" + std::to_string(localID);
        data.customModelName = "vr_controller_vive_1_5";
        data.positionTrackingOption = KVR::JointPositionTrackingOption::Skeleton;
        data.rotationTrackingOption = KVR::JointRotationTrackingOption::Skeleton;
        
        return data;
    }
    const KVR::TrackedDeviceInputData defaultSensorDeviceData() {
        KVR::TrackedDeviceInputData data;
        data.deviceName = "Kinect Sensor";
        data.deviceId = TrackingPoolManager::kinectSensorGID;
        data.serial = "KINECT_SENSOR";
        data.customModelName = "arrow";
        data.positionTrackingOption = KVR::JointPositionTrackingOption::Skeleton;
        data.rotationTrackingOption = KVR::JointRotationTrackingOption::Skeleton;

        return data;
    }
    bool sensorShouldSkipUpdate(KVR::KinectTrackedDevice & device) {
        // The sensor doesn't actually need to be updated more often than not,
        // and by only updating it after the config changes, there's a pretty large
        // perf/responsiveness benefit to the trackers
        if (device.isSensor()) {
            using namespace KinectSettings;
            if (sensorConfigChanged) { // Sensor manually updated
                sensorConfigChanged = false;
                return false;
            }
            if (adjustingKinectRepresentationPos
                || adjustingKinectRepresentationRot) // Still adjusting the sensor
                return false;

            return true;
        }
        return false;
    }
    void applyKinectArrowCalibrationToTracker(KVR::KinectTrackedDevice & device, vr::HmdQuaternion_t &rotation, vr::HmdVector3d_t &position) {
        // If using the kinect, then the position/rot of the tracker has to be adjusted
        // according to the arrow, to transform it into the proper OpenVR coords

        //Rotate the position around the kinect rep's rot
        if (device.isSensor()) {
            //Rotate arrow by 180
            rotation = rotation * vrmath::quaternionFromRotationY(PI);
        }

        vr::HmdVector3d_t rotatedPos = vrmath::quaternionRotateVector(KinectSettings::kinectRepRotation, position, false);

        //Adjust this position by the Kinect's VR pos offset
        rotatedPos.v[0] += KinectSettings::kinectRepPosition.v[0];
        rotatedPos.v[1] += KinectSettings::kinectRepPosition.v[1];
        rotatedPos.v[2] += KinectSettings::kinectRepPosition.v[2];

        if (device.rotationFilterOption == KVR::JointRotationFilterOption::HeadLook) {}
        else
            rotation = KinectSettings::kinectRepRotation * rotation;
        position = rotatedPos;
    }
    void updateDevicePosePosition(KVR::KinectTrackedDevice & device, vr::DriverPose_t &pose, vr::HmdVector3d_t rotatedPos) {
        pose.vecPosition[0] = rotatedPos.v[0] + device.trackedPositionVROffset.v[0];
        pose.vecPosition[1] = rotatedPos.v[1] + device.trackedPositionVROffset.v[1];
        pose.vecPosition[2] = rotatedPos.v[2] + device.trackedPositionVROffset.v[2];
        if (device.role == KVR::KinectDeviceRole::Hip) {
            pose.vecPosition[1] += KinectSettings::hipRoleHeightAdjust;
        }
    }
    vr::DriverPose_t generateKinectPose(KVR::KinectTrackedDevice & device, vr::HmdQuaternion_t &rotation, vr::HmdVector3d_t &position) {
        vr::DriverPose_t pose = defaultReadyDriverPose();
        bool usingKinectCalibrationModel = true;
        if (usingKinectCalibrationModel) {
            applyKinectArrowCalibrationToTracker(device, rotation, position);
        }
        pose.deviceIsConnected = true;
        pose.qRotation = rotation;

        pose.qWorldFromDriverRotation = { 1,0,0,0 }; // need these two or else nothing rotates visually
        pose.qDriverFromHeadRotation = { 1,0,0,0 };

        //Final Position Adjustment
        updateDevicePosePosition(device, pose, position);

        rotation = pose.qRotation;
        position = { pose.vecPosition[0], pose.vecPosition[1], pose.vecPosition[2] };

        pose.poseIsValid = true;

        pose.result = vr::TrackingResult_Running_OK;

        return pose;
    }
    void updatePoolWithKinectSensor(KVR::KinectTrackedDevice & device) {
        // Kinect Sensor
        KVR::TrackedDeviceInputData data = defaultSensorDeviceData();

        data.position = KinectSettings::kinectRepPosition;
        data.rotation = KinectSettings::kinectRepRotation;
        
        vr::HmdQuaternion_t identityQuaterion = { 1,0,0,0 };

        data.pose = generateKinectPose(device, identityQuaterion, vr::HmdVector3d_t());

        TrackingPoolManager::updatePoolWithDevice(data, data.deviceId);
    }
    void updatePoolWithKinectJoint(KinectHandlerBase& kinect, KVR::KinectTrackedDevice & device) {
        bool usingSkeletonPosition = device.positionTrackingOption == KVR::JointPositionTrackingOption::Skeleton;
        bool usingSkeletonRotation = device.rotationTrackingOption == KVR::JointRotationTrackingOption::Skeleton;
        if (!usingSkeletonPosition
            && !usingSkeletonRotation)
            return;
        KVR::TrackedDeviceInputData data = defaultDeviceData((int)device.joint0.joint);

        data.deviceId = usingSkeletonPosition ? device.positionDevice_gId : device.rotationDevice_gId; // Doesn't need to be checked, as if it's made it past the initial check, it's going to be one or the other ID

        vr::HmdVector3d_t jointPosition{ 0,0,0 };
        vr::HmdQuaternion_t jointRotation{ 1,0,0,0 };
        if (kinect.getFilteredJoint(device, jointPosition, jointRotation)) {
            data.position = jointPosition;
            data.rotation = jointRotation;

            data.pose = generateKinectPose(device, data.rotation, data.position);

        }
        else
            data.pose.poseIsValid = false;
        // If no joint is gotten, then it will be left as 0,0,0 to be handled in the KinectTrackedDevice
        TrackingPoolManager::updatePoolWithDevice(data, data.deviceId);
    }
};