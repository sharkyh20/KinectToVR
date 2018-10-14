#pragma once
#include "stdafx.h"
#include <iostream>
#include <vector>
#include <string>

#include "KinectSettings.h"
#include "KinectJoint.h"
#include "IETracker.h"
#include <vrinputemulator.h>
#include <SFML/System/Vector3.hpp>
#include <openvr_math.h>
#include "VectorMath.h"

namespace KVR {

    enum class KinectDeviceRole {
        Unassigned = 0,
        LeftFoot,
        RightFoot,
        Hip,
        KinectSensor,
        LeftHand,
        RightHand,
        Count
    };
    const static std::string KinectDeviceRoleName[] = {
        "Unassigned",
        "Left Foot",
        "Right Foot",
        "Hip",
        "Kinect Sensor",
        "Left Hand",
        "Right Hand",
        "Count"
    };

    enum class JointRotationFilterOption {
        Filtered,
        Unfiltered,
        HeadLook
    };
    enum class JointPositionFilterOption {
        Filtered,
        Unfiltered
    };
    enum class JointPositionTrackingOption {
        Skeleton,
        IMU,
        Color
    };
    enum class JointRotationTrackingOption {
        Skeleton,
        IMU,
        Headlook
    };

    

    class KinectTrackedDevice {
    public:
        KinectTrackedDevice(
            vrinputemulator::VRInputEmulator& inputEmulator,
            uint32_t posDevice_gId,
            uint32_t rotDevice_gId,
            KinectDeviceRole r)
            :
            inputEmulatorRef(inputEmulator),
            positionDevice_gId(posDevice_gId),
            rotationDevice_gId(rotDevice_gId),
            trackedPositionVROffset({ 0,0,0 }),
            role(r)
        {

        }
        KinectTrackedDevice(
            vrinputemulator::VRInputEmulator& inputEmulator,
            KVR::KinectJointType j0,
            KVR::KinectJointType j1,
            KinectDeviceRole r)
            :
            inputEmulatorRef(inputEmulator),
            joint0(j0),
            joint1(j1),
            trackedPositionVROffset({ 0,0,0 }),
            role(r)
        {

        }
        void init(vrinputemulator::VRInputEmulator& inputEmulator) {
            deviceId = initTracker(inputEmulator, true);
        }

        void setRotationForNextUpdate(vr::HmdQuaternion_t rotation) {
            nextUpdateRotation = rotation;
        }
        void setPositionForNextUpdate(vr::HmdVector3d_t position) {
            nextUpdatePosition = position;
        }
        void update() {
            // Send through the positions for the next update of the controller
            // - called for each controller at the end of the TrackingMethod iteration
            vr::DriverPose_t pose{};

            usingKinectCalibrationModel = false;
            if (usingKinectCalibrationModel) {
                applyKinectArrowCalibrationToTracker(nextUpdateRotation, nextUpdatePosition);
            }
            pose.deviceIsConnected = true;
            pose.qRotation = nextUpdateRotation;

            pose.qWorldFromDriverRotation = { 1,0,0,0 }; // need these else nothing rotates/moves visually
            pose.vecWorldFromDriverTranslation[0] = 0;
            pose.vecWorldFromDriverTranslation[1] = 0;
            pose.vecWorldFromDriverTranslation[2] = 0;

            pose.qDriverFromHeadRotation = { 1,0,0,0 };
            pose.vecDriverFromHeadTranslation[0] = 0;
            pose.vecDriverFromHeadTranslation[1] = 0;
            pose.vecDriverFromHeadTranslation[2] = 0;
            
            //Final Position Adjustment
            updateDevicePosePosition(pose, nextUpdatePosition);

            pose.poseIsValid = true;

            pose.result = vr::TrackingResult_Running_OK;
            inputEmulatorRef.setVirtualDevicePose(deviceId, pose);
        }

        void update(vr::DriverPose_t pose) {
            // Pose already completely handled by Tracking Method
            inputEmulatorRef.setVirtualDevicePose(deviceId, pose);
        }
        bool sensorShouldSkipUpdate() {
            // The sensor doesn't actually need to be updated more often than not,
            // and by only updating it after the config changes, there's a pretty large
            // perf/responsiveness benefit to the trackers
            if (isSensor()) {
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
        void update(vr::HmdVector3d_t additionalOffset, vr::HmdVector3d_t rawJointPos, vr::HmdQuaternion_t rawJointRotation) {
            // Old, and soon to be deprecated method of updating tracker
            // Done all at once, with little modularity
            if (sensorShouldSkipUpdate()) {
                return;
            }

            //auto pose = inputEmulatorRef.getVirtualDevicePose(deviceId);
            vr::DriverPose_t pose{};
            vr::HmdQuaternion_t jointRotation = rawJointRotation;
            vr::HmdVector3d_t jointPosition = rawJointPos;


            // JUST FOR PSMOVE TESTING
            usingKinectCalibrationModel = true;
            // ---------------------

            if (usingKinectCalibrationModel) {
                applyKinectArrowCalibrationToTracker(jointRotation, jointPosition);
            }
            pose.deviceIsConnected = true;
            pose.qRotation = jointRotation;

            pose.qWorldFromDriverRotation = { 1,0,0,0 }; // need these two or else nothing rotates visually
            pose.qDriverFromHeadRotation = { 1,0,0,0 };

            //Final Position Adjustment
            updateDevicePosePosition(pose, jointPosition);

            pose.poseIsValid = true;

            pose.result = vr::TrackingResult_Running_OK;
            inputEmulatorRef.setVirtualDevicePose(deviceId, pose);
        }
        void applyKinectArrowCalibrationToTracker(vr::HmdQuaternion_t &rotation, vr::HmdVector3d_t &position) {
            // If using the kinect, then the position/rot of the tracker has to be adjusted
            // according to the arrow, to transform it into the proper OpenVR coords

            //Rotate the position around the kinect rep's rot
            if (isSensor()) {
                //Rotate arrow by 180
                rotation = rotation * vrmath::quaternionFromRotationY(PI);
            }

            vr::HmdVector3d_t rotatedPos = vrmath::quaternionRotateVector(KinectSettings::kinectRepRotation, position, false);

            //Adjust this position by the Kinect's VR pos offset
            rotatedPos.v[0] += KinectSettings::kinectRepPosition.v[0];
            rotatedPos.v[1] += KinectSettings::kinectRepPosition.v[1];
            rotatedPos.v[2] += KinectSettings::kinectRepPosition.v[2];
            
            if (rotationFilterOption == JointRotationFilterOption::HeadLook) {}
            else
                rotation = KinectSettings::kinectRepRotation * rotation;
            position = rotatedPos;
        }

        bool isSensor() {
            return role == KinectDeviceRole::KinectSensor;
        }

        ~KinectTrackedDevice() {

        }
        void destroy() {
            destroyTracker(inputEmulatorRef, deviceId);
        }

        vrinputemulator::VRInputEmulator &inputEmulatorRef;
        uint32_t deviceId;

        KVR::KinectJoint joint0 = KVR::KinectJointType::Head;
        KVR::KinectJoint joint1 = KVR::KinectJointType::Head;

        std::string defaultModelName{ "vr_controller_vive_1_5" };

        bool invisible = false;

        bool usingKinectCalibrationModel = true;

        vr::HmdVector3_t trackedPositionVROffset;
        vr::HmdVector3_t lastRawPos{ 0,0,0 };
        vr::DriverPose_t nextUpdatePose;
        vr::HmdQuaternion_t nextUpdateRotation{ 1,0,0,0 };
        vr::HmdVector3d_t nextUpdatePosition{0,0,0};

        JointRotationFilterOption rotationFilterOption = JointRotationFilterOption::Filtered;
        JointPositionFilterOption positionFilterOption = JointPositionFilterOption::Filtered;
        JointPositionTrackingOption positionTrackingOption = JointPositionTrackingOption::Skeleton;
        JointRotationTrackingOption rotationTrackingOption = JointRotationTrackingOption::Skeleton;

        uint32_t positionDevice_gId = 404;
        uint32_t rotationDevice_gId = 404;


        KinectDeviceRole role;
    private:
        void updateDevicePosePosition(vr::DriverPose_t &pose, vr::HmdVector3d_t rotatedPos) {
            pose.vecPosition[0] = rotatedPos.v[0] + trackedPositionVROffset.v[0];
            pose.vecPosition[1] = rotatedPos.v[1] + trackedPositionVROffset.v[1];
            pose.vecPosition[2] = rotatedPos.v[2] + trackedPositionVROffset.v[2];
            if (role == KinectDeviceRole::Hip) {
                pose.vecPosition[1] += KinectSettings::hipRoleHeightAdjust;
            }
        }
    };

}
