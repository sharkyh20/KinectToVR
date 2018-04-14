#pragma once
#include "stdafx.h"
#include <iostream>
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

    enum class JointRotationOption {
        Filtered,
        Unfiltered,
        HeadLook
    };
    enum class JointPositionOption {
        Filtered,
        Unfiltered
    };

    class KinectTrackedDevice {
    public:
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
        void update(vr::HmdVector3_t additionalOffset, vr::HmdVector3_t rawJointPos, vr::HmdQuaternion_t rawJointRotation) {
            auto pose = inputEmulatorRef.getVirtualDevicePose(deviceId);
            vr::HmdQuaternion_t jointRotation = rawJointRotation;
            //Rotate the position around the kinect rep's rot
            if (isSensor()) {
                //Rotate arrow by 180
                jointRotation = jointRotation * vrmath::quaternionFromRotationY(PI);
            }

            vr::HmdVector3d_t dPos = { rawJointPos.v[0], rawJointPos.v[1], rawJointPos.v[2]};
            vr::HmdVector3d_t rotatedPos = vrmath::quaternionRotateVector(KinectSettings::kinectRepRotation, dPos, false);

            //Adjust this position by the Kinect's VR pos offset
            rotatedPos.v[0] += KinectSettings::kinectRepPosition.v[0];
            rotatedPos.v[1] += KinectSettings::kinectRepPosition.v[1];
            rotatedPos.v[2] += KinectSettings::kinectRepPosition.v[2];

            if (!isSensor()) {
                if (rotationOption == JointRotationOption::HeadLook) {}
                else 
                    jointRotation = KinectSettings::kinectRepRotation * jointRotation;
            }
            pose.qRotation = jointRotation;

            //Final Position Adjustment
            updateDevicePosePosition(pose, rotatedPos);

            pose.poseIsValid = true;

            pose.result = vr::TrackingResult_Running_OK;
            inputEmulatorRef.setVirtualDevicePose(deviceId, pose);
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

        KVR::KinectJoint joint0;
        KVR::KinectJoint joint1;

        std::string defaultModelName{ "vr_controller_vive_1_5" };

        bool invisible = false;

        vr::HmdVector3_t trackedPositionVROffset;
        vr::HmdVector3_t lastRawPos{ 0,0,0 };

        JointRotationOption rotationOption = JointRotationOption::Filtered;
        JointPositionOption positionOption = JointPositionOption::Filtered;

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