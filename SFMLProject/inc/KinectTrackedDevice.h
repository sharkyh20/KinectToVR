#pragma once
#include "stdafx.h"

#include "KinectSettings.h"
#include "KinectJoint.h"
#include "IETracker.h"
#include <vrinputemulator.h>
#include <SFML/System/Vector3.hpp>
#include <openvr_math.h>

class KinectTrackedDevice {
public:
    KinectTrackedDevice(
        vrinputemulator::VRInputEmulator& inputEmulator,
        KinectJointType j0,
        KinectJointType j1,
        bool isKinect,
        KinectVersion version)
        :
        inputEmulatorRef(inputEmulator),
        joint0(j0),
        joint1(j1),
        hmdRelativePosition(sf::Vector3f(0, 0, 0)),
        isKinectRepresentation(isKinect),
        deviceKinectVersion(version)
    {
        deviceId = initTracker(inputEmulator, true);
    }

    void update(vr::HmdVector3_t trackedPositionVROffset, vr::HmdVector3_t rawJointPos, sf::Vector3f zeroPos, vr::HmdQuaternion_t rawJointRotation) {
        lastRawPos = rawJointPos;
        auto pose = inputEmulatorRef.getVirtualDevicePose(deviceId);
        //POSITION
        double kRelativeX = rawJointPos.v[0] - zeroPos.x;
        double kRelativeY = rawJointPos.v[1] - zeroPos.y;
        double kRelativeZ = rawJointPos.v[2] - zeroPos.z;

        //TODO REPLACE KS OFFSET WITH LOCAL DEVICE OFFSET, AND PROVIDE FN TO SET ALL DEVICES AT ONCE
        double rawVRPositionX = trackedPositionVROffset.v[0] + KinectSettings::hmdZero.v[0] + kRelativeX;
        double rawVRPositionY = trackedPositionVROffset.v[1] + kRelativeY;   // The Y axis is always up, but the other two depend on kinect orientation
        double rawVRPositionZ = trackedPositionVROffset.v[2] + KinectSettings::hmdZero.v[2] + kRelativeZ;

        pose.vecPosition[0] = KinectSettings::kinectToVRScale * rawVRPositionX;
        pose.vecPosition[1] = KinectSettings::kinectToVRScale * rawVRPositionY;
        pose.vecPosition[2] = KinectSettings::kinectToVRScale * rawVRPositionZ;

        //ROTATION
        pose.qRotation.w = rawJointRotation.w;
        pose.qRotation.x = rawJointRotation.x;
        pose.qRotation.y = rawJointRotation.y;
        pose.qRotation.z = rawJointRotation.z;

        pose.qRotation = pose.qRotation;

        pose.poseIsValid = true;
        pose.result = vr::TrackingResult_Running_OK;
        inputEmulatorRef.setVirtualDevicePose(deviceId, pose);
    }
    /*
    void destroyTrackers(vrinputemulator::VRInputEmulator& inputEmulator, std::vector<KinectTrackedDevice> trackers) {
        for (auto device : trackers) {
            auto pose = inputEmulator.getVirtualDevicePose(device.deviceId);
            if (pose.deviceIsConnected) {
                pose.deviceIsConnected = false;
                pose.poseIsValid = false;
                inputEmulator.setVirtualDevicePose(device.deviceId, pose);
            }
        }
    }
    */
    ~KinectTrackedDevice() {
        
    }
    void destroy() {
        destroyTracker(inputEmulatorRef, deviceId);
    }

    vrinputemulator::VRInputEmulator &inputEmulatorRef;
    uint32_t deviceId;

    KinectJoint joint0;
    KinectJoint joint1;

    sf::Vector3f hmdRelativePosition;
    vr::HmdVector3_t lastRawPos{ 0,0,0 };
    bool isKinectRepresentation;

    KinectVersion deviceKinectVersion;
};

