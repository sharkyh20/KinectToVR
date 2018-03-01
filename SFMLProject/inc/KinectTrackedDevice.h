#pragma once
#include "stdafx.h"
#include <iostream>
#include "KinectSettings.h"
#include "KinectJoint.h"
#include "IETracker.h"
#include <vrinputemulator.h>
#include <SFML/System/Vector3.hpp>
#include <openvr_math.h>
#include "VectorMath.h"

class KinectTrackedDevice {
public:
    KinectTrackedDevice(
        vrinputemulator::VRInputEmulator& inputEmulator,
        KinectJointType j0,
        KinectJointType j1,
        bool isKinect)
        :
        inputEmulatorRef(inputEmulator),
        joint0(j0),
        joint1(j1),
        trackedPositionVROffset({ 0,0,0 }),
        isKinectRepresentation(isKinect)
    {
        deviceId = initTracker(inputEmulator, true);
    }

    void update(vr::HmdVector3_t additionalOffset, vr::HmdVector3_t rawJointPos, vr::HmdQuaternion_t rawJointRotation) {
        lastRawPos = rawJointPos;
        auto pose = inputEmulatorRef.getVirtualDevicePose(deviceId);
        //JOINT POSITION
        //Perspective Correction for changing coord systems
        const float PI = 3.14159265359f;
        //Convert to sf vector for rotation fn
        sf::Vector3f jPosition = { rawJointPos.v[0],rawJointPos.v[1] ,rawJointPos.v[2] };

        //Rotate the position around the kinect rep's rot
        sf::Vector3f laterallyRotatedPos = rotate(jPosition, { 0,1,0 }, -KinectSettings::kinectRadRotation.v[0]);
        sf::Vector3f tiltRotatedPos = rotate(laterallyRotatedPos, { 1,0,0 }, -KinectSettings::kinectRadRotation.v[1]);
        vr::HmdVector3_t pos = { { tiltRotatedPos.x,tiltRotatedPos.y,tiltRotatedPos.z } };

        //Adjust this position by the Kinect's VR pos offset
        pos.v[0] += KinectSettings::kinectRepPosition.v[0];
        pos.v[1] += KinectSettings::kinectRepPosition.v[1];
        pos.v[2] += KinectSettings::kinectRepPosition.v[2];

        //std::cerr << "jPOS:" << pos.v[0] << ", " << pos.v[1] << ", " << pos.v[2] << "\n";
        //JOINT ROTATION
        rawJointRotation = rawJointRotation * KinectSettings::kinectRepRotation;

        pose.qRotation.w = rawJointRotation.w;
        pose.qRotation.x = rawJointRotation.x;
        pose.qRotation.y = rawJointRotation.y;
        pose.qRotation.z = rawJointRotation.z;
        pose.vecPosition[0] = pos.v[0];
        pose.vecPosition[1] = pos.v[1];
        pose.vecPosition[2] = pos.v[2];
        //Debug
        /*
        if (isKinectRepresentation) {
            
            std::cerr << "KMOV: " << pose.vecPosition[0] << ", " << pose.vecPosition[1] << ", " << pose.vecPosition[2] << '\n';
            std::cerr << "KROT: " << KinectSettings::kinectRadRotation.v[0] << ", " << KinectSettings::kinectRadRotation.v[1] << ", " << KinectSettings::kinectRadRotation.v[2]  << '\n';
        }
        */
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

    vr::HmdVector3_t trackedPositionVROffset;
    vr::HmdVector3_t lastRawPos{ 0,0,0 };
    bool isKinectRepresentation;
};

