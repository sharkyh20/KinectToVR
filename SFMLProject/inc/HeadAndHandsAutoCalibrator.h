#pragma once

#include "Calibrator.h"
#include "KinectSettings.h"
#include "VRController.h"
#include "KinectHandlerBase.h"
#include <math.h>

#include "KinectTrackedDevice.h"

// This tracking calibrator is designed for use with the kinect.
// It solves the location of the kinect relative to the OpenVR coords
// and places the kinect representation there for visual.
// It assumes that the user has two tracked controllers and a headset.

class HeadAndHandsAutoCalibrator : Calibrator{
public:
    static void Calibrate(
        double deltaT,
        KinectHandlerBase &kinect,
        VRcontroller leftController,
        VRcontroller rightController,
        GUIHandler &guiRef) {
        if (KinectSettings::adjustingKinectRepresentationPos) {
            performCalibration(kinect, leftController, rightController);
        }
    }
private:
    static void performCalibration(
        KinectHandlerBase &kinect,
        VRcontroller leftController,
        VRcontroller rightController) {

        // Math and paper courtesy of To3x, cheers mate.

        // vec_Xvr = A * vec_Xk + vec_b
        // The calibration needs to find the A and b.
        // If A and b is known, the output of the Kinect can easily  
        // be converted to coordinates of the Rift system.

        /*
        // Get Input Data from both systems
        vr::HmdVector3d_t headVRPos = KinectSettings::hmdPosition;
        vr::HmdVector3d_t leftConVRPos = GetVRPositionFromMatrix(leftController.GetPose().mDeviceToAbsoluteTracking);
        vr::HmdVector3d_t rightConVRPos = GetVRPositionFromMatrix(rightController.GetPose().mDeviceToAbsoluteTracking);

        KinectTrackedDevice leftHandDevice(KVR::KinectJointType::HandLeft, KVR::KinectJointType::HandLeft, KVR::KinectDeviceRole::LeftHand);
        KinectTrackedDevice rightHandDevice(KVR::KinectJointType::HandRight, KVR::KinectJointType::HandRight, KVR::KinectDeviceRole::RightHand);
        KinectTrackedDevice headDevice(KVR::KinectJointType::Head, KVR::KinectJointType::Head, KVR::KinectDeviceRole::Unassigned);
        vr::HmdVector3d_t headKPos, leftConKPos, rightConKPos;
        // Need to refactor this function, to allow for direct request
        // instead of creating a fake tracker
        kinect.getFilteredJoint(headDevice, headKPos, {});
        kinect.getFilteredJoint(leftHandDevice, leftConKPos, {});
        kinect.getFilteredJoint(rightHandDevice, rightConKPos, {});

        // Plausibility Check
        // Make sure that nothing is zeroed out

        // Calculate b - the origin of the Kinect coord system in VR coords.
        // The origin of the kinect coord system is the sensor camera itself.
        // Microsoft provides details for this: https://docs.microsoft.com/en-us/previous-versions/windows/kinect/dn785530%28v%3dieb.10%29

        // P is head, Q is Left Controller, R is Right controller
        // VR is VR space, K means Kinect Space

        double dist_P_k_sq = squaredDistanceFromVecOrigin(headKPos);
        double dist_Q_k_sq = squaredDistanceFromVecOrigin(leftConKPos);
        double dist_R_k_sq = squaredDistanceFromVecOrigin(rightConKPos);
        */
        //https://stackoverflow.com/questions/1406375/finding-intersection-points-between-3-spheres
        // Need to adapt the answer from Andrew Wagner for next step.
    }

    static double squaredDistanceFromVecOrigin(vr::HmdVector3d_t p) {
        // REMEMBER, this is distance SQUARED
        return p.v[0] * p.v[0]
            + p.v[1] * p.v[1]
            + p.v[2] * p.v[2];
    }
};
