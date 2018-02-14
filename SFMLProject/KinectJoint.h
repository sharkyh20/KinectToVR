#pragma once
#include "stdafx.h"

#include <iostream>
// Kinect Includes
#include <Windows.h> //MUST BE BEFORE NUI
#include <ole2.h>

#include <NuiApi.h>
#include <NuiImageCamera.h>
#include <NuiSensor.h>
#include <NuiSkeleton.h>

// Kinect V2
#include <Kinect.h>

enum class KinectJointType {
    SpineBase = 0,
    SpineMid = 1,
    Neck = 2,   //Not in v1
    Head = 3,
    ShoulderLeft = 4,
    ElbowLeft = 5,
    WristLeft = 6,
    HandLeft = 7,
    ShoulderRight = 8,
    ElbowRight = 9,
    WristRight = 10,
    HandRight = 11,
    HipLeft = 12,
    KneeLeft = 13,
    AnkleLeft = 14,
    FootLeft = 15,
    HipRight = 16,
    KneeRight = 17,
    AnkleRight = 18,
    FootRight = 19,
    SpineShoulder = 20,
    HandTipLeft = 21, //Not in v1
    ThumbLeft = 22, //Not in v1
    HandTipRight = 23, //Not in v1
    ThumbRight = 24, //Not in v1
};
class KinectJoint {
public:
    // Allows for conversion between v1 and v2 joints without multiple variables
    // Enums for v1 are 'NUI_SKELETON_POSITION_INDEX' in NuiSensor.h
    // Enums for v2 are 'JointType' in Kinect.h

    KinectJoint(KinectJointType type) :
        joint(type)
    {
    }
    ~KinectJoint() {}
    void setJoint(KinectJointType type) {
        joint = type;
    }

    NUI_SKELETON_POSITION_INDEX getV1Representation() {
        //Unfortunately I believe this is required because there are mismatches between v1 and v2 joint IDs
        //Might consider investigating to see if there's a way to shorten this
        switch (joint) {
        case KinectJointType::SpineBase:
            return NUI_SKELETON_POSITION_HIP_CENTER;
        case KinectJointType::SpineMid:
            return NUI_SKELETON_POSITION_SPINE;

        case KinectJointType::Head:
            return NUI_SKELETON_POSITION_HEAD;
        case KinectJointType::ShoulderLeft:
            return NUI_SKELETON_POSITION_SHOULDER_LEFT;
        case KinectJointType::ShoulderRight:
            return NUI_SKELETON_POSITION_SHOULDER_RIGHT;
        case KinectJointType::SpineShoulder:
            return NUI_SKELETON_POSITION_SHOULDER_CENTER;

        case KinectJointType::ElbowLeft:
            return NUI_SKELETON_POSITION_ELBOW_LEFT;
        case KinectJointType::WristLeft:
            return NUI_SKELETON_POSITION_WRIST_LEFT;
        case KinectJointType::HandLeft:
            return NUI_SKELETON_POSITION_HAND_LEFT;

        case KinectJointType::ElbowRight:
            return NUI_SKELETON_POSITION_ELBOW_RIGHT;
        case KinectJointType::WristRight:
            return NUI_SKELETON_POSITION_WRIST_RIGHT;
        case KinectJointType::HandRight:
            return NUI_SKELETON_POSITION_HAND_RIGHT;

        case KinectJointType::HipLeft:
            return NUI_SKELETON_POSITION_HIP_LEFT;
        case KinectJointType::HipRight:
            return NUI_SKELETON_POSITION_HIP_RIGHT;

        case KinectJointType::KneeLeft:
            return NUI_SKELETON_POSITION_KNEE_LEFT;
        case KinectJointType::KneeRight:
            return NUI_SKELETON_POSITION_KNEE_RIGHT;

        case KinectJointType::AnkleLeft:
            return NUI_SKELETON_POSITION_ANKLE_LEFT;
        case KinectJointType::AnkleRight:
            return NUI_SKELETON_POSITION_ANKLE_RIGHT;

        case KinectJointType::FootLeft:
            return NUI_SKELETON_POSITION_FOOT_LEFT;
        case KinectJointType::FootRight:
            return NUI_SKELETON_POSITION_FOOT_RIGHT;

            /*BELOW DO NOT HAVE A 1:1 V1 REPRESENTATION*/
            //refer to the skeleton images from Microsoft for diffs between v1 and 2

        case KinectJointType::Neck:
            return NUI_SKELETON_POSITION_SHOULDER_CENTER;
        case KinectJointType::HandTipLeft:
            return NUI_SKELETON_POSITION_HAND_LEFT;
        case KinectJointType::HandTipRight:
            return NUI_SKELETON_POSITION_HAND_RIGHT;
        case KinectJointType::ThumbLeft:
            return NUI_SKELETON_POSITION_HAND_LEFT;
        case KinectJointType::ThumbRight:
            return NUI_SKELETON_POSITION_HAND_RIGHT;

        default:
            std::cerr << "INVALID KinectJointType!!!\n";
            break;

        }
    }
    JointType getV2Representation() {
        // Currently, the v2 SDK id's match my jointtype class 1:1
        return static_cast<JointType>(joint);
    }
private:
    KinectJointType joint;
};