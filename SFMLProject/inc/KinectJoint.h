#pragma once
#include "stdafx.h"
#include <string>

namespace KVR {
    const int KinectJointCount = 25;
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

        INVALID = 25, // Indicates device doesn't use Kinect joint
    };
    extern std::string KinectJointName[KinectJointCount];

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

        KinectJointType joint;
    };
}