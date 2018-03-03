#pragma once
#include "stdafx.h"
#include <openvr.h>
#include <SFML/System/Vector3.hpp>

enum class KinectVersion {
    Version1 = 0,   //AKA Xbox 360/ Windows v1
    Version2,    //AKA Xbox One/ Windows v2
    INVALID 
};
namespace KinectSettings {
    extern bool isKinectDrawn;
    extern bool isSkeletonDrawn;
    extern bool ignoreInferredPositions;

    extern bool userChangingZero;

    extern float g_TrackedBoneThickness;
    extern float g_InferredBoneThickness;
    extern float g_JointThickness;

    extern const int kinectHeight;
    extern const int kinectWidth;

    extern const int kinectV2Height;
    extern const int kinectV2Width;

    extern double kinectToVRScale;

    extern vr::HmdVector3_t hmdZero; //TEMP GLOBAL
    extern vr::HmdQuaternion_t kinectRepRotation;
    extern vr::HmdVector3_t kinectRadRotation;
    extern vr::HmdVector3_t kinectRepPosition;
    extern bool adjustingKinectRepresentationRot;
    extern bool adjustingKinectRepresentationPos;


    void serializeKinectSettings();
    void writeKinectSettings();
}
namespace SFMLsettings {
    extern int m_window_width;
    extern int m_window_height;
}
# define M_PI           3.14159265358979323846
