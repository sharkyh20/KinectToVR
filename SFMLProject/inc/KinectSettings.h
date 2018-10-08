#pragma once
#include "stdafx.h"
#include <openvr.h>
#include <SFML/System/Vector3.hpp>
#include <SFML\Graphics\Text.hpp>
#include <string>
#include <sstream>

enum class KinectVersion {
    Version1 = 0,   //AKA Xbox 360/ Windows v1
    Version2,    //AKA Xbox One/ Windows v2
    INVALID 
};
namespace KinectSettings {
    extern bool isKinectDrawn;
    extern bool isSkeletonDrawn;
    extern bool ignoreInferredPositions;
    extern bool ignoreRotationSmoothing;

    extern bool userChangingZero;

    extern float g_TrackedBoneThickness;
    extern float g_InferredBoneThickness;
    extern float g_JointThickness;

    extern const int kinectHeight;
    extern const int kinectWidth;

    extern const int kinectV2Height;
    extern const int kinectV2Width;

    extern double kinectToVRScale;

    extern double hipRoleHeightAdjust;


    //Need to delete later (Merge should sort it)
    extern int leftHandPlayspaceMovementButton;
    extern int rightHandPlayspaceMovementButton;
    extern int leftFootPlayspaceMovementButton;
    extern int rightFootPlayspaceMovementButton;

    extern vr::HmdVector3_t hmdPosition; 
    extern vr::HmdQuaternion_t hmdRotation;


    extern vr::HmdQuaternion_t kinectRepRotation;
    extern vr::HmdVector3_t kinectRadRotation;
    extern vr::HmdVector3_t kinectRepPosition;
    extern bool adjustingKinectRepresentationRot;
    extern bool adjustingKinectRepresentationPos;

    extern std::string KVRversion;

    void serializeKinectSettings();
    void writeKinectSettings();
}
namespace SFMLsettings {
    extern int m_window_width;
    extern int m_window_height;
    extern float windowScale;

    extern bool usingGamepad;

    extern std::string fileDirectoryPath;

    extern std::stringstream debugDisplayTextStream;
}
namespace KVR {
    std::string fileToDirPath(std::string relativeFilePath);
}
# define M_PI           3.14159265358979323846
