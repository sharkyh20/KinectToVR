#include "stdafx.h"
#include "KinectSettings.h"

namespace KinectSettings {
    bool isKinectDrawn = false;
    bool isSkeletonDrawn = false;
    bool ignoreInferredPositions = true;

    bool userChangingZero = false;

    float g_TrackedBoneThickness = 6.0f;
    float g_InferredBoneThickness = 1.5f;
    float g_JointThickness = 4.0f;

    const int kinectHeight = 640;
    const int kinectWidth = 480;

    const int kinectV2Height = 1920;
    const int kinectV2Width = 1080;

    double kinectToVRScale = 1;

    vr::HmdVector3_t hmdZero; //TEMP GLOBAL
    vr::HmdQuaternion_t kinectRepRotation{0,0,0,0};  //TEMP
    extern vr::HmdVector3_t kinectRadRotation{0,0,0};
    vr::HmdVector3_t kinectRepPosition{0,0,0};
    bool adjustingKinectRepresentationRot = false;
    bool adjustingKinectRepresentationPos = true;
}
namespace SFMLsettings {
    int m_window_width = 800;
    int m_window_height = 600;
}
# define M_PI           3.14159265358979323846
