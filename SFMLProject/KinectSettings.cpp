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

    sf::Vector2f m_points[NUI_SKELETON_POSITION_COUNT];

    vr::HmdVector3_t hmdZero; //TEMP GLOBAL
    vr::HmdVector3_t m_HMDposition;
    vr::HmdQuaternion_t m_HMDquaternion;


}