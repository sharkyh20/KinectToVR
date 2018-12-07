#pragma once
#include "stdafx.h"
#include <openvr.h>
#include <SFML/System/Vector3.hpp>
#include <SFML\Graphics\Text.hpp>
#include <string>
#include <sstream>

#include "KinectJoint.h"

enum class KinectVersion {
    Version1 = 1,   //AKA Xbox 360/ Windows v1
    Version2 = 2,    //AKA Xbox One/ Windows v2
    INVALID = 404
};
namespace KinectSettings {
    extern bool isKinectDrawn;
    extern bool isSkeletonDrawn;
    extern bool ignoreInferredPositions;
    extern bool ignoreRotationSmoothing;

    extern KVR::KinectJointType leftFootJointWithRotation;
    extern KVR::KinectJointType rightFootJointWithRotation;
    extern KVR::KinectJointType leftFootJointWithoutRotation;
    extern KVR::KinectJointType rightFootJointWithoutRotation;

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

    extern vr::HmdVector3d_t hmdPosition; 
    extern vr::HmdQuaternion_t hmdRotation;
    extern vr::HmdMatrix34_t hmdAbsoluteTracking;
    extern vr::HmdMatrix34_t trackingOrigin;
    extern vr::HmdVector3d_t trackingOriginPosition; // Input Emulator is by default offset from this - so 0,0,0 in IE is really these coords


    extern vr::HmdQuaternion_t kinectRepRotation;
    extern vr::HmdVector3d_t kinectRadRotation;
    extern vr::HmdVector3d_t kinectRepPosition;
    extern bool sensorConfigChanged;

    extern bool adjustingKinectRepresentationRot;
    extern bool adjustingKinectRepresentationPos;
    void updateKinectQuaternion();

    extern std::string KVRversion;

    void serializeKinectSettings();
    void writeKinectSettings();
}
namespace SFMLsettings {
    extern int m_window_width;
    extern int m_window_height;
    extern float windowScale;

    extern float globalFontSize;

    extern bool keepRunning;

    extern bool usingGamepad;

    extern std::wstring fileDirectoryPath;

    extern std::stringstream debugDisplayTextStream;
}
namespace KVR {
    std::wstring fileToDirPath(std::wstring relativeFilePath);
    extern std::wstring ToUTF16(const std::string &data);

    extern std::string ToUTF8(const std::wstring &data);
    extern const char* inputDirForOpenVR(std::string file);
}

namespace VRInput {
    // Action Handles
    extern vr::VRActionHandle_t moveHorizontallyHandle;
    extern vr::VRActionHandle_t moveVerticallyHandle;
    extern vr::VRActionHandle_t confirmCalibrationHandle;

    // Calibration Sets
    extern vr::VRActionSetHandle_t calibrationSetHandle;

    // Action Sets
    extern vr::VRActiveActionSet_t activeActionSet;

    // Digital Action Data
    extern vr::InputDigitalActionData_t confirmCalibrationData;

    // Analog Action Data
    extern vr::InputAnalogActionData_t moveHorizontallyData;
    extern vr::InputAnalogActionData_t moveVerticallyData;

    bool initialiseVRInput();
    void updateVRInput();
}
# define M_PI           3.14159265358979323846
