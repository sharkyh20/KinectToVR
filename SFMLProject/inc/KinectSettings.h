#pragma once
#include "stdafx.h"
#include <openvr.h>
#include <SFML/System/Vector3.hpp>
#include <SFML\Graphics\Text.hpp>
#include <string>
#include <sstream>
#include <Eigen/Geometry>

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
    extern bool headtracked;
    extern float g_TrackedBoneThickness;
    extern float g_InferredBoneThickness;
    extern float g_JointThickness;

    extern const int kinectHeight;
    extern const int kinectWidth;

    extern const int kinectV2Height;
    extern const int kinectV2Width;

    extern double kinectToVRScale;

    extern double hipRoleHeightAdjust;
    extern float tryaw;

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
    extern vr::HmdVector3d_t secondaryTrackingOriginOffset; // Demonic offset, actual origin unknown. Probably evil and trying to destroy everything I love.


    extern vr::HmdQuaternion_t kinectRepRotation;
    extern vr::HmdVector3d_t kinectRadRotation;
    extern vr::HmdVector3d_t kinectRepPosition;
    extern vr::HmdVector3d_t moffsets[2][3];
    extern vr::HmdVector3d_t hoffsets;
    extern vr::HmdVector3d_t huoffsets;
    extern vr::HmdVector3d_t troffsets;
	extern float hroffset;
	extern float troffset;
	extern vr::HmdQuaternion_t hmdquat;

    extern Eigen::Matrix<float, 3, 3> R_matT;
    extern Eigen::Matrix<float, 3, 1> T_matT;
    extern bool ismatrixcalibrated;

    extern int cpoints;
    extern bool rtcalibrated;

    extern float hmdegree;
    extern bool sensorConfigChanged;

    extern vr::HmdVector3d_t mposes[3];

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
namespace vr {
    // For Cereal to actually use the serialization methods, they have to
    // be in the same namespace as the declared object...
    template<class Archive>
    void serialize(Archive & archive,
        vr::HmdQuaternion_t & q)
    {
        archive(q.w, q.x, q.y, q.z);
    }

    template<class Archive>
    void serialize(Archive & archive,
        vr::HmdVector3d_t & v)
    {
        archive(v.v[0], v.v[1], v.v[2]);
    }
    bool operator== (const vr::HmdVector3d_t &lhs, const vr::HmdVector3d_t &rhs);
    bool operator== (const vr::HmdQuaternion_t &lhs, const vr::HmdQuaternion_t &rhs);
    
}
namespace KVR {
    extern std::wstring trackerConfig;

    std::wstring fileToDirPath(const std::wstring & relativeFilePath);
    extern std::wstring ToUTF16(const std::string &data);

    extern std::string ToUTF8(const std::wstring &data);
    extern std::string inputDirForOpenVR(const std::string & file);

    // Each tracking system has it's global adjustments here, in the form
    // of their driver-from-world offsets, so that they can be reapplied at startup
    struct TrackingSystemCalibration {
        std::string systemName = "INVALID";
        vr::HmdQuaternion_t driverFromWorldRotation = { 1,0,0,0 };
        vr::HmdVector3d_t driverFromWorldPosition = { 0,0,0 };
    };
    
    TrackingSystemCalibration retrieveSystemCalibration(const std::string & systemName);
    void saveSystemCalibration(const std::string & systemName, TrackingSystemCalibration calibration);
}

namespace VRInput {
    // Switch to fall back to legacy controls when the new SteamVR Input system breaks.
    extern bool legacyInputModeEnabled;

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
