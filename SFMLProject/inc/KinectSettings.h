#pragma once
#include "stdafx.h"
#include <openvr.h>
#include <SFML/Graphics/Text.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/detail/type_vec3.hpp>
#include <glm/detail/type_vec4.hpp>
#include <glm/detail/type_vec2.hpp>
#include <string>
#include <sstream>
#include <Eigen/Geometry>
#include "KinectJoint.h"
#include <PSMoveClient_CAPI.h>

enum KinectVersion
{
	Version1 = 1,
	//AKA Xbox 360/ Windows v1
	Version2 = 2,
	//AKA Xbox One/ Windows v2
	INVALID = 404
};

enum footRotationFilterOption
{
	k_EnableOrientationFilter,
	///enable all rotation filter dimensions
	k_DisableOrientationFilter,
	///disable joints rotation
	k_EnableOrientationFilter_WithoutYaw,
	///don't rotate foots in +y
	k_EnableOrientationFilter_HeadOrientation,
	///use headset orientation for foots
	k_EnableOrientationFilter_Software,
	///use headset orientation for foots
};

enum hipsRotationFilterOption
{
	k_EnableHipsOrientationFilter,
	///enable all rotation filter dimensions
	k_DisableHipsOrientationFilter,
	///disable joint rotation
	k_EnableHipsOrientationFilter_HeadOrientation,
	///use headset orientation for hip tracker
};

static struct footRotFilter
{
	footRotationFilterOption filterOption;
} footOrientationFilterOption;

static struct hipsRotFilter
{
	hipsRotationFilterOption filterOption;
} hipsOrientationFilterOption;

enum positionalFilterOption
{
	k_EnablePositionFilter_Kalman,
	///use EKF filtering for position
	k_EnablePositionFilter_LowPass,
	///use LowPass filtering for position
	k_EnablePositionFilter_LERP,
	///use Interpolation filtering for position
	k_DisablePositionFilter,
	///disable filtering for position
};

static struct posFilter
{
	positionalFilterOption filterOption;
} positionFilterOption;

enum controllersTrackingOption
{
	k_PSMoveFull,
	k_PSMoveRot_KinectPose
};

static struct trackingOpt
{
	controllersTrackingOption trackingOption;
} controllersTrackingOption_s;

enum bodyTrackingOption
{
	k_PSMoveFullTracking,
	k_KinectFullTracking
};

static struct bodyTrackingOpt
{
	bodyTrackingOption trackingOption;
} bodyTrackingOption_s;

enum headTrackingOption
{
	k_PSMoveTracking,
	k_KinectTracking
};

static struct headTrackingOpt
{
	headTrackingOption trackingOption;
} headTrackingOption_s;

namespace KinectSettings
{
	static struct K2VR_PSMoveData
	{
		PSMPSMove PSMoveData;
		bool isValidController = false;
	} KVRPSMoveData[11];

	static std::vector<K2VR_PSMoveData> KVR_PSMoves;
	extern bool isCalibrating, isKinectPSMS;
	extern int K2Drivercode, kinectVersion;
	extern PSMPSMove right_move_controller, left_move_controller, left_foot_psmove, right_foot_psmove, waist_psmove, atamamove;
	extern glm::quat left_tracker_rot, right_tracker_rot, waist_tracker_rot;
	extern glm::quat trackerSoftRot[2]; //Software-calculated
	extern bool isGripPressed[2], isTriggerPressed[2]; //0L, 1R
	extern bool isDriverPresent;
	extern bool isKinectDrawn;
	extern bool isSkeletonDrawn;
	extern bool ignoreInferredPositions;
	extern bool ignoreRotationSmoothing;
	extern std::string opt;
	extern KVR::KinectJointType leftFootJointWithRotation;
	extern KVR::KinectJointType rightFootJointWithRotation;
	extern KVR::KinectJointType leftFootJointWithoutRotation;
	extern KVR::KinectJointType rightFootJointWithoutRotation;
	extern float svrhmdyaw;
	extern bool userChangingZero;
	extern bool headtracked;
	extern float g_TrackedBoneThickness;
	extern float g_InferredBoneThickness;
	extern float g_JointThickness;
	extern float hmdYaw, calibration_kinect_pitch;
	extern int flashnow[2];
	extern bool conActivated;
	extern std::vector<int> psmindexidpsm[2];
	extern float ardroffset;
	extern const int kinectHeight;
	extern const int kinectWidth;
	extern bool legacy;
	extern const int kinectV2Height;
	extern const int kinectV2Width;
	extern bool rtconcalib;
	extern double kinectToVRScale;
	extern bool initialised;
	extern bool psmbuttons[5][10];
	extern float conID[2];
	extern double hipRoleHeightAdjust;
	extern float calibration_trackers_yaw;
	extern int feet_rotation_option, hips_rotation_option, posOption, conOption;
	extern int positional_tracking_option, headtrackingoption;
	//Need to delete later (Merge should sort it)
	extern int leftHandPlayspaceMovementButton;
	extern int rightHandPlayspaceMovementButton;
	extern int leftFootPlayspaceMovementButton;
	extern int rightFootPlayspaceMovementButton;
	extern vr::TrackedDevicePose_t controllersPose[2];
	extern bool frame1;
	extern vr::HmdVector3d_t hmdPosition;
	extern vr::HmdQuaternion_t hmdRotation, hmdRot;
	extern vr::HmdMatrix34_t hmdAbsoluteTracking;
	extern vr::HmdMatrix34_t trackingOrigin;
	extern vr::HmdVector3d_t trackingOriginPosition;
	// Input Emulator is by default offset from this - so 0,0,0 in IE is really these coords
	extern vr::HmdVector3d_t secondaryTrackingOriginOffset;
	// Demonic offset, actual origin unknown. Probably evil and trying to destroy everything I love.
	extern vr::HmdVector3d_t hauoffset, mauoffset;
	extern int psmh, psmm;
	extern vr::HmdQuaternion_t kinectRepRotation;
	extern vr::HmdVector3d_t kinectRadRotation;
	extern vr::HmdVector3d_t kinectRepPosition;
	extern vr::HmdVector3d_t manual_offsets[2][3];
	extern vr::HmdVector3d_t hoffsets;
	extern vr::HmdVector3d_t huoffsets;
	extern vr::HmdVector3d_t kinect_tracker_offsets;
	extern float hroffset;
	extern float troffset;
	extern vr::HmdQuaternion_t hmdquat;
	extern bool expcalib;
	extern bool jcalib;
	extern Eigen::Matrix<float, 3, 3> calibration_rotation;
	extern Eigen::Matrix<float, 3, 1> calibration_translation;
	extern bool ismatrixcalibrated;
	extern Eigen::Vector3f calibration_origin;
	extern int cpoints;
	extern bool matrixes_calibrated;
	extern int psmmigi, psmhidari, psmyobu, psmatama;

	extern float hmdegree;
	extern bool sensorConfigChanged;

	extern vr::HmdVector3d_t kinect_m_positions[3];

	extern bool adjustingKinectRepresentationRot;
	extern bool adjustingKinectRepresentationPos;
	void updateKinectQuaternion();

	extern std::string KVRversion;
	extern glm::vec3 head_position, left_hand_pose, mHandPose, left_foot_raw_pose, right_foot_raw_pose, waist_raw_pose, hElPose, mElPose,
	                 lastPose[3][2];
	extern glm::quat left_foot_raw_ori, right_foot_raw_ori, waist_raw_ori;

	void sendipc();

	void serializeKinectSettings();
	void writeKinectSettings();
}

namespace SFMLsettings
{
	extern int m_window_width;
	extern int m_window_height;
	extern float windowScale;

	extern float globalFontSize;

	extern bool keepRunning;

	extern bool usingGamepad;

	extern std::wstring fileDirectoryPath;

	extern std::stringstream debugDisplayTextStream;
}

namespace vr
{
	// For Cereal to actually use the serialization methods, they have to
	// be in the same namespace as the declared object...
	template <class Archive>
	void serialize(Archive& archive,
	               HmdQuaternion_t& q)
	{
		archive(q.w, q.x, q.y, q.z);
	}

	template <class Archive>
	void serialize(Archive& archive,
	               HmdVector3d_t& v)
	{
		archive(v.v[0], v.v[1], v.v[2]);
	}

	bool operator==(const HmdVector3d_t& lhs, const HmdVector3d_t& rhs);
	bool operator==(const HmdQuaternion_t& lhs, const HmdQuaternion_t& rhs);
}

namespace KVR
{
	extern std::wstring trackerConfig;

	std::wstring fileToDirPath(const std::wstring& relativeFilePath);
	extern std::wstring ToUTF16(const std::string& data);

	extern std::string ToUTF8(const std::wstring& data);
	extern std::string inputDirForOpenVR(const std::string& file);

	// Each tracking system has it's global adjustments here, in the form
	// of their driver-from-world offsets, so that they can be reapplied at startup
	struct TrackingSystemCalibration
	{
		std::string systemName = "INVALID";
		vr::HmdQuaternion_t driverFromWorldRotation = {1, 0, 0, 0};
		vr::HmdVector3d_t driverFromWorldPosition = {0, 0, 0};
	};

	TrackingSystemCalibration retrieveSystemCalibration(const std::string& systemName);
	void saveSystemCalibration(const std::string& systemName, TrackingSystemCalibration calibration);
}

namespace VRInput
{
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

	extern vr::InputAnalogActionData_t trackpadpose[2];
	extern vr::InputDigitalActionData_t confirmdatapose;

	bool initialiseVRInput();
	void updateVRInput();
}

# define M_PI           3.14159265358979323846
