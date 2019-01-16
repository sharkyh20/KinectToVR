#include "stdafx.h"
#include "KinectSettings.h"
#include <cereal/archives/json.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/array.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/types/common.hpp>

#include <iostream>
#include <fstream>

#include "wtypes.h"
#include <Windows.h>
#include <codecvt>

#include <openvr_math.h>



namespace KinectSettings {
    std::wstring const CFG_NAME(L"KinectToVR.cfg");
    std::string KVRversion = "a0.6.0 Prime-time Segfault Test 1";

    bool isKinectDrawn = false;
    bool isSkeletonDrawn = false;

    bool ignoreInferredPositions = false;
    bool ignoreRotationSmoothing = false;

    // The joints which actually have rotation change based on the kinect
    // Each kinect type should set these in their process beginning
    // These would be the defaults for the V1
    KVR::KinectJointType leftFootJointWithRotation = KVR::KinectJointType::FootLeft;
    KVR::KinectJointType rightFootJointWithRotation = KVR::KinectJointType::FootRight;
    KVR::KinectJointType leftFootJointWithoutRotation = KVR::KinectJointType::AnkleLeft;
    KVR::KinectJointType rightFootJointWithoutRotation = KVR::KinectJointType::AnkleRight;

    bool userChangingZero = false;

    float g_TrackedBoneThickness = 6.0f;
    float g_InferredBoneThickness = 1.5f;
    float g_JointThickness = 4.0f;

    const int kinectHeight = 640;
    const int kinectWidth = 480;

    const int kinectV2Height = 1920;
    const int kinectV2Width = 1080;

    double kinectToVRScale = 1;
    double hipRoleHeightAdjust = 0.0;   // in metres up - applied post-scale
                                        //Need to delete later (Merge should sort it)
    int leftHandPlayspaceMovementButton = 0;
    int rightHandPlayspaceMovementButton = 0;
    int leftFootPlayspaceMovementButton = 0;
    int rightFootPlayspaceMovementButton = 0;

    vr::HmdVector3d_t hmdPosition = { 0,0,0 };
    vr::HmdQuaternion_t hmdRotation = { 0,0,0,0 };
    vr::HmdMatrix34_t hmdAbsoluteTracking = {};
    extern vr::HmdMatrix34_t trackingOrigin = {};
    extern vr::HmdVector3d_t trackingOriginPosition = { 0,0,0 };
    vr::HmdVector3d_t secondaryTrackingOriginOffset = { 0 };
    vr::HmdQuaternion_t kinectRepRotation{0,0,0,0};  //TEMP
    vr::HmdVector3d_t kinectRadRotation{0,0,0};
    vr::HmdVector3d_t kinectRepPosition{0,0,0};
    bool sensorConfigChanged = true; // First time used, it's config has changed internally

    bool adjustingKinectRepresentationRot = false;
    bool adjustingKinectRepresentationPos = false;
    void updateKinectQuaternion() {
        KinectSettings::kinectRepRotation = vrmath::quaternionFromYawPitchRoll(KinectSettings::kinectRadRotation.v[1], KinectSettings::kinectRadRotation.v[0], KinectSettings::kinectRadRotation.v[2]);
    }

    void serializeKinectSettings() {
        std::ifstream is(KVR::fileToDirPath(CFG_NAME));
        LOG(INFO) << "Attempted CFG load: " << KVR::fileToDirPath(CFG_NAME) << '\n';
        //CHECK IF VALID
        if (is.fail()) {
            //FAIL!!!!
            LOG(ERROR) << "ERROR: COULD NOT OPEN CONFIG FILE, GENERATING NEW ONE...";
            writeKinectSettings();
        }
        else {
            LOG(INFO) << "CFG Loaded Attempted!";
            
            using namespace KinectSettings;
            using namespace SFMLsettings;
            float rot[3] = { 0,0,0 };
            float pos[3] = { 0,0,0 };
            double hipHeight = 0;
            float fontScale = 12.f;
            
            try {
                cereal::JSONInputArchive archive(is);
                archive(rot);
                archive(pos);
                archive(hipHeight);
                archive(globalFontSize);
                archive(secondaryTrackingOriginOffset);
            }
            catch(cereal::RapidJSONException & e){
                LOG(ERROR) << "CONFIG FILE LOAD JSON ERROR: " << e.what();
            }
            kinectRadRotation = { rot[0], rot[1], rot[2] };
            kinectRepPosition = { pos[0], pos[1], pos[2] };
            hipRoleHeightAdjust = hipHeight;
            KinectSettings::sensorConfigChanged = true;
        }
    }

    void writeKinectSettings() {
        std::ofstream os(KVR::fileToDirPath(CFG_NAME));
        if (os.fail()) {
            //FAIL!!!
            LOG(ERROR) << "ERROR: COULD NOT WRITE TO CONFIG FILE\n";
        }
        else {
            using namespace KinectSettings;
            using namespace SFMLsettings;
            vr::HmdVector3d_t rot = kinectRadRotation;
            float kRotation[3] = { rot.v[0], rot.v[1] , rot.v[2] };

            vr::HmdVector3d_t pos = kinectRepPosition;
            float kPosition[3] = { pos.v[0], pos.v[1] , pos.v[2] };
            cereal::JSONOutputArchive archive(os);
            LOG(INFO) << "Attempted to save config settings to file";
            try {
                archive(
                    CEREAL_NVP(kRotation),
                    CEREAL_NVP(kPosition),
                    CEREAL_NVP(hipRoleHeightAdjust),
                    CEREAL_NVP(globalFontSize),
                    CEREAL_NVP(secondaryTrackingOriginOffset)
                );
            }
            catch (cereal::RapidJSONException & e) {
                LOG(ERROR) << "CONFIG FILE SAVE JSON ERROR: " << e.what();
            }
            
        }
    }
}
namespace SFMLsettings {
    int m_window_width = 800;
    int m_window_height = 600;
    float windowScale = .4f;
    bool keepRunning = true;

    float globalFontSize = 12.f;

    std::wstring fileDirectoryPath;

    bool usingGamepad = false;
    std::stringstream debugDisplayTextStream;

    
    
    
}
namespace vr {
    bool operator== (const vr::HmdVector3d_t &lhs, const vr::HmdVector3d_t &rhs) {
        return lhs.v[0] == rhs.v[0]
            && lhs.v[1] == rhs.v[1]
            && lhs.v[2] == rhs.v[2];
    }
    bool operator== (const vr::HmdQuaternion_t &lhs, const vr::HmdQuaternion_t &rhs) {
        return lhs.w == rhs.w
            && lhs.x == rhs.x
            && lhs.y == rhs.y
            && lhs.z == rhs.z;
    }
}
namespace KVR {
    std::wstring trackerConfig = L"lastTrackers.cfg";

    std::wstring fileToDirPath(const std::wstring & relativeFilePath) {
        return SFMLsettings::fileDirectoryPath + relativeFilePath;
    }
    std::wstring ToUTF16(const std::string & data)
    {
        return std::wstring_convert<std::codecvt_utf8<wchar_t>>().from_bytes(data);
    }

    std::string ToUTF8(const std::wstring &data)
    {
        return std::wstring_convert<std::codecvt_utf8<wchar_t>>().to_bytes(data);
    }
    std::string inputDirForOpenVR(const std::string & file) {
        std::string pathStr = ToUTF8(SFMLsettings::fileDirectoryPath) + "Input\\" + file;
        std::cout << file << " PATH: " << pathStr << '\n';
        return pathStr;
    }

    TrackingSystemCalibration retrieveSystemCalibration(const std::string & systemName) {
        std::wstring trackingSystemConfig = ToUTF16(systemName) + L".tracking";
        std::ifstream is(KVR::fileToDirPath(trackingSystemConfig));
        LOG(INFO) << "Attempted tracking system load: " << KVR::fileToDirPath(trackingSystemConfig) << '\n';

        TrackingSystemCalibration calibration;

        //CHECK IF VALID
        if (is.fail()) {
            //FAIL!!!!
            LOG(ERROR) << "ERROR: COULD NOT OPEN " << systemName << " TRACKING FILE, GENERATING NEW ONE...";
            saveSystemCalibration(systemName, calibration);
        }
        else {
            LOG(INFO) << systemName << "Tracking Load Attempted!";

            vr::HmdQuaternion_t driverFromWorldRotation = { 1,0,0,0 };
            vr::HmdVector3d_t driverFromWorldPosition = { 0,0,0 };

            try {
                cereal::JSONInputArchive archive(is);
                archive(CEREAL_NVP(driverFromWorldRotation));
                archive(CEREAL_NVP(driverFromWorldPosition));
            }
            catch (cereal::Exception & e) {
                LOG(ERROR) << systemName << "TRACKING FILE LOAD JSON ERROR: " << e.what();
            }
            
            calibration.systemName = systemName;
            calibration.driverFromWorldRotation = driverFromWorldRotation;
            calibration.driverFromWorldPosition = driverFromWorldPosition;
        }
        return calibration;
    }
    void saveSystemCalibration(const std::string & systemName, TrackingSystemCalibration calibration) {
        std::wstring trackingSystemConfig = ToUTF16(systemName) + L".tracking";
        std::ofstream os(KVR::fileToDirPath(trackingSystemConfig));
        if (os.fail()) {
            //FAIL!!!
            LOG(ERROR) << "ERROR: COULD NOT WRITE TO TRACKING SYSTEM FILE\n";
        }
        else {
            cereal::JSONOutputArchive archive(os);
            LOG(INFO) << "Attempted to save " << systemName << " tracking system to file";
            
            vr::HmdQuaternion_t driverFromWorldRotation = calibration.driverFromWorldRotation;
            vr::HmdVector3d_t driverFromWorldPosition = calibration.driverFromWorldPosition;

            try {
                archive(CEREAL_NVP(driverFromWorldRotation));
                archive(CEREAL_NVP(driverFromWorldPosition));
            }
            catch (cereal::RapidJSONException & e) {
                LOG(ERROR) << systemName << "TRACKING FILE SAVE JSON ERROR: " << e.what();
            }

        }
    }

}
# define M_PI           3.14159265358979323846

namespace VRInput {
    bool legacyInputModeEnabled;

    // Action Handles
    vr::VRActionHandle_t moveHorizontallyHandle;
    vr::VRActionHandle_t moveVerticallyHandle;
    vr::VRActionHandle_t confirmCalibrationHandle;

    // Calibration Sets
    vr::VRActionSetHandle_t calibrationSetHandle;

    // Action Sets
    vr::VRActiveActionSet_t activeActionSet;

    // Digital Action Data
    vr::InputDigitalActionData_t confirmCalibrationData{};

    // Analog Action Data
    vr::InputAnalogActionData_t moveHorizontallyData{};
    vr::InputAnalogActionData_t moveVerticallyData{};
}
bool VRInput::initialiseVRInput()
{
    std::string path = KVR::inputDirForOpenVR("action-manifest.json");
    const char * c_path = path.c_str();
    vr::EVRInputError iError = vr::VRInput()->SetActionManifestPath(c_path);
    if (iError == vr::EVRInputError::VRInputError_None) {
        LOG(INFO) << "Action manifest path set correctly!";
    }
    else {
        LOG(ERROR) << "Action manifest path Error, EVRInputError Code: " << (int)iError;
        return false;
    }
    // Obtain handles
    iError = vr::VRInput()->GetActionHandle("/actions/calibration/in/MoveHorizontally", &moveHorizontallyHandle);
    iError = vr::VRInput()->GetActionHandle("/actions/calibration/in/MoveVertically", &moveVerticallyHandle);
    iError = vr::VRInput()->GetActionHandle("/actions/calibration/in/ConfirmCalibration", &confirmCalibrationHandle);

    iError = vr::VRInput()->GetActionSetHandle("/actions/calibration", &calibrationSetHandle);

    // Set Actionset Settings
    activeActionSet.ulActionSet = calibrationSetHandle;
    activeActionSet.ulRestrictedToDevice = vr::k_ulInvalidInputValueHandle;
    activeActionSet.unPadding; // Ignored
    activeActionSet.nPriority = 0;

    if (iError == vr::EVRInputError::VRInputError_None) {
        LOG(INFO) << "Input Handles set correctly!";
    }
    else {
        LOG(ERROR) << "Input Handle Error, EVRInputError Code: " << (int)iError;
        return false;
    }
    return true;
}

void VRInput::updateVRInput()
{
    vr::EVRInputError iError = vr::VRInput()->UpdateActionState(&activeActionSet, sizeof(activeActionSet), 1);
    if (iError != vr::EVRInputError::VRInputError_None) {
        LOG(ERROR) << "Error when updating input action state, EVRInputError Code: " << (int)iError;
        return;
    }
    vr::InputAnalogActionData_t moveHorizontallyData{};
    iError = vr::VRInput()->GetAnalogActionData(
        moveHorizontallyHandle,
        &moveHorizontallyData,
        sizeof(moveHorizontallyData),
        vr::k_ulInvalidInputValueHandle);

    vr::InputAnalogActionData_t moveVerticallyData{};
    iError = vr::VRInput()->GetAnalogActionData(
        moveVerticallyHandle,
        &moveVerticallyData,
        sizeof(moveVerticallyData),
        vr::k_ulInvalidInputValueHandle);

    vr::InputDigitalActionData_t confirmPosData{};
    iError = vr::VRInput()->GetDigitalActionData(
        confirmCalibrationHandle,
        &confirmPosData,
        sizeof(confirmPosData),
        vr::k_ulInvalidInputValueHandle);

    // Ugly Hack until Valve fixes this behaviour ---------
    if (iError == vr::EVRInputError::VRInputError_InvalidHandle) {
        // SteamVR's latest wonderful bug/feature:
        // Switches to Legacy mode on any application it doesn't recognize
        // Meaning that the new system isn't used at all...
        // Why god. Why do you taunt me so?
        VRInput::legacyInputModeEnabled = true;
    }
    else {
        VRInput::legacyInputModeEnabled = false;
    }
    // -----------------------------------------------------
}
