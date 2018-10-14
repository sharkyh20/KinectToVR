#include "stdafx.h"
#include "KinectSettings.h"
#include <cereal/archives/json.hpp>

#include <iostream>
#include <fstream>

#include "wtypes.h"
#include <Windows.h>
#include <codecvt>

#include <openvr_math.h>

namespace KinectSettings {
    std::wstring const CFG_NAME(L"KinectToVR.cfg");
    std::string KVRversion = "a0.5.6";

    bool isKinectDrawn = false;
    bool isSkeletonDrawn = false;

    bool ignoreInferredPositions = false;
    bool ignoreRotationSmoothing = false;

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

    vr::HmdQuaternion_t kinectRepRotation{0,0,0,0};  //TEMP
    vr::HmdVector3d_t kinectRadRotation{0,0,0};
    vr::HmdVector3d_t kinectRepPosition{0,0,0};
    bool sensorConfigChanged = false;

    bool adjustingKinectRepresentationRot = false;
    bool adjustingKinectRepresentationPos = false;
    void updateKinectQuaternion() {
        KinectSettings::kinectRepRotation = vrmath::quaternionFromYawPitchRoll(KinectSettings::kinectRadRotation.v[1], KinectSettings::kinectRadRotation.v[0], KinectSettings::kinectRadRotation.v[2]);
    }

    void serializeKinectSettings() {
        std::ifstream is(KVR::fileToDirPath(CFG_NAME));
        std::wcout << "Attempted CFG load: " << KVR::fileToDirPath(CFG_NAME) << '\n';
        //CHECK IF VALID
        if (is.fail()) {
            //FAIL!!!!
            std::cerr << "ERROR: COULD NOT OPEN CONFIG FILE, GENERATING NEW ONE...";
            writeKinectSettings();
        }
        else {
            std::cout << "CFG Loaded Attempted!\n";
            
            using namespace KinectSettings;
            float rot[3] = { 0,0,0 };
            float pos[3] = { 0,0,0 };
            double hipHeight = 0;
            
            try {
                cereal::JSONInputArchive archive(is);
                archive(rot);
                archive(pos);
                archive(hipHeight);
            }
            catch(cereal::RapidJSONException e){
                std::cerr << "CONFIG FILE LOAD ERROR: " << e.what() << '\n';
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
            std::cerr << "ERROR: COULD NOT WRITE TO CONFIG FILE\n";
        }
        else {
            using namespace KinectSettings;
            vr::HmdVector3d_t rot = kinectRadRotation;
            float kRotation[3] = { rot.v[0], rot.v[1] , rot.v[2] };

            vr::HmdVector3d_t pos = kinectRepPosition;
            float kPosition[3] = { pos.v[0], pos.v[1] , pos.v[2] };
            cereal::JSONOutputArchive archive(os);
            archive(
                CEREAL_NVP(kRotation),
                CEREAL_NVP(kPosition),
                CEREAL_NVP(hipRoleHeightAdjust)
            );
        }
    }
}
namespace SFMLsettings {
    int m_window_width = 800;
    int m_window_height = 600;
    float windowScale = .6f;
    bool keepRunning = true;

    std::wstring fileDirectoryPath;

    bool usingGamepad = false;
    std::stringstream debugDisplayTextStream;

    
    
    
}
namespace KVR {
    std::wstring fileToDirPath(std::wstring relativeFilePath) {
        return SFMLsettings::fileDirectoryPath + relativeFilePath;
    }
    std::wstring ToUTF16(const std::string &data)
    {
        return std::wstring_convert<std::codecvt_utf8<wchar_t>>().from_bytes(data);
    }

    std::string ToUTF8(const std::wstring &data)
    {
        return std::wstring_convert<std::codecvt_utf8<wchar_t>>().to_bytes(data);
    }
    const char* inputDirForOpenVR(std::string file) {
        const char* path;

        std::string pathStr = ToUTF8(SFMLsettings::fileDirectoryPath) + "Input\\" + file;
        path = pathStr.c_str();

        std::cout << file << " PATH: " << path << '\n';
        return path;
    }
}
# define M_PI           3.14159265358979323846




