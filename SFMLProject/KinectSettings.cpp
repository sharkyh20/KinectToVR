#include "stdafx.h"
#include "KinectSettings.h"
#include <cereal/archives/json.hpp>

#include <iostream>
#include <fstream>

namespace KinectSettings {
    #define CFG_NAME "KinectToVR.cfg"
    std::string KVRversion = "a0.5.0";


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

    vr::HmdVector3_t hmdZero; //TEMP GLOBAL
    vr::HmdQuaternion_t kinectRepRotation{0,0,0,0};  //TEMP
    vr::HmdVector3_t kinectRadRotation{0,0,0};
    vr::HmdVector3_t kinectRepPosition{0,0,0};
    bool adjustingKinectRepresentationRot = false;
    bool adjustingKinectRepresentationPos = false;

    void serializeKinectSettings() {
        std::ifstream is(CFG_NAME);
        //CHECK IF VALID
        if (is.fail()) {
            //FAIL!!!!
            std::cerr << "ERROR: COULD NOT OPEN CONFIG FILE, GENERATING NEW ONE...";
            writeKinectSettings();
        }
        else {
            cereal::JSONInputArchive archive(is);
            using namespace KinectSettings;
            float rot[3] = { 0,0,0 };
            float pos[3] = { 0,0,0 };
            double hipHeight = 0;
            try {
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
        }
    }

    void writeKinectSettings() {
        std::ofstream os(CFG_NAME);
        cereal::JSONOutputArchive archive(os);

        if (os.fail()) {
            //FAIL!!!
            std::cerr << "ERROR: COULD NOT WRITE TO CONFIG FILE\n";
        }
        else {
            using namespace KinectSettings;
            vr::HmdVector3_t rot = kinectRadRotation;
            float kRotation[3] = { rot.v[0], rot.v[1] , rot.v[2] };

            vr::HmdVector3_t pos = kinectRepPosition;
            float kPosition[3] = { pos.v[0], pos.v[1] , pos.v[2] };
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

    bool usingGamepad = false;
    std::stringstream debugDisplayTextStream;
}
# define M_PI           3.14159265358979323846




