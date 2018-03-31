#pragma once
#include "stdafx.h"
#include "KinectV1Includes.h"
#include "KinectHandlerBase.h"
#include "KinectOrientationFilter.h"

class KinectV1Handler : public KinectHandlerBase {
    // A representation of the Kinect elements for the v1 api
public:
    KinectV1Handler()
    {
        initialise();
        initOpenGL();
    }
    HANDLE kinectRGBStream = nullptr;
    INuiSensor* kinectSensor = nullptr;
    RotationalSmoothingFilter rotFilter;
    GLuint kinectTextureId;    // ID of the texture to contain Kinect RGB Data
    NUI_SKELETON_FRAME skeletonFrame = { 0 };

    Vector4 jointPositions[NUI_SKELETON_POSITION_COUNT];
    NUI_SKELETON_BONE_ORIENTATION boneOrientations[NUI_SKELETON_POSITION_COUNT];

    sf::Vector2f screenSkelePoints[NUI_SKELETON_POSITION_COUNT];

    virtual void initialise();
    virtual void initOpenGL();
    virtual void update();

    virtual ~KinectV1Handler() {}

    virtual HRESULT getStatusResult();
    virtual std::string statusResultString(HRESULT stat);

    virtual void drawKinectData(sf::RenderWindow &win);
    virtual void drawKinectImageData(sf::RenderWindow &win);
    virtual void drawTrackedSkeletons(sf::RenderWindow &win);

    //Consider moving this tracking stuff into a seperate class
    virtual void zeroAllTracking(vr::IVRSystem* &m_sys);
    virtual void updateTrackersWithSkeletonPosition(
        vrinputemulator::VRInputEmulator &emulator,
        std::vector<KVR::KinectTrackedDevice> trackers);

    bool getFilteredJoint(KVR::KinectTrackedDevice device, vr::HmdVector3_t& position, vr::HmdQuaternion_t &rotation);
    NUI_SKELETON_POSITION_INDEX convertJoint(KVR::KinectJoint joint);
private:
    bool initKinect();
    void getKinectRGBData();
    bool acquireKinectFrame(NUI_IMAGE_FRAME &imageFrame, HANDLE & rgbStream, INuiSensor* &sensor);
    INuiFrameTexture* lockKinectPixelData(NUI_IMAGE_FRAME &imageFrame, NUI_LOCKED_RECT &LockedRect);
    void copyKinectPixelData(NUI_LOCKED_RECT &LockedRect, GLubyte* dest);
    void unlockKinectPixelData(INuiFrameTexture* texture);
    void releaseKinectFrame(NUI_IMAGE_FRAME &imageFrame, HANDLE& rgbStream, INuiSensor* &sensor);

    void updateSkeletalData();
    void DrawSkeleton(const NUI_SKELETON_DATA & skel, sf::RenderWindow &window);
    sf::Vector2f SkeletonToScreen(Vector4 skeletonPoint, int _width, int _height);
    void DrawBone(const NUI_SKELETON_DATA & skel, NUI_SKELETON_POSITION_INDEX joint0,
        NUI_SKELETON_POSITION_INDEX joint1, sf::RenderWindow &window);
    void DrawLine(sf::Vector2f start, sf::Vector2f end, sf::Color colour, float lineThickness, sf::RenderWindow &window);
    Vector4 zeroKinectPosition(int trackedSkeletonIndex);
    void setKinectToVRMultiplier(int skeletonIndex);

    bool jointsUntracked(KVR::KinectJoint joint0, KVR::KinectJoint joint1, NUI_SKELETON_DATA data);
    bool jointsInferred(KVR::KinectJoint joint0, KVR::KinectJoint joint1, NUI_SKELETON_DATA data);
};