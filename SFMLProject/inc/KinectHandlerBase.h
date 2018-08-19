#pragma once
#include "IKinectHandler.h"
#include <opencv2\opencv.hpp>
class KinectHandlerBase : public IKinectHandler {
public:
    KinectHandlerBase() {

    }
    ~KinectHandlerBase() {

    }
    bool convertColorToDepthResolution = false;
    // Color Buffer
    std::vector<BYTE> colorBuffer;
    int colorWidth;
    int colorHeight;
    unsigned int colorBytesPerPixel;
    cv::Mat colorMat;

    // Depth Buffer
    std::vector<UINT16> depthBuffer;
    int depthWidth;
    int depthHeight;
    unsigned int depthBytesPerPixel;
    cv::Mat depthMat;

    virtual void initOpenGL() {};
    virtual void initialise() {};

    virtual void initialiseSkeleton() {};
    virtual void initialiseColor() {};
    virtual void initialiseDepth() {};

    virtual void terminateSkeleton() {};
    virtual void terminateColor() {};
    virtual void terminateDepth() {};

    virtual HRESULT getStatusResult() { return E_NOTIMPL; }
    virtual std::string statusResultString(HRESULT stat) { return "statusResultString behaviour not defined"; };

    virtual void update() {};

    virtual bool putRGBDataIntoMatrix(cv::Mat& image) { return false; }
    virtual void drawKinectData(sf::RenderWindow &win) {};  // Houses the below draw functions with a check
    virtual void drawKinectImageData(sf::RenderWindow &win) {};
    virtual void drawTrackedSkeletons(sf::RenderWindow &win) {};

    virtual bool getFilteredJoint(KVR::KinectTrackedDevice device, vr::HmdVector3d_t& position, vr::HmdQuaternion_t &rotation) { return false; };

    virtual void zeroAllTracking(vr::IVRSystem* &m_sys) {};
    virtual void updateTrackersWithSkeletonPosition(
        std::vector<KVR::KinectTrackedDevice> trackers
    ) {};
    virtual void updateTrackersWithColorPosition(
        std::vector<KVR::KinectTrackedDevice> trackers, sf::Vector2i pos) {}
};