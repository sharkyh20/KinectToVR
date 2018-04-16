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

    virtual HRESULT getStatusResult() { return E_NOTIMPL; }
    virtual std::string statusResultString(HRESULT stat) { return "statusResultString behaviour not defined"; };

    virtual void update() {};

    virtual bool putRGBDataIntoMatrix(cv::Mat& image) { return false; }
    virtual void drawKinectData(sf::RenderWindow &win) {};  // Houses the below draw functions with a check
    virtual void drawKinectImageData(sf::RenderWindow &win) {};
    virtual void drawTrackedSkeletons(sf::RenderWindow &win) {};

    virtual void zeroAllTracking(vr::IVRSystem* &m_sys) {};
    virtual void updateTrackersWithSkeletonPosition(
        vrinputemulator::VRInputEmulator &emulator,
        std::vector<KVR::KinectTrackedDevice> trackers
    ) {};
};