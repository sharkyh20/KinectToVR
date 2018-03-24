#pragma once
#include "IKinectHandler.h"
class KinectHandlerBase : public IKinectHandler {
public:
    KinectHandlerBase() {

    }
    ~KinectHandlerBase() {

    }
    virtual void initOpenGL() {};
    virtual void initialise() {};

    virtual HRESULT getStatusResult() { return E_NOTIMPL; }
    virtual std::string statusResultString(HRESULT stat) { return "statusResultString behaviour not defined"; };

    virtual void update() {};

    virtual void drawKinectData(sf::RenderWindow &win) {};  // Houses the below draw functions with a check
    virtual void drawKinectImageData(sf::RenderWindow &win) {};
    virtual void drawTrackedSkeletons(sf::RenderWindow &win) {};

    virtual void zeroAllTracking(vr::IVRSystem* &m_sys) {};
    virtual void updateTrackersWithSkeletonPosition(
        vrinputemulator::VRInputEmulator &emulator,
        std::vector<KVR::KinectTrackedDevice> trackers
    ) {};
};