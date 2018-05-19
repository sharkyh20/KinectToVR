#pragma once
#include "stdafx.h"
#include <IKinectHandler.h>
#include <KinectHandlerBase.h>
#include "KinectJointFilter.h"
#include "KinectDoubleExponentialRotationFilter.h"

#include <opencv2/opencv.hpp>
// Kinect V2 - directory local due to my win 7 machine being unsupported for actual install

#include <Kinect.h>
class KinectV2Handler : public KinectHandlerBase {
public:
    KinectV2Handler() {
        initialise();
        initOpenGL();
    }
    virtual ~KinectV2Handler() {}

    DoubleExponentialFilter filter;
    DoubleExpBoneOrientationsFilter rotationFilter;
    IKinectSensor* kinectSensor = nullptr;
    //IMultiSourceFrameReader* frameReader = nullptr;
    IBodyFrameReader* bodyFrameReader = nullptr;
    IColorFrameReader* colorFrameReader = nullptr;
    IDepthFrameReader* depthFrameReader = nullptr;
    IMultiSourceFrame* multiFrame = nullptr;
    ICoordinateMapper* coordMapper = nullptr;
    BOOLEAN isTracking = false;

    

    Joint joints[JointType_Count];
    JointOrientation jointOrientations[JointType_Count];
    IBody* kinectBodies[BODY_COUNT];

    // TODO: Convert the current opengl stuff into buffer objects
    GLuint kinectTextureId;   // color buffer object for the K
                                   //GLuint vertexBufferObjectId;    // unused, as depth data is not to be displayed

    virtual HRESULT getStatusResult();
    virtual std::string statusResultString(HRESULT stat);

    virtual void initialise();

    virtual void initialiseSkeleton();
    virtual void initialiseColor();
    virtual void initialiseDepth();

    virtual void terminateSkeleton();
    virtual void terminateColor();
    virtual void terminateDepth();

    virtual void initOpenGL();
    virtual void update();
    void updateColorData();
    void updateDepthData();

    virtual void drawKinectData(sf::RenderWindow &win);
    virtual void drawKinectImageData(sf::RenderWindow &win);
    virtual void drawTrackedSkeletons(sf::RenderWindow &win);

    

    bool convertColorToDepthResolution = false;
    /*
    virtual bool putRGBDataIntoMatrix(cv::Mat& image) override {
        
        //image.create(1080, 1920, CV_8UC4);
        //BYTE* imgDataPtr = (BYTE*)image.data;
        //imgDataPtr = kinectImageData.get();
        //
        //return true;
        

        const unsigned int img_size = 1920 * 1080 * 4;
        image = cv::Mat(1080, 1920, CV_8UC4);

        // copy data
        memcpy(image.data, kinectImageData.get(), img_size);

        return true;
    }
    */
    void onBodyFrameArrived(IBodyFrameReader& sender, IBodyFrameArrivedEventArgs& eventArgs);
    virtual void updateSkeletalData();
    virtual void zeroAllTracking(vr::IVRSystem* &m_sys);
    virtual void updateTrackersWithSkeletonPosition(
        vrinputemulator::VRInputEmulator &emulator,
        std::vector<KVR::KinectTrackedDevice> trackers);
    virtual void updateTrackersWithColorPosition(vrinputemulator::VRInputEmulator &emulator,
        std::vector<KVR::KinectTrackedDevice> trackers, sf::Vector2i pos);
    JointType convertJoint(KVR::KinectJoint joint);
private:
    bool initKinect();
    void updateKinectData();

    sf::Vector3f zeroKinectPosition(int trackedSkeletonIndex);
    void setKinectToVRMultiplier(int skeletonIndex);



    bool getFilteredJoint(KVR::KinectTrackedDevice device, vr::HmdVector3_t& position, vr::HmdQuaternion_t &rotation);
    sf::Vector2f BodyToScreen(const CameraSpacePoint& bodyPoint, int width, int height) {
        // Calculate the body's position on the screen
        DepthSpacePoint depthPoint = { 0 };
        coordMapper->MapCameraPointToDepthSpace(bodyPoint, &depthPoint);

        static const int        cDepthWidth = 512;
        static const int        cDepthHeight = 424;

        float screenPointX = static_cast<float>(depthPoint.X * width) / cDepthWidth;
        float screenPointY = static_cast<float>(depthPoint.Y * height) / cDepthHeight;

        return sf::Vector2f(screenPointX, screenPointY);
    }
    void drawBody(const Joint* pJoints, const sf::Vector2f* pJointPoints, sf::RenderWindow &window);
    void drawHand(HandState handState, const sf::Vector2f& handPosition, sf::RenderWindow &win);
    void drawBone(const Joint* pJoints, const sf::Vector2f* pJointPoints, JointType joint0, JointType joint1, sf::RenderWindow &window);
    void drawLine(sf::Vector2f start, sf::Vector2f end, sf::Color colour, float lineThickness, sf::RenderWindow &window);
};

