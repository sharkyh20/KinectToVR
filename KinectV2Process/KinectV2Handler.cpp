#pragma once
#include "stdafx.h"
#include "KinectV2Handler.h"
#include <sfLine.h>
#include <iostream>
#include <VRHelper.h>
#include "KinectJointFilter.h"
#include <Eigen/Geometry>
#include <ppl.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/transform2.hpp>
#include <glm/mat4x4.hpp>
#include <glm/detail/type_vec3.hpp>
#include <glm/detail/type_vec4.hpp>
#include <glm/detail/type_vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp> 
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <thread>
#include <chrono>

HRESULT KinectV2Handler::getStatusResult()
{
    BOOLEAN avail;
    kinectSensor->get_IsAvailable(&avail);
    if (avail)
        return S_OK;
    else
        return S_FALSE; // Hresult only actually determines whether the function worked, the bool is the true value....
}

std::string KinectV2Handler::statusResultString(HRESULT stat)
{
    switch (stat) {
    case S_OK: return "S_OK";
    case S_FALSE: return "Sensor Unavailable! Check if it's plugged in to your USB and power plugs";
    default: return "Uh Oh undefined kinect error! " + std::to_string(stat);
    }
}

void KinectV2Handler::initialise() {
    try {
        kVersion = KinectVersion::Version2;
        kinectImageData = std::make_unique<GLubyte[]>(KinectSettings::kinectV2Width * KinectSettings::kinectV2Height * 4);  //RGBA
        initialised = initKinect();
        // initialiseColor(); 
        // Commented both image frames out, as most people use the kinect for skeletal data
        // Updating all of the arrays uses a shit ton of CPU, but then again, it's still WIP
        // initialiseDepth();
        initialiseSkeleton();
        if (!initialised) throw FailedKinectInitialisation;
    }
    catch (std::exception& e) {
        LOG(ERROR) << e.what() << std::endl;
    }
}
void KinectV2Handler::initialiseSkeleton()
{
    if (bodyFrameReader)
        bodyFrameReader->Release();
    IBodyFrameSource* bodyFrameSource;
    kinectSensor->get_BodyFrameSource(&bodyFrameSource);
    bodyFrameSource->OpenReader(&bodyFrameReader);

    // Newfangled event based frame capture
    // https://github.com/StevenHickson/PCL_Kinect2SDK/blob/master/src/Microsoft_grabber2.cpp
    h_bodyFrameEvent = (WAITABLE_HANDLE)CreateEvent(NULL, FALSE, FALSE, NULL);
    HRESULT hr = bodyFrameReader->SubscribeFrameArrived(&h_bodyFrameEvent);
    if (bodyFrameSource) bodyFrameSource->Release();
    if (FAILED(hr)) {
        //throw std::exception("Couldn't subscribe frame");
        LOG(ERROR) << "ERROR: Could not subscribe to skeleton frame event! HRESULT " << hr;
    }
    else {
        LOG(INFO) << "Kinect Skeleton Reader subscribed to event, initialised successfully.";
    }
}
void KinectV2Handler::initialiseColor()
{
https://github.com/UnaNancyOwen/Kinect2Sample/blob/master/sample/CoordinateMapper/app.h
    if (colorFrameReader)
        colorFrameReader->Release();
    // Open Color Reader
    IColorFrameSource* colorFrameSource;
    kinectSensor->get_ColorFrameSource(&colorFrameSource);
    HRESULT hr = colorFrameSource->OpenReader(&colorFrameReader);

    // Retrieve Color Description
    IFrameDescription* colorFrameDescription;
    colorFrameSource->CreateFrameDescription(ColorImageFormat::ColorImageFormat_Bgra, &colorFrameDescription);
    colorFrameDescription->get_Width(&colorWidth); // 1920
    colorFrameDescription->get_Height(&colorHeight); // 1080
    colorFrameDescription->get_BytesPerPixel(&colorBytesPerPixel); // 4

                                                                                // Allocation Color Buffer
    colorBuffer.resize(colorWidth * colorHeight * colorBytesPerPixel);
    if (colorFrameSource) colorFrameSource->Release();
    if (colorFrameDescription) colorFrameDescription->Release();

    if (FAILED(hr)) {
        LOG(ERROR) << "Kinect Color Reader could not be opened! HRESULT " << hr;
    }
    else
        LOG(INFO) << "Kinect Color Reader initialised successfully.";
}
void KinectV2Handler::initialiseDepth()
{
https://github.com/UnaNancyOwen/Kinect2Sample/blob/master/sample/CoordinateMapper/app.h
    if (depthFrameReader) depthFrameReader->Release();

    // Open Depth Reader
    IDepthFrameSource* depthFrameSource;
    kinectSensor->get_DepthFrameSource(&depthFrameSource);
    HRESULT hr = depthFrameSource->OpenReader(&depthFrameReader);

    // Retrieve Depth Description
    IFrameDescription* depthFrameDescription;
    depthFrameSource->get_FrameDescription(&depthFrameDescription);
    depthFrameDescription->get_Width(&depthWidth); // 512
    depthFrameDescription->get_Height(&depthHeight); // 424
    depthFrameDescription->get_BytesPerPixel(&depthBytesPerPixel); // 2
                                                                                // Allocation Depth Buffer
    depthBuffer.resize(depthWidth * depthHeight);

    if (depthFrameSource) depthFrameSource->Release();
    if (depthFrameDescription) depthFrameDescription->Release();

    if (FAILED(hr)) {
        LOG(ERROR) << "Kinect Depth Reader could not be opened! HRESULT " << hr;
    }
    else
        LOG(INFO) << "Kinect Depth Reader initialised successfully.";
}
void KinectV2Handler::terminateSkeleton()
{
    if (bodyFrameReader) {
        HRESULT hr = bodyFrameReader->UnsubscribeFrameArrived(h_bodyFrameEvent);
        if (FAILED(hr)) {
            LOG(ERROR) << "Couldn't unsubscribe skeleton frame! HRESULT " << hr;
            throw std::exception("Couldn't unsubscribe frame!");
        }
        CloseHandle((HANDLE)h_bodyFrameEvent);
        h_bodyFrameEvent = NULL;

        bodyFrameReader->Release();
        bodyFrameReader = nullptr;
        LOG(INFO) << "Skeleton Reader closed successfully";
    }
    else
        LOG(WARNING) << "Skeleton Reader was asked to terminate, but was already closed!";
}
void KinectV2Handler::terminateColor()
{
    if (colorFrameReader) {
        colorFrameReader->Release();
        colorFrameReader = nullptr;
        colorMat.release();
        LOG(INFO) << "Color Reader closed successfully";
    }
    else
        LOG(WARNING) << "Color Reader was asked to terminate, but was already closed!";
}
void KinectV2Handler::terminateDepth()
{
    if (depthFrameReader) {
        depthFrameReader->Release();
        depthFrameReader = nullptr;
        depthMat.release();
        LOG(INFO) << "Depth Reader closed successfully";
    }
    else
        LOG(WARNING) << "Depth Reader was asked to terminate, but was already closed!";
}

void KinectV2Handler::initOpenGL()
{
    LOG(INFO) << "Attempted to initialise OpenGL";
    int width = 0, height = 0;
    if (kVersion == KinectVersion::Version1) {
        width = KinectSettings::kinectWidth;
        height = KinectSettings::kinectHeight;
    }
    else if (kVersion == KinectVersion::Version2) {
        width = KinectSettings::kinectV2Width;
        height = KinectSettings::kinectV2Height;
    }   // REMOVE THIS INTO KINECT V2 IMPL
        // Initialize textures
    glGenTextures(1, &kinectTextureId);
    glBindTexture(GL_TEXTURE_2D, kinectTextureId);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height,
        0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, (GLvoid*)kinectImageData.get());
    glBindTexture(GL_TEXTURE_2D, 0);

    // OpenGL setup
    glClearColor(1, 0, 0, 0);
    glClearDepth(1.0f);
    glEnable(GL_TEXTURE_2D);

    // Camera setup
    glViewport(0, 0, SFMLsettings::m_window_width, SFMLsettings::m_window_height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, SFMLsettings::m_window_width, SFMLsettings::m_window_height, 0, 1, -1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}
void KinectV2Handler::update()
{
    if (isInitialised()) {
        BOOLEAN isAvailable = false;
        HRESULT kinectStatus = kinectSensor->get_IsAvailable(&isAvailable);
        if (kinectStatus == S_OK) {
            // NEW ARRIVED FRAMES ------------------------
            MSG msg;
            while (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE)) // Unneccesary?
            {
                DispatchMessage(&msg);
            }

            if (h_bodyFrameEvent)
            {
                //printf("Kinect Event ID: %d\n" ,(int)h_bodyFrameEvent);

                //now check for IR Events
                HANDLE handles[] = { reinterpret_cast<HANDLE>(h_bodyFrameEvent) }; // , reinterpret_cast<HANDLE>(ke.hMSEvent)		};

                switch (MsgWaitForMultipleObjects(_countof(handles), handles, false, 0, QS_ALLINPUT))
                {
                case WAIT_OBJECT_0:
                {
                    IBodyFrameArrivedEventArgs* pArgs = nullptr;
                    //printf("Body Frame Event Signaled.\n");

                    if (bodyFrameReader)
                    {
                        HRESULT hr = bodyFrameReader->GetFrameArrivedEventData(h_bodyFrameEvent, &pArgs);
                        //printf("Retreive Frame Arrive Event Data -HR: %d\n", hr);

                        if (SUCCEEDED(hr))
                        {
                            //printf("Retreived Frame Arrived Event Data\n");
                            onBodyFrameArrived(*bodyFrameReader, *pArgs);
                            pArgs->Release();
                            //printf("Frame Arrived Event Data Released\n");
                        }
                    }
                }
                break;
                }
            }
            // ------------------------------
            updateKinectData();
        }
    }
}
template <typename T>
void updateBufferWithSmoothedMat(cv::Mat &in, cv::Mat &out, std::vector<T> &buffer) {
    int filterIntensity = 5; //MUST be odd
    cv::medianBlur(in, out, filterIntensity);
    

    if (out.isContinuous()) {
        buffer.assign((T*)out.datastart, (T*)out.dataend);
    }
    else {
        for (int i = 0; i < out.rows; ++i) {
            buffer.insert(buffer.end(), out.ptr<T>(i), out.ptr<T>(i) + out.cols);
        }
    }
}
void KinectV2Handler::updateColorData()
{
    if (colorFrameReader) {
        IColorFrame* colorFrame;
        const HRESULT retrieveFrame = colorFrameReader->AcquireLatestFrame(&colorFrame);
        if (FAILED(retrieveFrame)) {
            LOG(ERROR) << "Could not retrieve color frame! HRESULT " << retrieveFrame;
        }
        else { // Necessary instead of instant return to prevent memory leak of colorFrame
            //Convert from YUY2 -> BGRA
            colorFrame->CopyConvertedFrameDataToArray(static_cast<UINT>(colorBuffer.size()), &colorBuffer[0], ColorImageFormat::ColorImageFormat_Bgra);
            colorMat = cv::Mat(colorHeight, colorWidth, CV_8UC4, &colorBuffer[0]);

            updateBufferWithSmoothedMat(colorMat, colorMat, colorBuffer);
        }
        if (colorFrame) colorFrame->Release();
    }
}

void KinectV2Handler::updateDepthData()
{
    if (depthFrameReader) {
        // Retrieve Depth Frame
        IDepthFrame* depthFrame;
        const HRESULT retrieveFrame = depthFrameReader->AcquireLatestFrame(&depthFrame);
        if (FAILED(retrieveFrame)) {
            LOG(ERROR) << "Could not retrieve depth frame! HRESULT " << retrieveFrame;
        }
        else {// Necessary instead of instant return to prevent memory leak of depthFrame
            // Retrieve Depth Data
            depthFrame->CopyFrameDataToArray(static_cast<UINT>(depthBuffer.size()), &depthBuffer[0]);

            // Create cv::Mat from Depth Buffer
            auto unsmoothedDepthMat = cv::Mat(depthHeight, depthWidth, CV_16U, &depthBuffer[0]);
            updateBufferWithSmoothedMat(unsmoothedDepthMat, depthMat, depthBuffer);
        }
        if (depthFrame) depthFrame->Release();
    }
}
void KinectV2Handler::drawKinectData(sf::RenderWindow &win) {
    if (KinectSettings::isKinectDrawn) {
        drawKinectImageData(win);
    }
    if (KinectSettings::isSkeletonDrawn) {
        drawTrackedSkeletons(win);
    }
}
void KinectV2Handler::drawKinectImageData(sf::RenderWindow &win) {
    glBindTexture(GL_TEXTURE_2D, kinectTextureId);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, SFMLsettings::m_window_width, SFMLsettings::m_window_height, GL_BGRA, GL_UNSIGNED_BYTE, (GLvoid*)kinectImageData.get());
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glBegin(GL_QUADS);
    glTexCoord2f(0.0f, 0.0f);
    glVertex3f(0, 0, 0);
    glTexCoord2f(1.0f, 0.0f);
    glVertex3f(SFMLsettings::m_window_width, 0, 0);
    glTexCoord2f(1.0f, 1.0f);
    glVertex3f(SFMLsettings::m_window_width, SFMLsettings::m_window_height, 0.0f);
    glTexCoord2f(0.0f, 1.0f);
    glVertex3f(0, SFMLsettings::m_window_height, 0.0f);
    glEnd();
}
void KinectV2Handler::drawTrackedSkeletons(sf::RenderWindow &win) {
    for (int i = 0; i < BODY_COUNT; ++i) {
        IBody* pBody = kinectBodies[i];
        if (pBody)
        {
            BOOLEAN bTracked = false;
            HRESULT thisBodyTracked = pBody->get_IsTracked(&bTracked);
            if (SUCCEEDED(thisBodyTracked) && bTracked)
            {
                Joint joints[JointType_Count];
                sf::Vector2f jointPoints[JointType_Count];
                HandState leftHandState = HandState_Unknown;
                HandState rightHandState = HandState_Unknown;

                pBody->get_HandLeftState(&leftHandState);
                pBody->get_HandRightState(&rightHandState);

                HRESULT jointsFound = pBody->GetJoints(_countof(joints), joints);
                if (SUCCEEDED(jointsFound))
                {
                    for (int j = 0; j < _countof(joints); ++j)
                    {
                        jointPoints[j] = BodyToScreen(joints[j].Position, SFMLsettings::m_window_width, SFMLsettings::m_window_height);
                    }

                    if (KinectSettings::isSkeletonDrawn) {
                        win.pushGLStates();
                        win.resetGLStates();

                        drawBody(joints, jointPoints, win);

                        drawHand(leftHandState, jointPoints[JointType_HandLeft], win);
                        drawHand(rightHandState, jointPoints[JointType_HandRight], win);

                        win.popGLStates();
                    }
                    
                }
            }
        }
    }
}
void KinectV2Handler::drawHand(HandState handState, const sf::Vector2f& handPosition, sf::RenderWindow &win)
{
    sf::CircleShape circle{};
    circle.setPosition(handPosition);
    static const float c_HandSize = 30.0f;
    circle.setRadius(c_HandSize);

    switch (handState)
    {
    case HandState_Closed:
        circle.setFillColor(sf::Color::Red);
        win.draw(circle);
        break;

    case HandState_Open:
        circle.setFillColor(sf::Color::Green);
        win.draw(circle);
        break;

    case HandState_Lasso:
        circle.setFillColor(sf::Color::Blue);
        win.draw(circle);
        break;
    }
}
void KinectV2Handler::onBodyFrameArrived(IBodyFrameReader& sender, IBodyFrameArrivedEventArgs& eventArgs) {
    updateSkeletalData();
}
void KinectV2Handler::updateSkeletalData() {
    if (bodyFrameReader) {
        IBodyFrame* bodyFrame = nullptr;
        HRESULT frameReceived = bodyFrameReader->AcquireLatestFrame(&bodyFrame);
        if (FAILED(frameReceived)) {
            if (frameReceived == E_PENDING) {
                // LOG(INFO) << "Could not retrieve skeleton frame, stuck pending...";
                // Harmless
            }
            else
                LOG(ERROR) << "Could not retrieve skeleton frame! HRESULT " << frameReceived;
        }

        //IBodyFrameReference* frameRef = nullptr;
        //multiFrame->get_BodyFrameReference(&frameRef);
        //frameRef->AcquireFrame(&bodyFrame);
        //if (frameRef) frameRef->Release();
        //if (bodyFrameReader) bodyFrameReader->Release();
        if (!bodyFrame) return;

        bodyFrame->GetAndRefreshBodyData(BODY_COUNT, kinectBodies);
        newBodyFrameArrived = true;
        if (bodyFrame) bodyFrame->Release();

        updateSkeletalFilters();
    }
}

static bool flip = false;
void KinectV2Handler::updateSkeletalFilters() {
    for (int i = 0; i < BODY_COUNT; i++) {
        if (kinectBodies[i])
            kinectBodies[i]->get_IsTracked(&isTracking);
        if (isTracking) {
            kinectBodies[i]->GetJoints(JointType_Count, joints);
            kinectBodies[i]->GetJointOrientations(JointType_Count, jointOrientations);

            //Smooth
            filter.update(joints, newBodyFrameArrived);
            rotationFilter.UpdateFilter(kinectBodies[i], jointOrientations);

            newBodyFrameArrived = false;

            break;
        }
    }


	//joints[JointType_HandRight].Position.X

    KinectSettings::mposes[0].v[0] = joints[JointType_Head].Position.X;
    KinectSettings::mposes[0].v[1] = joints[JointType_Head].Position.Y;
    KinectSettings::mposes[0].v[2] = joints[JointType_Head].Position.Z;

    vr::HmdVector3d_t trotation[3] = { {0,0,0},{0,0,0},{0,0,0} };

#pragma region PipeSeup

    HANDLE pipeTracker = CreateFile(TEXT("\\\\.\\pipe\\LogPipeTracker"), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    HANDLE pipeSan = CreateFile(TEXT("\\\\.\\pipe\\LogPipeSan"), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    HANDLE pipeLeft = CreateFile(TEXT("\\\\.\\pipe\\LogPipeNi"), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    HANDLE pipeRight = CreateFile(TEXT("\\\\.\\pipe\\LogPipeIchi"), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    HANDLE pipeLeftRot = CreateFile(TEXT("\\\\.\\pipe\\LogPipeNiRot"), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    HANDLE pipeRightRot = CreateFile(TEXT("\\\\.\\pipe\\LogPipeIchiRot"), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);

#pragma endregion

    DWORD numWritten;

#pragma region TrackerIPC

    float yaw = KinectSettings::hmdYaw * 180 / M_PI;
    float facing = yaw - KinectSettings::tryaw;

    if (facing < 25 && facing > -25)flip = false;
    if (facing < -155 && facing > -205)flip = true;

    std::string TrackerS = [&]()->std::string {
        std::stringstream S;

        using PointSet = Eigen::Matrix<float, 3, Eigen::Dynamic>;

#pragma region Rotation_Hips

        glm::vec3 hipLeft(joints[JointType_HipLeft].Position.X, joints[JointType_HipLeft].Position.Y, joints[JointType_HipLeft].Position.Z),
            hipRight(joints[JointType_HipRight].Position.X, joints[JointType_HipRight].Position.Y, joints[JointType_HipRight].Position.Z);

        glm::quat hipsRot = glm::lookAt(hipLeft, hipRight, glm::vec3(0.0f, 1.0f, 0.0f));
        glm::vec3 hipsRotRad = glm::eulerAngles(hipsRot);
        trotation[2].v[1] = -((hipsRotRad.y * 180.f / M_PI) + 90.f);
#pragma endregion
#pragma region Rotation_Ankles

        glm::vec3 ankle[2] = { glm::vec3(joints[JointType_AnkleLeft].Position.X, joints[JointType_AnkleLeft].Position.Y, joints[JointType_AnkleLeft].Position.Z),
            glm::vec3(joints[JointType_AnkleRight].Position.X, joints[JointType_AnkleRight].Position.Y, joints[JointType_AnkleRight].Position.Z) };

        glm::vec3 foot[2] = { glm::vec3(joints[JointType_FootLeft].Position.X, joints[JointType_FootLeft].Position.Y, joints[JointType_FootLeft].Position.Z),
            glm::vec3(joints[JointType_FootRight].Position.X, joints[JointType_FootRight].Position.Y, joints[JointType_FootRight].Position.Z) };

        glm::quat ankleRot[2] = { glm::lookAt(ankle[0], foot[0], glm::vec3(0.0f, 1.0f, 0.0f)), glm::lookAt(ankle[1], foot[1], glm::vec3(0.0f, 1.0f, 0.0f)) };
        glm::vec3 ankleRotRad[2] = { glm::eulerAngles(ankleRot[0]), glm::eulerAngles(ankleRot[1]) };

        trotation[0].v[1] = -(ankleRotRad[0].y * 180.f / M_PI);
        trotation[1].v[1] = -(ankleRotRad[1].y * 180.f / M_PI);
#pragma endregion

        if (KinectSettings::rtcalibrated) {
            Eigen::Vector3f Hf, Mf, Hp;
            if (!flip) {
                Hf(0) = joints[JointType_AnkleLeft].Position.X;
                Hf(1) = joints[JointType_AnkleLeft].Position.Y;
                Hf(2) = joints[JointType_AnkleLeft].Position.Z;

                Mf(0) = joints[JointType_AnkleRight].Position.X;
                Mf(1) = joints[JointType_AnkleRight].Position.Y;
                Mf(2) = joints[JointType_AnkleRight].Position.Z;
            }
            else {
                trotation[0].v[1] += 180;
                trotation[1].v[1] += 180;
                trotation[2].v[1] += 180;

                Mf(0) = joints[JointType_AnkleLeft].Position.X;
                Mf(1) = joints[JointType_AnkleLeft].Position.Y;
                Mf(2) = joints[JointType_AnkleLeft].Position.Z;

                Hf(0) = joints[JointType_AnkleRight].Position.X;
                Hf(1) = joints[JointType_AnkleRight].Position.Y;
                Hf(2) = joints[JointType_AnkleRight].Position.Z;
            }

            Hp(0) = joints[JointType_SpineBase].Position.X;
            Hp(1) = joints[JointType_SpineBase].Position.Y;
            Hp(2) = joints[JointType_SpineBase].Position.Z;

            PointSet Hf2 = (KinectSettings::R_matT * Hf).colwise() + KinectSettings::T_matT;
            PointSet Mf2 = (KinectSettings::R_matT * Mf).colwise() + KinectSettings::T_matT;
            PointSet Hp2 = (KinectSettings::R_matT * Hp).colwise() + KinectSettings::T_matT;

            S << "HX" << 10000 * (Hf2(0) + KinectSettings::moffsets[0][1].v[0] + KinectSettings::troffsets.v[0]) <<
                "/HY" << 10000 * (Hf2(1) + KinectSettings::moffsets[0][1].v[1] + KinectSettings::troffsets.v[1]) <<
                "/HZ" << 10000 * (Hf2(2) + KinectSettings::moffsets[0][1].v[2] + KinectSettings::troffsets.v[2]) <<
                "/MX" << 10000 * (Mf2(0) + KinectSettings::moffsets[0][0].v[0] + KinectSettings::troffsets.v[0]) <<
                "/MY" << 10000 * (Mf2(1) + KinectSettings::moffsets[0][0].v[1] + KinectSettings::troffsets.v[1]) <<
                "/MZ" << 10000 * (Mf2(2) + KinectSettings::moffsets[0][0].v[2] + KinectSettings::troffsets.v[2]) <<
                "/PX" << 10000 * (Hp2(0) + KinectSettings::moffsets[0][2].v[0] + KinectSettings::troffsets.v[0]) <<
                "/PY" << 10000 * (Hp2(1) + KinectSettings::moffsets[0][2].v[1] + KinectSettings::troffsets.v[1]) <<
                "/PZ" << 10000 * (Hp2(2) + KinectSettings::moffsets[0][2].v[2] + KinectSettings::troffsets.v[2]) <<
                "/HRX" << 10000 * (trotation[0].v[0] + KinectSettings::moffsets[1][1].v[0]) * M_PI / 180 <<
                "/HRY" << 10000 * (trotation[0].v[1] + KinectSettings::tryaw + KinectSettings::moffsets[1][1].v[1]) * M_PI / 180 <<
                "/HRZ" << 10000 * (trotation[0].v[2] + KinectSettings::moffsets[1][1].v[2]) * M_PI / 180 <<
                "/MRX" << 10000 * (trotation[1].v[0] + KinectSettings::moffsets[1][0].v[0]) * M_PI / 180 <<
                "/MRY" << 10000 * (trotation[1].v[1] + KinectSettings::tryaw + KinectSettings::moffsets[1][0].v[1]) * M_PI / 180 <<
                "/MRZ" << 10000 * (trotation[1].v[2] + KinectSettings::moffsets[1][0].v[2]) * M_PI / 180 <<
                "/PRX" << 10000 * (trotation[2].v[0] + KinectSettings::moffsets[1][2].v[0]) * M_PI / 180 <<
                "/PRY" << 10000 * (trotation[2].v[1] + KinectSettings::tryaw + KinectSettings::moffsets[1][2].v[1]) * M_PI / 180 <<
                "/PRZ" << 10000 * (trotation[2].v[2] + KinectSettings::moffsets[1][2].v[2]) * M_PI / 180 <<
                "/WRW" << 10000 * (0) << "/"; //DEPRECATED: GLM_ROTATE SCREWED UP WITH > 99
        }
        else {
            if (!flip) {
                S << "HX" << 10000 * (joints[JointType_AnkleLeft].Position.X + KinectSettings::moffsets[0][1].v[0] + KinectSettings::troffsets.v[0]) <<
                    "/HY" << 10000 * (joints[JointType_AnkleLeft].Position.Y + KinectSettings::moffsets[0][1].v[1] + KinectSettings::troffsets.v[1]) <<
                    "/HZ" << 10000 * (joints[JointType_AnkleLeft].Position.Z + KinectSettings::moffsets[0][1].v[2] + KinectSettings::troffsets.v[2]) <<
                    "/MX" << 10000 * (joints[JointType_AnkleRight].Position.X + KinectSettings::moffsets[0][0].v[0] + KinectSettings::troffsets.v[0]) <<
                    "/MY" << 10000 * (joints[JointType_AnkleRight].Position.Y + KinectSettings::moffsets[0][0].v[1] + KinectSettings::troffsets.v[1]) <<
                    "/MZ" << 10000 * (joints[JointType_AnkleRight].Position.Z + KinectSettings::moffsets[0][0].v[2] + KinectSettings::troffsets.v[2]) <<
                    "/PX" << 10000 * (joints[JointType_SpineBase].Position.X + KinectSettings::moffsets[0][2].v[0] + KinectSettings::troffsets.v[0]) <<
                    "/PY" << 10000 * (joints[JointType_SpineBase].Position.Y + KinectSettings::moffsets[0][2].v[1] + KinectSettings::troffsets.v[1]) <<
                    "/PZ" << 10000 * (joints[JointType_SpineBase].Position.Z + KinectSettings::moffsets[0][2].v[2] + KinectSettings::troffsets.v[2]) <<
                    "/HRX" << 10000 * (trotation[0].v[0] + KinectSettings::moffsets[1][1].v[0]) * M_PI / 180 <<
                    "/HRY" << 10000 * (trotation[0].v[1] + KinectSettings::moffsets[1][1].v[1]) * M_PI / 180 <<
                    "/HRZ" << 10000 * (trotation[0].v[2] + KinectSettings::moffsets[1][1].v[2]) * M_PI / 180 <<
                    "/MRX" << 10000 * (trotation[1].v[0] + KinectSettings::moffsets[1][0].v[0]) * M_PI / 180 <<
                    "/MRY" << 10000 * (trotation[1].v[1] + KinectSettings::moffsets[1][0].v[1]) * M_PI / 180 <<
                    "/MRZ" << 10000 * (trotation[1].v[2] + KinectSettings::moffsets[1][0].v[2]) * M_PI / 180 <<
                    "/PRX" << 10000 * (trotation[2].v[0] + KinectSettings::moffsets[1][2].v[0]) * M_PI / 180 <<
                    "/PRY" << 10000 * (trotation[2].v[1] + KinectSettings::moffsets[1][2].v[1]) * M_PI / 180 <<
                    "/PRZ" << 10000 * (trotation[2].v[2] + KinectSettings::moffsets[1][2].v[2]) * M_PI / 180 <<
                    "/WRW" << 10000 * (0) << "/"; //DEPRECATED: GLM_ROTATE SCREWED UP WITH > 99
            }
            else {
                trotation[0].v[1] += 180;
                trotation[1].v[1] += 180;
                trotation[2].v[1] += 180;

                S << "HX" << 10000 * (joints[JointType_AnkleRight].Position.X + KinectSettings::moffsets[0][1].v[0] + KinectSettings::troffsets.v[0]) <<
                    "/HY" << 10000 * (joints[JointType_AnkleRight].Position.Y + KinectSettings::moffsets[0][1].v[1] + KinectSettings::troffsets.v[1]) <<
                    "/HZ" << 10000 * (joints[JointType_AnkleRight].Position.Z + KinectSettings::moffsets[0][1].v[2] + KinectSettings::troffsets.v[2]) <<
                    "/MX" << 10000 * (joints[JointType_AnkleLeft].Position.X + KinectSettings::moffsets[0][0].v[0] + KinectSettings::troffsets.v[0]) <<
                    "/MY" << 10000 * (joints[JointType_AnkleLeft].Position.Y + KinectSettings::moffsets[0][0].v[1] + KinectSettings::troffsets.v[1]) <<
                    "/MZ" << 10000 * (joints[JointType_AnkleLeft].Position.Z + KinectSettings::moffsets[0][0].v[2] + KinectSettings::troffsets.v[2]) <<
                    "/PX" << 10000 * (joints[JointType_SpineBase].Position.X + KinectSettings::moffsets[0][2].v[0] + KinectSettings::troffsets.v[0]) <<
                    "/PY" << 10000 * (joints[JointType_SpineBase].Position.Y + KinectSettings::moffsets[0][2].v[1] + KinectSettings::troffsets.v[1]) <<
                    "/PZ" << 10000 * (joints[JointType_SpineBase].Position.Z + KinectSettings::moffsets[0][2].v[2] + KinectSettings::troffsets.v[2]) <<
                    "/HRX" << 10000 * (trotation[0].v[0] + KinectSettings::moffsets[1][1].v[0]) * M_PI / 180 <<
                    "/HRY" << 10000 * (trotation[0].v[1] + KinectSettings::moffsets[1][1].v[1]) * M_PI / 180 <<
                    "/HRZ" << 10000 * (trotation[0].v[2] + KinectSettings::moffsets[1][1].v[2]) * M_PI / 180 <<
                    "/MRX" << 10000 * (trotation[1].v[0] + KinectSettings::moffsets[1][0].v[0]) * M_PI / 180 <<
                    "/MRY" << 10000 * (trotation[1].v[1] + KinectSettings::moffsets[1][0].v[1]) * M_PI / 180 <<
                    "/MRZ" << 10000 * (trotation[1].v[2] + KinectSettings::moffsets[1][0].v[2]) * M_PI / 180 <<
                    "/PRX" << 10000 * (trotation[2].v[0] + KinectSettings::moffsets[1][2].v[0]) * M_PI / 180 <<
                    "/PRY" << 10000 * (trotation[2].v[1] + KinectSettings::moffsets[1][2].v[1]) * M_PI / 180 <<
                    "/PRZ" << 10000 * (trotation[2].v[2] + KinectSettings::moffsets[1][2].v[2]) * M_PI / 180 <<
                    "/WRW" << 10000 * (0) << "/"; //DEPRECATED: GLM_ROTATE SCREWED UP WITH > 99
            }
        }

        return S.str();
    }();

#pragma endregion

    std::string HedoS = [&]()->std::string {
        std::stringstream S;

        Eigen::AngleAxisd rollAngle(0.f, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd yawAngle(KinectSettings::hroffset * M_PI / 180, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd pitchAngle(0.f, Eigen::Vector3d::UnitX());

        Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

        Eigen::Vector3d in(joints[JointType_Head].Position.X,
            joints[JointType_Head].Position.Y,
            joints[JointType_Head].Position.Z);

        Eigen::Vector3d out = q * in;


        S << "X" << 10000 * (out(0) + KinectSettings::hoffsets.v[0]) <<
            "/Y" << 10000 * (out(1) + KinectSettings::hoffsets.v[1]) <<
            "/Z" << 10000 * (out(2) + KinectSettings::hoffsets.v[2]) << "/";

        return S.str();
    }();

    std::string RightS = [&]()->std::string {
        std::stringstream S;

        if (KinectSettings::rtconcalib) {
            using PointSet = Eigen::Matrix<float, 3, Eigen::Dynamic>;

            Eigen::Vector3f Hf, Ef;
            if (!flip) {
                Hf(0) = joints[JointType_HandRight].Position.X;
                Hf(1) = joints[JointType_HandRight].Position.Y;
                Hf(2) = joints[JointType_HandRight].Position.Z;
                Ef(0) = joints[JointType_ElbowRight].Position.X;
                Ef(1) = joints[JointType_ElbowRight].Position.Y;
                Ef(2) = joints[JointType_ElbowRight].Position.Z;
            }
            else {
                trotation[0].v[1] = 180;
                trotation[1].v[1] = 180;
                trotation[2].v[1] = 180;

                Hf(0) = joints[JointType_HandLeft].Position.X;
                Hf(1) = joints[JointType_HandLeft].Position.Y;
                Hf(2) = joints[JointType_HandLeft].Position.Z;
                Ef(0) = joints[JointType_ElbowLeft].Position.X;
                Ef(1) = joints[JointType_ElbowLeft].Position.Y;
                Ef(2) = joints[JointType_ElbowLeft].Position.Z;
            }

            PointSet Hf2 = (KinectSettings::R_matT * Hf).colwise() + KinectSettings::T_matT;
            PointSet Mf2 = (KinectSettings::R_matT * Ef).colwise() + KinectSettings::T_matT;

            S << "X" << 10000 * (Hf(0) + KinectSettings::hoffsets.v[0] + KinectSettings::mauoffset.v[0]) <<
                "/Y" << 10000 * (Hf(1) + KinectSettings::hoffsets.v[1] + KinectSettings::mauoffset.v[1]) <<
                "/Z" << 10000 * (Hf(2) + KinectSettings::hoffsets.v[2] + KinectSettings::mauoffset.v[2]) <<
                "/EX" << 10000 * (Ef(0) + KinectSettings::hoffsets.v[0] + KinectSettings::mauoffset.v[0]) <<
                "/EY" << 10000 * (Ef(1) + KinectSettings::hoffsets.v[1] + KinectSettings::mauoffset.v[1]) <<
                "/EZ" << 10000 * (Ef(2) + KinectSettings::hoffsets.v[2] + KinectSettings::mauoffset.v[2]) << "/";
        }
        else {
            if (!flip) {
                S << "X" << 10000 * (joints[JointType_HandRight].Position.X + KinectSettings::hoffsets.v[0] + KinectSettings::mauoffset.v[0]) <<
                    "/Y" << 10000 * (joints[JointType_HandRight].Position.Y + KinectSettings::hoffsets.v[1] + KinectSettings::mauoffset.v[1]) <<
                    "/Z" << 10000 * (joints[JointType_HandRight].Position.Z + KinectSettings::hoffsets.v[2] + KinectSettings::mauoffset.v[2]) <<
                    "/EX" << 10000 * (joints[JointType_ElbowRight].Position.X + KinectSettings::hoffsets.v[0] + KinectSettings::mauoffset.v[0]) <<
                    "/EY" << 10000 * (joints[JointType_ElbowRight].Position.Y + KinectSettings::hoffsets.v[1] + KinectSettings::mauoffset.v[1]) <<
                    "/EZ" << 10000 * (joints[JointType_ElbowRight].Position.Z + KinectSettings::hoffsets.v[2] + KinectSettings::mauoffset.v[2]) << "/";
            }
            else {
                S << "X" << 10000 * (joints[JointType_HandLeft].Position.X + KinectSettings::hoffsets.v[0] + KinectSettings::mauoffset.v[0]) <<
                    "/Y" << 10000 * (joints[JointType_HandLeft].Position.Y + KinectSettings::hoffsets.v[1] + KinectSettings::mauoffset.v[1]) <<
                    "/Z" << 10000 * (joints[JointType_HandLeft].Position.Z + KinectSettings::hoffsets.v[2] + KinectSettings::mauoffset.v[2]) <<
                    "/EX" << 10000 * (joints[JointType_ElbowLeft].Position.X + KinectSettings::hoffsets.v[0] + KinectSettings::mauoffset.v[0]) <<
                    "/EY" << 10000 * (joints[JointType_ElbowLeft].Position.Y + KinectSettings::hoffsets.v[1] + KinectSettings::mauoffset.v[1]) <<
                    "/EZ" << 10000 * (joints[JointType_ElbowLeft].Position.Z + KinectSettings::hoffsets.v[2] + KinectSettings::mauoffset.v[2]) << "/";
            }
        }

        return S.str();
    }();

    std::string LeftS = [&]()->std::string {
        std::stringstream S;

        if (KinectSettings::rtconcalib) {
            using PointSet = Eigen::Matrix<float, 3, Eigen::Dynamic>;

            Eigen::Vector3f Hf, Ef;
            if (flip) {
                Hf(0) = joints[JointType_HandRight].Position.X;
                Hf(1) = joints[JointType_HandRight].Position.Y;
                Hf(2) = joints[JointType_HandRight].Position.Z;
                Ef(0) = joints[JointType_ElbowRight].Position.X;
                Ef(1) = joints[JointType_ElbowRight].Position.Y;
                Ef(2) = joints[JointType_ElbowRight].Position.Z;
            }
            else {
                trotation[0].v[1] = 180;
                trotation[1].v[1] = 180;
                trotation[2].v[1] = 180;

                Hf(0) = joints[JointType_HandLeft].Position.X;
                Hf(1) = joints[JointType_HandLeft].Position.Y;
                Hf(2) = joints[JointType_HandLeft].Position.Z;
                Ef(0) = joints[JointType_ElbowLeft].Position.X;
                Ef(1) = joints[JointType_ElbowLeft].Position.Y;
                Ef(2) = joints[JointType_ElbowLeft].Position.Z;
            }

            PointSet Hf2 = (KinectSettings::R_matT * Hf).colwise() + KinectSettings::T_matT;
            PointSet Mf2 = (KinectSettings::R_matT * Ef).colwise() + KinectSettings::T_matT;

            S << "X" << 10000 * (Hf(0) + KinectSettings::hoffsets.v[0] + KinectSettings::hauoffset.v[0]) <<
                "/Y" << 10000 * (Hf(1) + KinectSettings::hoffsets.v[1] + KinectSettings::hauoffset.v[1]) <<
                "/Z" << 10000 * (Hf(2) + KinectSettings::hoffsets.v[2] + KinectSettings::hauoffset.v[2]) <<
                "/EX" << 10000 * (Ef(0) + KinectSettings::hoffsets.v[0] + KinectSettings::hauoffset.v[0]) <<
                "/EY" << 10000 * (Ef(1) + KinectSettings::hoffsets.v[1] + KinectSettings::hauoffset.v[1]) <<
                "/EZ" << 10000 * (Ef(2) + KinectSettings::hoffsets.v[2] + KinectSettings::hauoffset.v[2]) << "/";
        }
        else {
            if (flip) {
                S << "X" << 10000 * (joints[JointType_HandRight].Position.X + KinectSettings::hoffsets.v[0] + KinectSettings::hauoffset.v[0]) <<
                    "/Y" << 10000 * (joints[JointType_HandRight].Position.Y + KinectSettings::hoffsets.v[1] + KinectSettings::hauoffset.v[1]) <<
                    "/Z" << 10000 * (joints[JointType_HandRight].Position.Z + KinectSettings::hoffsets.v[2] + KinectSettings::hauoffset.v[2]) <<
                    "/EX" << 10000 * (joints[JointType_ElbowRight].Position.X + KinectSettings::hoffsets.v[0] + KinectSettings::hauoffset.v[0]) <<
                    "/EY" << 10000 * (joints[JointType_ElbowRight].Position.Y + KinectSettings::hoffsets.v[1] + KinectSettings::hauoffset.v[1]) <<
                    "/EZ" << 10000 * (joints[JointType_ElbowRight].Position.Z + KinectSettings::hoffsets.v[2] + KinectSettings::hauoffset.v[2]) << "/";
            }
            else {
                S << "X" << 10000 * (joints[JointType_HandLeft].Position.X + KinectSettings::hoffsets.v[0] + KinectSettings::hauoffset.v[0]) <<
                    "/Y" << 10000 * (joints[JointType_HandLeft].Position.Y + KinectSettings::hoffsets.v[1] + KinectSettings::hauoffset.v[1]) <<
                    "/Z" << 10000 * (joints[JointType_HandLeft].Position.Z + KinectSettings::hoffsets.v[2] + KinectSettings::hauoffset.v[2]) <<
                    "/EX" << 10000 * (joints[JointType_ElbowLeft].Position.X + KinectSettings::hoffsets.v[0] + KinectSettings::hauoffset.v[0]) <<
                    "/EY" << 10000 * (joints[JointType_ElbowLeft].Position.Y + KinectSettings::hoffsets.v[1] + KinectSettings::hauoffset.v[1]) <<
                    "/EZ" << 10000 * (joints[JointType_ElbowLeft].Position.Z + KinectSettings::hoffsets.v[2] + KinectSettings::hauoffset.v[2]) << "/";
            }
        }

        return S.str();
    }();

    std::string RotS = [&]()->std::string {
        std::stringstream S;

        if (KinectSettings::rtconcalib) {

            Eigen::Vector3f Hf, Ef;
            if (flip) {
                S << "X" << 0.f <<
                    "/Y" << 10000 * KinectSettings::tryaw * 180 / M_PI <<
                    "/Z" << 10000 * 0.f << "/";
            }
            else {
                S << "X" << 0.f <<
                    "/Y" << 10000 * ((KinectSettings::tryaw * 180 / M_PI) + 180.f) <<
                    "/Z" << 0.f << "/";
            }
        }
        else {
            if (flip) {
                S << "X" << 0.f <<
                    "/Y" << 0.f <<
                    "/Z" << 0.f << "/";
            }
            else {
                S << "X" << 0.f <<
                    "/Y" << 180.f <<
                    "/Z" << 0.f << "/";
            }
        }

        return S.str();
    }();

    char TrackerD[1024];
    char HedoD[1024];
    char RightD[1024];
    char LeftD[1024];
    char RotD[1024];

    strcpy_s(TrackerD, TrackerS.c_str());
    strcpy_s(HedoD, HedoS.c_str());
    strcpy_s(RightD, RightS.c_str());
    strcpy_s(LeftD, LeftS.c_str());
    strcpy_s(RotD, RotS.c_str());

    WriteFile(pipeTracker, TrackerD, sizeof(TrackerD), &numWritten, NULL);
    WriteFile(pipeSan, HedoD, sizeof(HedoD), &numWritten, NULL);
    WriteFile(pipeRight, RightD, sizeof(RightD), &numWritten, NULL);
    WriteFile(pipeLeft, LeftD, sizeof(LeftD), &numWritten, NULL);
    WriteFile(pipeLeftRot, RotD, sizeof(RotD), &numWritten, NULL);
    WriteFile(pipeRightRot, RotD, sizeof(RotD), &numWritten, NULL);

    CloseHandle(pipeTracker);
    CloseHandle(pipeSan);
    CloseHandle(pipeRight);
    CloseHandle(pipeLeft);
    CloseHandle(pipeLeftRot);
    CloseHandle(pipeRightRot);



}
sf::Vector3f KinectV2Handler::zeroKinectPosition(int trackedSkeletonIndex) {
    return sf::Vector3f(
        joints[JointType_Head].Position.X,
        joints[JointType_Head].Position.Y, 
        joints[JointType_Head].Position.Z);
}
void KinectV2Handler::zeroAllTracking(vr::IVRSystem *& m_sys)
{
    /*
    for (int i = 0; i < BODY_COUNT; ++i) {
        TrackingState trackingState = joints->TrackingState;

        if (trackingState == TrackingState_Tracked) {
            KinectSettings::hmdZero = getHMDPosition(m_sys);
            setKinectToVRMultiplier(i);
            zeroed = true;
            break;
        }
    }
    */
}
void KinectV2Handler::setKinectToVRMultiplier(int skeletonIndex) {
    /*
    KinectSettings::kinectToVRScale = KinectSettings::hmdZero.v[1]
        / (joints[JointType_Head].Position.Y
            +
            -joints[JointType_AnkleLeft].Position.Y);
    std::cerr << "HMD zero: " << KinectSettings::hmdZero.v[1] << '\n';
    std::cerr << "head pos: " << joints[JointType_Head].Position.Y << '\n';
    std::cerr << "foot pos: " << joints[JointType_AnkleLeft].Position.Y << '\n';
    */
}

// THIS IS BEING DEPRECATED INTO A SIMPLE RETURN OF THE JOINTS.
// FROM NOW ON, EACH TRACKING-METHOD WILL HANDLE PROCESSING OF
// THIS DATA, AND THEN UPDATE THE TRACKERS IN ANOTHER METHOD
void KinectV2Handler::updateTrackersWithSkeletonPosition( std::vector<KVR::KinectTrackedDevice> & trackers)
{
    for (KVR::KinectTrackedDevice & device : trackers) {
        if (device.positionTrackingOption == KVR::JointPositionTrackingOption::Skeleton) {
            if (device.isSensor() ) {
                device.update(KinectSettings::kinectRepPosition, { 0,0,0 }, KinectSettings::kinectRepRotation);
                // Only update after the calibration value has been changed
                // e.g. on boot, calibration box change
            }
            else {
                vr::HmdVector3d_t jointPosition{ 0,0,0 };
                vr::HmdQuaternion_t jointRotation{ 0,0,0,0 };
                if (getFilteredJoint(device, jointPosition, jointRotation)) {

                    device.update(trackedPositionVROffset, jointPosition, jointRotation);
                }
            }
        }
    }
}
void KinectV2Handler::updateTrackersWithColorPosition( std::vector<KVR::KinectTrackedDevice> trackers, sf::Vector2i pos)
{
    //std::cerr << "Tracked Point: " << pos.x << ", " << pos.y << '\n';

    std::vector<CameraSpacePoint> resultPoints(colorWidth * colorHeight);
    HRESULT hr = coordMapper->MapColorFrameToCameraSpace(depthBuffer.size(), &depthBuffer[0], resultPoints.size(), &resultPoints[0]);
    if (SUCCEEDED(hr)) {
        int colorX = static_cast<int>(pos.x + 0.5f);
        int colorY = static_cast<int>(pos.y + 0.5f);
        long colorIndex = (long)(colorY * 1920 + colorX);
        CameraSpacePoint worldCoordinate = resultPoints[colorIndex];
        //auto worldCoordinate = resultArray[pos.y * 1920 + pos.x];
        //std::cerr << "World Point: " << worldCoordinate.X << ", " << worldCoordinate.Y << ", " << worldCoordinate.Z << '\n';
        if (isnan(worldCoordinate.X)) {
            return;
        }
        for (KVR::KinectTrackedDevice device : trackers) {
            if (device.positionTrackingOption == KVR::JointPositionTrackingOption::Color) {
                if (device.isSensor()) {
                    device.update(KinectSettings::kinectRepPosition, { 0,0,0 }, KinectSettings::kinectRepRotation);
                }
                else {
                    vr::HmdVector3d_t jointPosition{ 0,0,0 };
                    if (worldCoordinate.X + worldCoordinate.Y + worldCoordinate.Z != 0) {
                        jointPosition.v[0] = worldCoordinate.X;
                        jointPosition.v[1] = worldCoordinate.Y;
                        jointPosition.v[2] = worldCoordinate.Z;
                        device.update(trackedPositionVROffset, jointPosition, { 0,0,0,1 });
                    }
                }
            }
        }
    }
    //std::cout << "HR: " << hr << '\n';
}
bool KinectV2Handler::getFilteredJoint(KVR::KinectTrackedDevice device, vr::HmdVector3d_t& position, vr::HmdQuaternion_t &rotation) {
    sf::Vector3f filteredPos = filter.GetFilteredJoints()[convertJoint(device.joint0)];
    float jointX = filteredPos.x;
    float jointY = filteredPos.y;
    float jointZ = filteredPos.z;
    position = vr::HmdVector3d_t{ jointX,jointY,jointZ };

    //Rotation - need to seperate into function
    Vector4 kRotation;
    switch (device.rotationFilterOption) {
    case KVR::JointRotationFilterOption::Unfiltered:
        kRotation = jointOrientations[convertJoint(device.joint0)].Orientation;
        break;
    case KVR::JointRotationFilterOption::Filtered:
        kRotation = rotationFilter.GetFilteredJoints()[convertJoint(device.joint0)]; 
        break;
    case KVR::JointRotationFilterOption::HeadLook: {        // Ew
        auto q = KinectSettings::hmdRotation;
        //Isolate Yaw
        float yaw = atan2(2 * q.w*q.y + 2 * q.x*q.z, +q.w*q.w + q.x*q.x - q.z*q.z - q.y*q.y);

        auto kq = vrmath::quaternionFromRotationY(yaw);
        kRotation.w = kq.w;
        kRotation.x = kq.x;
        kRotation.y = kq.y;
        kRotation.z = kq.z;
    }
                                             break;
    default:
        LOG(ERROR) << "JOINT ROTATION OPTION UNDEFINED IN DEVICE " << device.deviceId << '\n';
        break;
    }
    rotation.w = kRotation.w;
    rotation.x = kRotation.x;
    rotation.y = kRotation.y;
    rotation.z = kRotation.z;
    
    return true;
}
bool KinectV2Handler::initKinect() {
    if (FAILED(GetDefaultKinectSensor(&kinectSensor))) {
        LOG(ERROR) << "Could not get default Kinect Sensor!";
        return false;
    }
    if (kinectSensor) {
        kinectSensor->get_CoordinateMapper(&coordMapper);

        HRESULT hr_open = kinectSensor->Open();
        //kinectSensor->OpenMultiSourceFrameReader( FrameSourceTypes::FrameSourceTypes_Body| FrameSourceTypes::FrameSourceTypes_Depth
         //    | FrameSourceTypes::FrameSourceTypes_Color,
         //   &frameReader);
        //return frameReader;
        std::this_thread::sleep_for(std::chrono::seconds(2)); // Necessary to allow kinect to become available behind the scenes

        BOOLEAN available = false;
        kinectSensor->get_IsAvailable(&available);

        if (FAILED(hr_open) || !available) {
            LOG(ERROR) << "Kinect sensor failed to open!";
            return false;
        }
        else LOG(INFO) << "Kinect sensor opened successfully.";
        return true;
    }
    return false;
}
void KinectV2Handler::updateKinectData() {
    updateDepthData();
    updateColorData();
}

 
void KinectV2Handler::drawBody(const Joint * pJoints, const sf::Vector2f * pJointPoints, sf::RenderWindow & window)
{
    // Draw the bones

    // Torso
    drawBone(pJoints, pJointPoints, JointType_Head, JointType_Neck, window);
    drawBone(pJoints, pJointPoints, JointType_Neck, JointType_SpineShoulder, window);
    drawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_SpineMid, window);
    drawBone(pJoints, pJointPoints, JointType_SpineMid, JointType_SpineBase, window);
    drawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_ShoulderRight, window);
    drawBone(pJoints, pJointPoints, JointType_SpineShoulder, JointType_ShoulderLeft, window);
    drawBone(pJoints, pJointPoints, JointType_SpineBase, JointType_HipRight, window);
    drawBone(pJoints, pJointPoints, JointType_SpineBase, JointType_HipLeft, window);

    // Right Arm    
    drawBone(pJoints, pJointPoints, JointType_ShoulderRight, JointType_ElbowRight, window);
    drawBone(pJoints, pJointPoints, JointType_ElbowRight, JointType_WristRight, window);
    drawBone(pJoints, pJointPoints, JointType_WristRight, JointType_HandRight, window);
    drawBone(pJoints, pJointPoints, JointType_HandRight, JointType_HandTipRight, window);
    drawBone(pJoints, pJointPoints, JointType_WristRight, JointType_ThumbRight, window);

    // Left Arm
    drawBone(pJoints, pJointPoints, JointType_ShoulderLeft, JointType_ElbowLeft, window);
    drawBone(pJoints, pJointPoints, JointType_ElbowLeft, JointType_WristLeft, window);
    drawBone(pJoints, pJointPoints, JointType_WristLeft, JointType_HandLeft, window);
    drawBone(pJoints, pJointPoints, JointType_HandLeft, JointType_HandTipLeft, window);
    drawBone(pJoints, pJointPoints, JointType_WristLeft, JointType_ThumbLeft, window);

    // Right Leg
    drawBone(pJoints, pJointPoints, JointType_HipRight, JointType_KneeRight, window);
    drawBone(pJoints, pJointPoints, JointType_KneeRight, JointType_AnkleRight, window);
    drawBone(pJoints, pJointPoints, JointType_AnkleRight, JointType_FootRight, window);
    // Left Leg
    drawBone(pJoints, pJointPoints, JointType_HipLeft, JointType_KneeLeft, window);
    drawBone(pJoints, pJointPoints, JointType_KneeLeft, JointType_AnkleLeft, window);
    drawBone(pJoints, pJointPoints, JointType_AnkleLeft, JointType_FootLeft, window);

    // Draw the joints
    for (int i = 0; i < JointType_Count; ++i)
    {
        sf::CircleShape circle{};
        circle.setRadius(KinectSettings::g_JointThickness);
        circle.setPosition(pJointPoints[i]);

        if (pJoints[i].TrackingState == TrackingState_Inferred)
        {
            circle.setFillColor(sf::Color::Red);
            window.draw(circle);
        }
        else if (pJoints[i].TrackingState == TrackingState_Tracked)
        {
            circle.setFillColor(sf::Color::Yellow);
            window.draw(circle);
        }
    }
}
void KinectV2Handler::drawBone(const Joint* pJoints, const sf::Vector2f* pJointPoints, JointType joint0, JointType joint1, sf::RenderWindow &window) {
    TrackingState joint0State = pJoints[joint0].TrackingState;
    TrackingState joint1State = pJoints[joint1].TrackingState;

    // If we can't find either of these joints, exit
    if ((joint0State == TrackingState_NotTracked) || (joint1State == TrackingState_NotTracked))
    {
        return;
    }

    // Don't draw if both points are inferred
    if ((joint0State == TrackingState_Inferred) && (joint1State == TrackingState_Inferred))
    {
        return;
    }

    // We assume all drawn bones are inferred unless BOTH joints are tracked
    if ((joint0State == TrackingState_Tracked) && (joint1State == TrackingState_Tracked))
    {
        drawLine(pJointPoints[joint0], pJointPoints[joint1], sf::Color::Green, KinectSettings::g_TrackedBoneThickness, window);
    }
    else
    {
        drawLine(pJointPoints[joint0], pJointPoints[joint1], sf::Color::Green, KinectSettings::g_TrackedBoneThickness, window);
    }
}
void KinectV2Handler::drawLine(sf::Vector2f start, sf::Vector2f end, sf::Color colour, float lineThickness, sf::RenderWindow &window) {
    sfLine line(start, end);
    line.setColor(colour);
    line.setThickness(lineThickness);
    window.draw(line);
}
JointType KinectV2Handler::convertJoint(KVR::KinectJoint kJoint) {
    // Currently, the v2 SDK id's match my jointtype class 1:1
    return static_cast<JointType>(kJoint.joint);
}