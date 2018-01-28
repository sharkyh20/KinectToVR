#include "stdafx.h"
#include "KinectToVR.h"

namespace KinectSettings {
    bool isKinectDrawn = false;
    bool isSkeletonDrawn = false;

    double trackedPositionOffset[3]{ 0,0,0 };
    bool userChangingZero = false;

    float g_TrackedBoneThickness = 6.0f;
    float g_InferredBoneThickness = 1.5f;
    float g_JointThickness = 4.0f;

    const int kinectHeight = 640;
    const int kinectWidth = 480;
}

namespace SFMLsettings {
    int m_window_width = 1280;
    int m_window_height = 960;
}
# define M_PI           3.14159265358979323846
sf::Vector2f m_points[NUI_SKELETON_POSITION_COUNT];

bool zeroed = false;
vr::HmdVector3_t hmdZero; //TEMP GLOBAL
Vector4 kinectZero; //TEMP GLOBAL
vr::HmdVector3_t m_HMDposition;
vr::HmdQuaternion_t m_HMDquaternion;
double kinectToVRScale = 1;



KinectTrackedDevice::KinectTrackedDevice(
    vrinputemulator::VRInputEmulator& inputEmulator,
    NUI_SKELETON_POSITION_INDEX j0,
    NUI_SKELETON_POSITION_INDEX j1,
    bool isKinect)
:
joint0(j0),
joint1(j1),
hmdRelativePosition(sf::Vector3f(0, 0, 0)),
isKinectDevice(isKinect)
{
    deviceId = initTracker(inputEmulator, true);
}

void drawKinectImageData(KinectHandler& kinect) {
    getKinectData(kinect.kinectImageData.get(), kinect.kinectRGBStream, kinect.kinectSensor);

    glBindTexture(GL_TEXTURE_2D, kinect.kinectTextureId);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, SFMLsettings::m_window_width, SFMLsettings::m_window_height, GL_BGRA_EXT, GL_UNSIGNED_BYTE, (GLvoid*)kinect.kinectImageData.get());
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
void getKinectData(GLubyte* dest, HANDLE& rgbStream, INuiSensor* &sensor) {
    NUI_IMAGE_FRAME imageFrame{};
    NUI_LOCKED_RECT LockedRect{};
    if (acquireKinectFrame(imageFrame, rgbStream, sensor)) {
        return;
    }
    INuiFrameTexture* texture = lockKinectPixelData(imageFrame, LockedRect);
    copyKinectPixelData(LockedRect, dest);
    unlockKinectPixelData(texture);

    releaseKinectFrame(imageFrame, rgbStream, sensor);
}
bool acquireKinectFrame(NUI_IMAGE_FRAME &imageFrame, HANDLE & rgbStream, INuiSensor* &sensor)
{
    return (sensor->NuiImageStreamGetNextFrame(rgbStream, 1, &imageFrame) < 0);
}
INuiFrameTexture* lockKinectPixelData(NUI_IMAGE_FRAME &imageFrame, NUI_LOCKED_RECT &LockedRect)
{
    INuiFrameTexture* texture = imageFrame.pFrameTexture;
    texture->LockRect(0, &LockedRect, NULL, 0);
    return imageFrame.pFrameTexture;
}
void copyKinectPixelData(NUI_LOCKED_RECT &LockedRect, GLubyte* dest)
{
    int bytesInFrameRow = LockedRect.Pitch;
    if (bytesInFrameRow != 0) {
        const BYTE* curr = (const BYTE*)LockedRect.pBits;
        const BYTE* dataEnd = curr + (KinectSettings::kinectWidth*KinectSettings::kinectHeight) * 4;

        while (curr < dataEnd) {
            *dest++ = *curr++;
        }
    }
}
void unlockKinectPixelData(INuiFrameTexture* texture)
{
    texture->UnlockRect(0);
}
void releaseKinectFrame(NUI_IMAGE_FRAME &imageFrame, HANDLE& rgbStream, INuiSensor* &sensor)
{
    sensor->NuiImageStreamReleaseFrame(rgbStream, &imageFrame);
}

void updateTrackersWithSkeletonPosition(vrinputemulator::VRInputEmulator &emulator, std::vector<KinectTrackedDevice> trackers, NUI_SKELETON_FRAME &skeletonFrame) {
    for (int i = 0; i < NUI_SKELETON_COUNT; ++i) {
        NUI_SKELETON_TRACKING_STATE trackingState = skeletonFrame.SkeletonData[i].eTrackingState;

        if (NUI_SKELETON_TRACKED == trackingState)
        {
            for (KinectTrackedDevice device : trackers) {
                if (!device.isKinectDevice)
                    updateKinectTrackedDevice(i, emulator, device, skeletonFrame, hmdZero);
                else
                    updateKinectTracker(emulator, device);
            }
        }
    }
}
void updateKinectTrackedDevice(int i, vrinputemulator::VRInputEmulator &emulator,
    KinectTrackedDevice device, const NUI_SKELETON_FRAME & skel,
    vr::HmdVector3_t zeroPos)
{
    NUI_SKELETON_POSITION_TRACKING_STATE joint0State = skel.SkeletonData[i].eSkeletonPositionTrackingState[device.joint0];
    NUI_SKELETON_POSITION_TRACKING_STATE joint1State = skel.SkeletonData[i].eSkeletonPositionTrackingState[device.joint1];

    // If we can't find either of these joints, exit
    if (joint0State == NUI_SKELETON_POSITION_NOT_TRACKED || joint1State == NUI_SKELETON_POSITION_NOT_TRACKED)
    {
        return;
    }

    // Don't track if both points are inferred
    if (joint0State == NUI_SKELETON_POSITION_INFERRED && joint1State == NUI_SKELETON_POSITION_INFERRED)
    {
        return;
    }
    else
    {
        auto pose = emulator.getVirtualDevicePose(device.deviceId);
        //POSITION
        double kRelativeX = skel.SkeletonData[i].SkeletonPositions[device.joint0].x - kinectZero.x;
        double kRelativeY = skel.SkeletonData[i].SkeletonPositions[device.joint0].y - kinectZero.y;
        double kRelativeZ = skel.SkeletonData[i].SkeletonPositions[device.joint0].z - kinectZero.z;

        double rawPositionX = KinectSettings::trackedPositionOffset[0] + hmdZero.v[0] + kRelativeX;
        double rawPositionY = KinectSettings::trackedPositionOffset[1] + kRelativeY;   // The Y axis is always up, but the other two depend on kinect orientation
        double rawPositionZ = KinectSettings::trackedPositionOffset[2] + hmdZero.v[2] + kRelativeZ;

        pose.vecPosition[0] = kinectToVRScale * rawPositionX;
        pose.vecPosition[1] = kinectToVRScale * rawPositionY;
        pose.vecPosition[2] = kinectToVRScale * rawPositionZ;

        pose.poseIsValid = true;
        pose.result = vr::TrackingResult_Running_OK;
        emulator.setVirtualDevicePose(device.deviceId, pose);
    }
}

void updateKinectTracker(vrinputemulator::VRInputEmulator &emulator, KinectTrackedDevice device)
{
    auto pose = emulator.getVirtualDevicePose(device.deviceId);
    // The Kinect Tracker position must be rotated, as otherwise the tracker is oriented to the wrong direction
    /*
    double kRelativeX =  - kinectZero.x;
    double kRelativeY =  - kinectZero.y;
    double kRelativeZ = - kinectZero.z;
    double rawPositionX = hmdZero.v[0] + kRelativeX;
    double rawPositionZ =  hmdZero.v[2] + kRelativeZ;

    pose.vecPosition[0] = kinectToVRScale * rawPositionX;
    pose.vecPosition[1] = kinectToVRScale * kRelativeY;
    pose.vecPosition[2] = kinectToVRScale * rawPositionZ;
    */

    double kRelativeX = -kinectZero.x;
    double kRelativeY = -kinectZero.y;
    double kRelativeZ = -kinectZero.z;
    double rawPositionX = hmdZero.v[0] + kRelativeX;
    double rawPositionZ = hmdZero.v[2] + kRelativeZ;

    pose.vecPosition[0] = rawPositionX;
    pose.vecPosition[1] = kinectToVRScale * kRelativeY;
    pose.vecPosition[2] = rawPositionZ;

    pose.poseIsValid = true;
    pose.result = vr::TrackingResult_Running_OK;
    emulator.setVirtualDevicePose(device.deviceId, pose);
}
void toEulerAngle(vr::HmdQuaternion_t q, double& roll, double& pitch, double& yaw)
{
    // roll (x-axis rotation)
    double sinr = +2.0 * (q.w * q.x + q.y * q.z);
    double cosr = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    roll = atan2(sinr, cosr);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny = +2.0 * (q.w * q.z + q.x * q.y);
    double cosy = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    yaw = atan2(siny, cosy);
}

void zeroAllTracking(NUI_SKELETON_FRAME& skeletonFrame, vr::IVRSystem* &m_sys) {
    for (int i = 0; i < NUI_SKELETON_COUNT; ++i) {
        NUI_SKELETON_TRACKING_STATE trackingState = skeletonFrame.SkeletonData[i].eTrackingState;

        if (NUI_SKELETON_TRACKED == trackingState)
        {
            if (!zeroed) {
                hmdZero = getHMDPosition(m_sys);
                kinectZero = zeroKinectPosition(skeletonFrame, i);
                setKinectToVRMultiplier(skeletonFrame, i);
                zeroed = true;
                break;
            }
        }
    }
}
vr::HmdVector3_t getHMDPosition(vr::IVRSystem* &m_system) {
    //Zeros the kinect positions to the HMD location for relative position setting
    // Use the head joint for the zero location!

    vr::TrackedDevicePose_t hmdPose;
    vr::TrackedDevicePose_t devicePose[vr::k_unMaxTrackedDeviceCount];
    /*
    vr::EVRInitError vrInitError;
    vr::VR_Init(&vrInitError, vr::EVRApplicationType::VRApplication_Scene);
    
    vr::VRCompositor()->WaitGetPoses(devicePose, vr::k_unMaxTrackedDeviceCount, NULL, 0);
    for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
        if (devicePose[i].bPoseIsValid) {
            if (vr::VRSystem()->GetTrackedDeviceClass(i) == vr::TrackedDeviceClass_HMD) {
                hmdPose = devicePose[i];
                m_HMDposition = GetPosition(hmdPose.mDeviceToAbsoluteTracking);
                m_HMDquaternion = GetRotation(hmdPose.mDeviceToAbsoluteTracking);
                break;
            }
        }
    }
    
    vr::VR_Shutdown();
    */
    m_system->GetDeviceToAbsoluteTrackingPose(vr::ETrackingUniverseOrigin::TrackingUniverseStanding, 0, devicePose, 1);
    if (devicePose[0].bPoseIsValid) {
        if (vr::VRSystem()->GetTrackedDeviceClass(0) == vr::TrackedDeviceClass_HMD) {
            hmdPose = devicePose[0];
            m_HMDposition = GetPosition(hmdPose.mDeviceToAbsoluteTracking);
            m_HMDquaternion = GetRotation(hmdPose.mDeviceToAbsoluteTracking);
        }
    }
    return m_HMDposition;
}
Vector4 zeroKinectPosition(NUI_SKELETON_FRAME &skeletonFrame, int i) {
    return skeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HEAD];
}
void setKinectToVRMultiplier(NUI_SKELETON_FRAME & skel, int i) {
    kinectToVRScale = hmdZero.v[1]
        / (skel.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HEAD].y
            +
            -skel.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_FOOT_LEFT].y);
}

void updateSkeletalData(NUI_SKELETON_FRAME &skeletonFrame, INuiSensor* sensor) {
    if (sensor->NuiSkeletonGetNextFrame(0, &skeletonFrame) >= 0) {
        sensor->NuiTransformSmooth(&skeletonFrame, NULL);   //Smooths jittery tracking
    }
    return;
}
void drawTrackedSkeletons(NUI_SKELETON_FRAME& skeletonFrame, sf::RenderWindow &window) {
    for (int i = 0; i < NUI_SKELETON_POSITION_COUNT; ++i) {
        m_points[i] = sf::Vector2f(0.0f, 0.0f);
    }
    for (int i = 0; i < NUI_SKELETON_COUNT; ++i) {
        NUI_SKELETON_TRACKING_STATE trackingState = skeletonFrame.SkeletonData[i].eTrackingState;

        if (NUI_SKELETON_TRACKED == trackingState)
        {
            if (KinectSettings::isSkeletonDrawn) {
                window.pushGLStates();
                window.resetGLStates();

                DrawSkeleton(skeletonFrame.SkeletonData[i], window);

                window.popGLStates();
            }

        }
        else if (NUI_SKELETON_POSITION_ONLY == trackingState) {
            //ONLY CENTER POINT TO DRAW
            if (KinectSettings::isSkeletonDrawn) {
                sf::CircleShape circle(KinectSettings::g_JointThickness, 30);
                circle.setRadius(KinectSettings::g_JointThickness);
                circle.setPosition(SkeletonToScreen(skeletonFrame.SkeletonData[i].Position, SFMLsettings::m_window_width, SFMLsettings::m_window_height));
                circle.setFillColor(sf::Color::Yellow);

                window.pushGLStates();
                window.resetGLStates();

                window.draw(circle);

                window.popGLStates();
            }
        }
    }


}
void DrawSkeleton(const NUI_SKELETON_DATA & skel, sf::RenderWindow &window) {
    for (int i = 0; i < NUI_SKELETON_POSITION_COUNT; ++i) {
        m_points[i] = SkeletonToScreen(skel.SkeletonPositions[i], SFMLsettings::m_window_width, SFMLsettings::m_window_height);
        std::cerr << "m_points[" << i << "] = " << m_points[i].x << ", " << m_points[i].y << std::endl;
        
        // Same with the other cerr, without this, the skeleton flickers
    }
    // Render Torso
    DrawBone(skel, NUI_SKELETON_POSITION_HEAD, NUI_SKELETON_POSITION_SHOULDER_CENTER, window);
    DrawBone(skel, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_LEFT, window);
    DrawBone(skel, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_RIGHT, window);
    DrawBone(skel, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SPINE, window);
    DrawBone(skel, NUI_SKELETON_POSITION_SPINE, NUI_SKELETON_POSITION_HIP_CENTER, window);
    DrawBone(skel, NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_HIP_LEFT, window);
    DrawBone(skel, NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_HIP_RIGHT, window);

    // Left Arm
    DrawBone(skel, NUI_SKELETON_POSITION_SHOULDER_LEFT, NUI_SKELETON_POSITION_ELBOW_LEFT, window);
    DrawBone(skel, NUI_SKELETON_POSITION_ELBOW_LEFT, NUI_SKELETON_POSITION_WRIST_LEFT, window);
    DrawBone(skel, NUI_SKELETON_POSITION_WRIST_LEFT, NUI_SKELETON_POSITION_HAND_LEFT, window);

    // Right Arm
    DrawBone(skel, NUI_SKELETON_POSITION_SHOULDER_RIGHT, NUI_SKELETON_POSITION_ELBOW_RIGHT, window);
    DrawBone(skel, NUI_SKELETON_POSITION_ELBOW_RIGHT, NUI_SKELETON_POSITION_WRIST_RIGHT, window);
    DrawBone(skel, NUI_SKELETON_POSITION_WRIST_RIGHT, NUI_SKELETON_POSITION_HAND_RIGHT, window);

    // Left Leg
    DrawBone(skel, NUI_SKELETON_POSITION_HIP_LEFT, NUI_SKELETON_POSITION_KNEE_LEFT, window);
    DrawBone(skel, NUI_SKELETON_POSITION_KNEE_LEFT, NUI_SKELETON_POSITION_ANKLE_LEFT, window);
    DrawBone(skel, NUI_SKELETON_POSITION_ANKLE_LEFT, NUI_SKELETON_POSITION_FOOT_LEFT, window);

    // Right Leg
    DrawBone(skel, NUI_SKELETON_POSITION_HIP_RIGHT, NUI_SKELETON_POSITION_KNEE_RIGHT, window);
    DrawBone(skel, NUI_SKELETON_POSITION_KNEE_RIGHT, NUI_SKELETON_POSITION_ANKLE_RIGHT, window);
    DrawBone(skel, NUI_SKELETON_POSITION_ANKLE_RIGHT, NUI_SKELETON_POSITION_FOOT_RIGHT, window);


    // Draw the joints in a different color
    for (int i = 0; i < NUI_SKELETON_POSITION_COUNT; ++i)
    {
        sf::CircleShape circle{};
        circle.setRadius(KinectSettings::g_JointThickness);
        circle.setPosition(m_points[i]);

        if (skel.eSkeletonPositionTrackingState[i] == NUI_SKELETON_POSITION_INFERRED)
        {
            circle.setFillColor(sf::Color::Red);
            window.draw(circle);
        }
        else if (skel.eSkeletonPositionTrackingState[i] == NUI_SKELETON_POSITION_TRACKED)
        {
            circle.setFillColor(sf::Color::Yellow);
            window.draw(circle);
        }
    }

}
sf::Vector2f SkeletonToScreen(Vector4 skeletonPoint, int _width, int _height) {
    LONG x = 0, y = 0;
    USHORT depth = 0;

    // Calculate the skeleton's position on the screen
    // NuiTransformSkeletonToDepthImage returns coordinates in NUI_IMAGE_RESOLUTION_320x240 space
    NuiTransformSkeletonToDepthImage(skeletonPoint, &x, &y, &depth);

    float screenPointX = static_cast<float>(x * _width) / 320;
    float screenPointY = static_cast<float>(y * _height) / 240;
    std::cerr << "x = " << x << " ScreenX = " << screenPointX << " y = " << y << " ScreenY = " << screenPointY << std::endl;
    
    // The skeleton constantly flickers and drops out without the cerr command...
    return sf::Vector2f(screenPointX, screenPointY);
}
void DrawBone(const NUI_SKELETON_DATA & skel, NUI_SKELETON_POSITION_INDEX joint0,
    NUI_SKELETON_POSITION_INDEX joint1, sf::RenderWindow &window)
{
    NUI_SKELETON_POSITION_TRACKING_STATE joint0State = skel.eSkeletonPositionTrackingState[joint0];
    NUI_SKELETON_POSITION_TRACKING_STATE joint1State = skel.eSkeletonPositionTrackingState[joint1];

    // If we can't find either of these joints, exit
    if (joint0State == NUI_SKELETON_POSITION_NOT_TRACKED || joint1State == NUI_SKELETON_POSITION_NOT_TRACKED)
    {
        return;
    }

    // Don't draw if both points are inferred
    if (joint0State == NUI_SKELETON_POSITION_INFERRED && joint1State == NUI_SKELETON_POSITION_INFERRED)
    {
        return;
    }
    // Assume all bones are inferred unless BOTH joints are tracked
    if (joint0State == NUI_SKELETON_POSITION_TRACKED && joint1State == NUI_SKELETON_POSITION_TRACKED)
    {
        DrawLine(m_points[joint0], m_points[joint1], sf::Color::Green, KinectSettings::g_TrackedBoneThickness, window);
    }
    else
    {
        DrawLine(m_points[joint0], m_points[joint1], sf::Color::Red, KinectSettings::g_InferredBoneThickness, window);
    }
}
void DrawLine(sf::Vector2f start, sf::Vector2f end, sf::Color colour, float lineThickness, sf::RenderWindow &window) {
    sfLine line(start, end);
    line.setColor(colour);
    line.setThickness(lineThickness);
    window.draw(line);
}

// Get the quaternion representing the rotation
vr::HmdQuaternion_t GetRotation(vr::HmdMatrix34_t matrix) {
    // Credit to Omnifinity https://github.com/Omnifinity/OpenVR-Tracking-Example/
    vr::HmdQuaternion_t q;

    q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
    q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
    q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);
    return q;
}
// Get the vector representing the position
vr::HmdVector3_t GetPosition(vr::HmdMatrix34_t matrix) {
    // Credit to Omnifinity https://github.com/Omnifinity/OpenVR-Tracking-Example/
    vr::HmdVector3_t vector;

    vector.v[0] = matrix.m[0][3];
    vector.v[1] = matrix.m[1][3];
    vector.v[2] = matrix.m[2][3];

    return vector;
}

//VR Tracking
uint32_t initTracker(vrinputemulator::VRInputEmulator &inputEmulator, bool connected) {
    uint32_t deviceId = inputEmulator.getVirtualDeviceCount();
    inputEmulator.addVirtualDevice(vrinputemulator::VirtualDeviceType::TrackedController, std::to_string(deviceId), false);
    setTrackerDefaultProperties(deviceId);
    inputEmulator.publishVirtualDevice(deviceId);
    //Connect device
    auto pose = inputEmulator.getVirtualDevicePose(deviceId);
    if (pose.deviceIsConnected != connected) {
        pose.deviceIsConnected = connected;
        pose.poseIsValid = connected;
        inputEmulator.setVirtualDevicePose(deviceId, pose);
    }
    return deviceId;
}
void setTrackerDefaultProperties(uint32_t &deviceId) {
    setDeviceProperty(deviceId, 1000, "string", "lighthouse");
    setDeviceProperty(deviceId, 1001, "string", "Vive Controller MV");
    setDeviceProperty(deviceId, 1003, "string", "vr_controller_vive_1_5");
    setDeviceProperty(deviceId, 1004, "bool", "0");
    setDeviceProperty(deviceId, 1005, "string", "HTC");
    setDeviceProperty(deviceId, 1006, "string", "1465809478 htcvrsoftware@firmware-win32 2016-06-13 FPGA 1.6/0/0 VRC 1465809477 Radio 1466630404");
    setDeviceProperty(deviceId, 1007, "string", "product 129 rev 1.5.0 lot 2000/0/0 0");
    setDeviceProperty(deviceId, 1010, "bool", "1");
    setDeviceProperty(deviceId, 1017, "uint64", "2164327680");
    setDeviceProperty(deviceId, 1018, "uint64", "1465809478");
    setDeviceProperty(deviceId, 1029, "int32", "3");
    setDeviceProperty(deviceId, 3001, "uint64", "12884901895");
    setDeviceProperty(deviceId, 3002, "int32", "1");
    setDeviceProperty(deviceId, 3003, "int32", "3");
    setDeviceProperty(deviceId, 3004, "int32", "0");
    setDeviceProperty(deviceId, 3005, "int32", "0");
    setDeviceProperty(deviceId, 3006, "int32", "0");
    setDeviceProperty(deviceId, 3007, "int32", "0");
    setDeviceProperty(deviceId, 5000, "string", "icons");
    setDeviceProperty(deviceId, 5001, "string", "{htc}controller_status_off.png");
    setDeviceProperty(deviceId, 5002, "string", "{htc}controller_status_searching.gif");
    setDeviceProperty(deviceId, 5003, "string", "{htc}controller_status_searching_alert.gif");
    setDeviceProperty(deviceId, 5004, "string", "{htc}controller_status_ready.png");
    setDeviceProperty(deviceId, 5005, "string", "{htc}controller_status_ready_alert.png");
    setDeviceProperty(deviceId, 5006, "string", "{htc}controller_status_error.png");
    setDeviceProperty(deviceId, 5007, "string", "{htc}controller_status_standby.png");
    setDeviceProperty(deviceId, 5008, "string", "{htc}controller_status_ready_low.png");
}
void setDeviceProperty(uint32_t deviceId, int dProp, std::string type, std::string value) {
    vr::ETrackedDeviceProperty deviceProperty = (vr::ETrackedDeviceProperty)dProp;
    vrinputemulator::VRInputEmulator inputEmulator;
    inputEmulator.connect();
    if (std::strcmp(type.c_str(), "int32") == 0) {
        inputEmulator.setVirtualDeviceProperty(deviceId, deviceProperty, (int32_t)std::atoi(value.c_str()));
    }
    else if (std::strcmp(type.c_str(), "uint64") == 0) {
        inputEmulator.setVirtualDeviceProperty(deviceId, deviceProperty, (uint64_t)std::atoll(value.c_str()));
    }
    else if (std::strcmp(type.c_str(), "float") == 0) {
        inputEmulator.setVirtualDeviceProperty(deviceId, deviceProperty, (float)std::atof(value.c_str()));
    }
    else if (std::strcmp(type.c_str(), "bool") == 0) {
        inputEmulator.setVirtualDeviceProperty(deviceId, deviceProperty, std::atoi(value.c_str()) != 0);
    }
    else if (std::strcmp(type.c_str(), "string") == 0) {
        inputEmulator.setVirtualDeviceProperty(deviceId, deviceProperty, value.c_str());
    }
    else {
        throw std::runtime_error("Unknown value type.");
    }
}
void removeAllTrackerProperties(uint32_t &deviceId) {
    removeDeviceProperty(deviceId, 1000, "string", "lighthouse");
    removeDeviceProperty(deviceId, 1001, "string", "Vive Controller MV");
    removeDeviceProperty(deviceId, 1003, "string", "vr_controller_vive_1_5");
    removeDeviceProperty(deviceId, 1004, "bool", "0");
    removeDeviceProperty(deviceId, 1005, "string", "HTC");
    removeDeviceProperty(deviceId, 1006, "string", "1465809478 htcvrsoftware@firmware-win32 2016-06-13 FPGA 1.6/0/0 VRC 1465809477 Radio 1466630404");
    removeDeviceProperty(deviceId, 1007, "string", "product 129 rev 1.5.0 lot 2000/0/0 0");
    removeDeviceProperty(deviceId, 1010, "bool", "1");
    removeDeviceProperty(deviceId, 1017, "uint64", "2164327680");
    removeDeviceProperty(deviceId, 1018, "uint64", "1465809478");
    removeDeviceProperty(deviceId, 1029, "int32", "3");
    removeDeviceProperty(deviceId, 3001, "uint64", "12884901895");
    removeDeviceProperty(deviceId, 3002, "int32", "1");
    removeDeviceProperty(deviceId, 3003, "int32", "3");
    removeDeviceProperty(deviceId, 3004, "int32", "0");
    removeDeviceProperty(deviceId, 3005, "int32", "0");
    removeDeviceProperty(deviceId, 3006, "int32", "0");
    removeDeviceProperty(deviceId, 3007, "int32", "0");
    removeDeviceProperty(deviceId, 5000, "string", "icons");
    removeDeviceProperty(deviceId, 5001, "string", "{htc}controller_status_off.png");
    removeDeviceProperty(deviceId, 5002, "string", "{htc}controller_status_searching.gif");
    removeDeviceProperty(deviceId, 5003, "string", "{htc}controller_status_searching_alert.gif");
    removeDeviceProperty(deviceId, 5004, "string", "{htc}controller_status_ready.png");
    removeDeviceProperty(deviceId, 5005, "string", "{htc}controller_status_ready_alert.png");
    removeDeviceProperty(deviceId, 5006, "string", "{htc}controller_status_error.png");
    removeDeviceProperty(deviceId, 5007, "string", "{htc}controller_status_standby.png");
    removeDeviceProperty(deviceId, 5008, "string", "{htc}controller_status_ready_low.png");
}
void removeDeviceProperty(uint32_t deviceId, int dProp, std::string type, std::string value) {
    vr::ETrackedDeviceProperty deviceProperty = (vr::ETrackedDeviceProperty)dProp;
    vrinputemulator::VRInputEmulator inputEmulator;
    inputEmulator.connect();
    if (std::strcmp(type.c_str(), "int32") == 0) {
        inputEmulator.removeVirtualDeviceProperty(deviceId, deviceProperty);
    }
    else if (std::strcmp(type.c_str(), "uint64") == 0) {
        inputEmulator.removeVirtualDeviceProperty(deviceId, deviceProperty);
    }
    else if (std::strcmp(type.c_str(), "float") == 0) {
        inputEmulator.removeVirtualDeviceProperty(deviceId, deviceProperty);
    }
    else if (std::strcmp(type.c_str(), "bool") == 0) {
        inputEmulator.removeVirtualDeviceProperty(deviceId, deviceProperty);
    }
    else if (std::strcmp(type.c_str(), "string") == 0) {
        inputEmulator.removeVirtualDeviceProperty(deviceId, deviceProperty);
    }
    else {
        throw std::runtime_error("Unknown value type.");
    }
}
void destroyTrackers(vrinputemulator::VRInputEmulator& inputEmulator, std::vector<KinectTrackedDevice> trackers) {
    for (auto device : trackers) {
        auto pose = inputEmulator.getVirtualDevicePose(device.deviceId);
        if (pose.deviceIsConnected) {
            pose.deviceIsConnected = false;
            pose.poseIsValid = false;
            inputEmulator.setVirtualDevicePose(device.deviceId, pose);
        }
    }
}
void setKinectTrackerProperties(uint32_t deviceId) {
    setDeviceProperty(deviceId, 1003, "string", "arrow");
    setDeviceProperty(deviceId, 1029, "int32", "4");
}


void processKeyEvents(sf::Event event) {
    switch (event.key.code) {
    case sf::Keyboard::A:
        toggle(KinectSettings::isKinectDrawn);
        break;
    case sf::Keyboard::S:
        toggle(KinectSettings::isSkeletonDrawn);
        break;
    case sf::Keyboard::Q:
        KinectSettings::userChangingZero = true;
        break;
    default:
        break;
    }
}
void toggle(bool &b) {
    b = !b;
}

void initOpenGL(GLuint &textureId, GLubyte* data) {
    // Initialize textures
    glGenTextures(1, &textureId);
    glBindTexture(GL_TEXTURE_2D, textureId);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, KinectSettings::kinectWidth, KinectSettings::kinectHeight,
        0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, (GLvoid*)data);
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

