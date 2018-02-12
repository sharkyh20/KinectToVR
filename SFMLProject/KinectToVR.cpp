#include "stdafx.h"
#include "KinectToVR.h"

#include "KinectSettings.h"
/*
void updateKinectTracker(vrinputemulator::VRInputEmulator &emulator, KinectTrackedDevice device)
{
    auto pose = emulator.getVirtualDevicePose(device.deviceId);
    // The Kinect Tracker position must be rotated, as otherwise the tracker is oriented to the wrong direction
    
    double kRelativeX =  - kinectZero.x;
    double kRelativeY =  - kinectZero.y;
    double kRelativeZ = - kinectZero.z;
    double rawPositionX = hmdZero.v[0] + kRelativeX;
    double rawPositionZ =  hmdZero.v[2] + kRelativeZ;

    pose.vecPosition[0] = kinectToVRScale * rawPositionX;
    pose.vecPosition[1] = kinectToVRScale * kRelativeY;
    pose.vecPosition[2] = kinectToVRScale * rawPositionZ;
    

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
*/
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
