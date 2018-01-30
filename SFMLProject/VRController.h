#pragma once
#include "stdafx.h"
#include <algorithm>

#include <openvr.h>
#include <SFML\System\Vector2.hpp>

class VRcontroller {
public:
    VRcontroller( vr::ETrackedControllerRole role)
        : 
        triggerDeadzone(0.1f),
        triggerOn(false),
        triggerLimit(0.0f),
        controllerType(role)
    {
    }
    ~VRcontroller() {}
    bool Connect(vr::IVRSystem* &m_sys) {
        if (m_sys->IsInputAvailable()) {
            m_HMDSystem = m_sys;
            controllerID = m_HMDSystem->GetTrackedDeviceIndexForControllerRole(controllerType);
            update();
            return true;
        }
        else {
            return false;
        }
    }
    bool isConnected() {
        if (m_HMDSystem->IsTrackedDeviceConnected(controllerID)) {
            return true;
        }
        return false;
    }
    void update() {
        prevState_ = state_;
        if (m_HMDSystem != nullptr)
        {
            lastStateValid = m_HMDSystem->GetControllerStateWithPose(
                vr::ETrackingUniverseOrigin::TrackingUniverseStanding, controllerID, &state_, sizeof(state_), &controllerPose
            );
            if (prevState_.ulButtonPressed != state_.ulButtonPressed)
            {
                int breakpoint = 0;
            }
            UpdateTrigger();
        }
    }

    sf::Vector2f GetControllerAxisValue(vr::EVRButtonId buttonId)
    {
        sf::Vector2f axis_value;
        uint64_t axisId = (uint64_t)buttonId - (uint64_t)vr::k_EButton_Axis0;
        switch (axisId)
        {
        case 0: axis_value = sf::Vector2f(state_.rAxis[0].x, state_.rAxis[0].y); break;
        case 1: axis_value = sf::Vector2f(state_.rAxis[1].x, state_.rAxis[1].y); break;
        case 2: axis_value = sf::Vector2f(state_.rAxis[2].x, state_.rAxis[2].y); break;
        case 3: axis_value = sf::Vector2f(state_.rAxis[3].x, state_.rAxis[3].y); break;
        case 4: axis_value = sf::Vector2f(state_.rAxis[4].x, state_.rAxis[4].y); break;
        }
        return axis_value;
    }

    //Trigger Input
    void UpdateTrigger()
    {
        triggerPrevOn = triggerOn;
        float value = state_.rAxis[1].x; 
        if (triggerOn)
        {
            if (value < triggerLimit - triggerDeadzone || value <= 0.0f) {
                triggerOn = false;
            }
        }
        else
        {
            if (value > triggerLimit + triggerDeadzone || value >= 1.0f) {
                triggerOn = true;
            }
        }
        triggerLimit = triggerOn ? std::max(triggerLimit, value) : std::min(triggerLimit, value);
    }
    bool GetTrigger() { return triggerOn; }
    bool GetTriggerDown() { return triggerOn && !triggerPrevOn; }
    bool GetTriggerUp() {  return !triggerOn && triggerPrevOn; }

    //controller input
    bool GetPress(vr::EVRButtonId buttonId) {  return (state_.ulButtonPressed& vr::ButtonMaskFromId(buttonId)) != 0; }
    bool GetPressDown(vr::EVRButtonId buttonId) { return (state_.ulButtonPressed& vr::ButtonMaskFromId(buttonId)) != 0 && (prevState_.ulButtonPressed & vr::ButtonMaskFromId(buttonId)) == 0; }
    bool GetPressUp(vr::EVRButtonId buttonId) {  return (state_.ulButtonPressed& vr::ButtonMaskFromId(buttonId)) == 0 && (prevState_.ulButtonPressed & vr::ButtonMaskFromId(buttonId)) != 0; }

    bool GetTouch(vr::EVRButtonId buttonId) {  return (state_.ulButtonTouched& vr::ButtonMaskFromId(buttonId)) != 0; }
    bool GetTouchDown(vr::EVRButtonId buttonId) {  return (state_.ulButtonTouched& vr::ButtonMaskFromId(buttonId)) != 0 && (prevState_.ulButtonTouched & vr::ButtonMaskFromId(buttonId)) == 0; }
    bool GetTouchUp(vr::EVRButtonId buttonId) { return (state_.ulButtonTouched& vr::ButtonMaskFromId(buttonId)) == 0 && (prevState_.ulButtonTouched & vr::ButtonMaskFromId(buttonId)) != 0; }


private:
    vr::TrackedDeviceIndex_t controllerID;
    vr::TrackedDevicePose_t controllerPose;
    vr::VRControllerState_t state_;
    vr::VRControllerState_t prevState_;

    bool triggerOn;
    bool triggerPrevOn;
    float triggerDeadzone;
    float triggerLimit;

    bool lastStateValid;
    vr::IVRSystem* m_HMDSystem;
    vr::ETrackedControllerRole controllerType;
};
/** VR controller button and axis IDs ---pasted from openvr.h--- */
/*
enum EVRButtonId
{
k_EButton_System = 0,
k_EButton_ApplicationMenu = 1,
k_EButton_Grip = 2,
k_EButton_DPad_Left = 3,
k_EButton_DPad_Up = 4,
k_EButton_DPad_Right = 5,
k_EButton_DPad_Down = 6,
k_EButton_A = 7,

k_EButton_ProximitySensor = 31,

k_EButton_Axis0 = 32,
k_EButton_Axis1 = 33,
k_EButton_Axis2 = 34,
k_EButton_Axis3 = 35,
k_EButton_Axis4 = 36,

// aliases for well known controllers
k_EButton_SteamVR_Touchpad = k_EButton_Axis0,
k_EButton_SteamVR_Trigger = k_EButton_Axis1,

k_EButton_Dashboard_Back = k_EButton_Grip,

k_EButton_Max = 64
};
*/
