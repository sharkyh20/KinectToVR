#pragma once
#include "stdafx.h"
#include <algorithm>
#include <thread>

#include <openvr.h>
#include <SFML\System\Vector2.hpp>

class VRcontroller {
public:
    VRcontroller( vr::ETrackedControllerRole role)
        : 
        triggerDeadzone(0.4f),
        triggerOn(false),
        triggerLimit(0.0f),
        controllerType(role)
    {
    }
    ~VRcontroller() {}
    bool Connect(vr::IVRSystem* &m_sys) {
        if (m_sys != nullptr) {
                m_HMDSystem = m_sys;
                controllerID = m_HMDSystem->GetTrackedDeviceIndexForControllerRole(controllerType);
                update(0.0f);
                return true;
        }
        return false;
    }
    bool isConnected() {
        if (m_HMDSystem != nullptr) {
            if (m_HMDSystem->IsTrackedDeviceConnected(controllerID)) {
                return true;
            }
            return false;
        }
        return false;
    }
    void update(float delta) {
        deltaTime = delta;
        if (m_HMDSystem != nullptr)
        {
            lastStateValid = m_HMDSystem->GetControllerStateWithPose(
                vr::ETrackingUniverseOrigin::TrackingUniverseStanding, controllerID, &state_, sizeof(state_), &controllerPose
            );
            if (lastStateValid) {
                prevState_ = state_;
            }
            UpdateTrigger();
            UpdateHapticPulse();
        }
    }

    sf::Vector2f GetControllerAxisValue(vr::EVRButtonId buttonId)
    {
        if (lastStateValid) {
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
        return sf::Vector2f(0.0f, 0.0f);
    }

    //Trigger Input
    void UpdateTrigger()    //TODO test returned values, as they may be acidentally triggering on touch
    {
        if (lastStateValid) {
            triggerPrevOn = triggerOn;
            float value = state_.rAxis[1].x;
            if (triggerOn)
            {
                if (value < triggerDeadzone || value <= 0.0f) {
                    triggerOn = false;
                }
            }
            else
            {
                if (value > triggerDeadzone || value >= 1.0f) {
                    triggerOn = true;
                }
            }
        }
        //triggerLimit = triggerOn ? std::max(triggerLimit, value) : std::min(triggerLimit, value);
        
    }
    bool GetTrigger() {
        if (lastStateValid) return triggerOn; else return false;
    }
    bool GetTriggerDown() {
        if (lastStateValid) return triggerOn && !triggerPrevOn; else return false;
    }
    bool GetTriggerUp() {
        if (lastStateValid) return !triggerOn && triggerPrevOn; else return false;
    }

    //controller input
    bool GetPress(vr::EVRButtonId buttonId) { 
        if (lastStateValid) 
            return (state_.ulButtonPressed& vr::ButtonMaskFromId(buttonId)) != 0; 
        else return false; }
    bool GetPressDown(vr::EVRButtonId buttonId) { 
        if (lastStateValid) 
            return (state_.ulButtonPressed& vr::ButtonMaskFromId(buttonId)) != 0 
            && (prevState_.ulButtonPressed & vr::ButtonMaskFromId(buttonId)) == 0; 
        else return false;
    }
    bool GetPressUp(vr::EVRButtonId buttonId) { 
        if (lastStateValid) 
            return (state_.ulButtonPressed& vr::ButtonMaskFromId(buttonId)) == 0 
            && (prevState_.ulButtonPressed & vr::ButtonMaskFromId(buttonId)) != 0; 
        else return false;
    }

    bool GetTouch(vr::EVRButtonId buttonId) { 
        if (lastStateValid) 
            return (state_.ulButtonTouched& vr::ButtonMaskFromId(buttonId)) != 0; 
        else return false;
    }
    bool GetTouchDown(vr::EVRButtonId buttonId) { 
        if (lastStateValid) 
            return (state_.ulButtonTouched& vr::ButtonMaskFromId(buttonId)) != 0 
            && (prevState_.ulButtonTouched & vr::ButtonMaskFromId(buttonId)) == 0; 
        else return false;
    }
    bool GetTouchUp(vr::EVRButtonId buttonId) { 
        if (lastStateValid) 
            return (state_.ulButtonTouched& vr::ButtonMaskFromId(buttonId)) == 0 
            && (prevState_.ulButtonTouched & vr::ButtonMaskFromId(buttonId)) != 0; 
        else return false;
    }

    /*
    void TriggerHapticPulse(uint64_t duration_micro_sec, vr::EVRButtonId buttonId)
    {
        if (m_HMDSystem != nullptr && lastStateValid)
        {
            uint32_t axisId = (uint32_t)buttonId - (uint32_t)vr::k_EButton_Axis0;
            m_HMDSystem->TriggerHapticPulse(controllerID, 0, duration_micro_sec);   // replace 0 with axisID, removed for testing
            std::cout << "Sent Haptic!\n";  //debug
        }
        
    }
    */

    void UpdateHapticPulse() {
        if (m_HMDSystem != nullptr && lastStateValid)
        {
            if (controllerPulse.active) {
                if (controllerPulse.elapsed < controllerPulse.length)  {
                    controllerPulse.elapsed += deltaTime;
                    m_HMDSystem->TriggerHapticPulse(controllerID, 0, lerp(0, 3999, controllerPulse.strength));    // replace 0 with axisID, removed for testing
                } 
                else
                    controllerPulse.active = false;
            }
        }
    }
    void setHapticPulse(float len, float str, uint32_t id) {
        controllerPulse.length = len;
        controllerPulse.strength = str;
        controllerPulse.axisId = id;
        controllerPulse.active = true;
        controllerPulse.elapsed = 0;
    }
    float lerp(float start, float finish, float alpha) {
        return (1 - alpha) * start + alpha * finish; 
    }

private:
    vr::TrackedDeviceIndex_t controllerID;
    vr::TrackedDevicePose_t controllerPose;
    vr::VRControllerState_t state_;
    vr::VRControllerState_t prevState_;

    struct HapticPulse {
        //length is how long the vibration should go for
        //strength is vibration strength from 0-1
        float elapsed = 0;

        float length = 0;
        float strength = 0;
        uint32_t axisId = 0;
        bool active = false;
    };
    HapticPulse controllerPulse;

    bool triggerOn;
    bool triggerPrevOn;
    float triggerDeadzone;
    float triggerLimit;

    bool lastStateValid;
    float deltaTime;
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
