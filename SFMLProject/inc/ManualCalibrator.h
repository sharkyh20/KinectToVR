#pragma once
#include "stdafx.h"
#include "Calibrator.h"
#include "KinectSettings.h"
#include "GamepadController.h"
#include "VRController.h"
#include <openvr_math.h>
double deltaScaled(double valuePerSecond, double delta) {
    return valuePerSecond * delta;
}
class ManualCalibrator : Calibrator {
public:
    static void Calibrate(double deltaT, VRcontroller leftController, VRcontroller rightController, GUIHandler &guiRef) {
        if (KinectSettings::adjustingKinectRepresentationPos) {
            adjustHorizontalPosition(deltaT, leftController);
            adjustVerticalPosition(deltaT, rightController);
            confirmPosition(rightController, guiRef);
        }
        else if (KinectSettings::adjustingKinectRepresentationRot) { //TEMP FOR TESTING IMPLMENTATION
            updateKinectQuaternion();
            adjustYawRotation(deltaT, leftController);
            adjustPitchRotation(deltaT, rightController);
            confirmRotation(rightController, guiRef);
        }
    }
    /*  //Disabled for now until I integrate emulation of controllers as VR wands
    static void Calibrate(double deltaT, GamepadController gamepad, GUIHandler &guiRef) {
        updateKinectQuaternion();
        if (SFMLsettings::usingGamepad) { //TODO
            sf::Vector2f axis = gamepad.leftThumbstickValue();
            KinectSettings::kinectRadRotation.v[1] += deltaScaled(3.0, deltaT) * axis.x;

            axis = gamepad.rightThumbstickValue();
            KinectSettings::kinectRadRotation.v[0] += deltaScaled(3.0, deltaT) * axis.y;

            if (gamepad.pressedRightTrigger()) {
                KinectSettings::adjustingKinectRepresentationRot = false;
                guiRef.toggleRotButton();
            }
        }
    }
    */
private:
    static void adjustHorizontalPosition(double deltaT, VRcontroller controller) {
        if (controller.GetTouch(vr::EVRButtonId::k_EButton_SteamVR_Touchpad)) {
            sf::Vector2f axis = controller.GetControllerAxisValue(vr::EVRButtonId::k_EButton_SteamVR_Touchpad);
            KinectSettings::kinectRepPosition.v[0] += deltaScaled(1.0, deltaT) * axis.x;
            KinectSettings::kinectRepPosition.v[2] += deltaScaled(1.0, deltaT) * axis.y;
        }
    }
    static void adjustVerticalPosition(double deltaT, VRcontroller controller) {
        if (controller.GetTouch(vr::EVRButtonId::k_EButton_SteamVR_Touchpad)) {
            sf::Vector2f axis = controller.GetControllerAxisValue(vr::EVRButtonId::k_EButton_SteamVR_Touchpad);
            KinectSettings::kinectRepPosition.v[1] += deltaScaled(1.0, deltaT) * axis.y;
        }
    }
    static void confirmPosition(VRcontroller controller, GUIHandler & guiRef) {
        if (controller.GetTrigger()) {
            KinectSettings::adjustingKinectRepresentationPos = false;
            guiRef.togglePosButton();
            controller.setHapticPulse(.15, 1000, 0);
            KinectSettings::writeKinectSettings(); // This prevents most people's issues with configs not saving due to program crash or steamVR force closing
        }
    }

    static void adjustYawRotation(double deltaT, VRcontroller controller) {
        if (controller.GetTouch(vr::EVRButtonId::k_EButton_SteamVR_Touchpad)) {
            sf::Vector2f axis = controller.GetControllerAxisValue(vr::EVRButtonId::k_EButton_SteamVR_Touchpad);
            KinectSettings::kinectRadRotation.v[1] += deltaScaled(3.0, deltaT) * axis.x;
        }
    }
    static void adjustPitchRotation(double deltaT, VRcontroller controller) {
        if (controller.GetTouch(vr::EVRButtonId::k_EButton_SteamVR_Touchpad)) {
            sf::Vector2f axis = controller.GetControllerAxisValue(vr::EVRButtonId::k_EButton_SteamVR_Touchpad);
            KinectSettings::kinectRadRotation.v[0] += deltaScaled(3.0, deltaT) * axis.y;
        }
    }
    static void confirmRotation(VRcontroller controller, GUIHandler & guiRef) {
        if (controller.GetTrigger()) {
            KinectSettings::adjustingKinectRepresentationRot = false;
            guiRef.toggleRotButton();
            controller.setHapticPulse(.15, 1000, 0);
            KinectSettings::writeKinectSettings(); // This prevents most people's issues with configs not saving due to program crash or steamVR force closing
        }
    }
    static void updateKinectQuaternion() {
        KinectSettings::kinectRepRotation = vrmath::quaternionFromYawPitchRoll(KinectSettings::kinectRadRotation.v[1], KinectSettings::kinectRadRotation.v[0], KinectSettings::kinectRadRotation.v[2]);
    }
};