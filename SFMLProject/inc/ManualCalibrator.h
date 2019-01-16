#pragma once
#include "stdafx.h"
#include "Calibrator.h"
#include "KinectSettings.h"
#include "GamepadController.h"
#include <openvr.h>
#include <openvr_math.h>
double deltaScaled(double valuePerSecond, double delta) {
    return valuePerSecond * delta;
}
class ManualCalibrator : Calibrator {
public:
    static void Calibrate(
        double deltaT,
        KinectHandlerBase& kinect,
        vr::VRActionHandle_t & h_horizontalPos,
        vr::VRActionHandle_t & h_verticalPos,
        vr::VRActionHandle_t & h_confirmPos,
        GUIHandler &guiRef) {
        if (KinectSettings::adjustingKinectRepresentationPos) {
            adjustHorizontalPosition(deltaT, h_horizontalPos);
            adjustVerticalPosition(deltaT, h_verticalPos);
            confirmPosition(h_confirmPos, guiRef);
            guiRef.refreshCalibrationMenuValues();
        }
        else if (KinectSettings::adjustingKinectRepresentationRot) { //TEMP FOR TESTING IMPLMENTATION
            KinectSettings::updateKinectQuaternion();
            adjustYawRotation(deltaT, h_horizontalPos);
            adjustPitchRotation(deltaT, h_verticalPos);
            confirmRotation(h_confirmPos, guiRef);
            guiRef.refreshCalibrationMenuValues();
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
    static void adjustHorizontalPosition(double deltaT, vr::VRActionHandle_t & h_horizontalPos) {
        using namespace VRInput;
        if (moveHorizontallyData.bActive) {
            float x = moveHorizontallyData.x;
            float y = moveHorizontallyData.y;
            KinectSettings::kinectRepPosition.v[0] += deltaScaled(1.0, deltaT) * x;
            KinectSettings::kinectRepPosition.v[2] += deltaScaled(1.0, deltaT) * y;
        }
    }
    static void adjustVerticalPosition(double deltaT, vr::VRActionHandle_t & h_verticalPos) {
        using namespace VRInput;
        if (moveVerticallyData.bActive) {
            float y = moveVerticallyData.y;
            KinectSettings::kinectRepPosition.v[1] += deltaScaled(1.0, deltaT) * y;
        }
    }
    static void confirmPosition(vr::VRActionHandle_t & h_confirmPos, GUIHandler & guiRef) {
        using namespace VRInput;
        if (confirmCalibrationData.bState) {
            KinectSettings::adjustingKinectRepresentationPos = false;
            KinectSettings::sensorConfigChanged = true;
            guiRef.togglePosButton();
            //controller.setHapticPulse(.15, 1000, 0); // Still need to figure out how to send rumbles
            // This prevents most people's issues with configs not saving due to program crash or steamVR force closing
            KinectSettings::writeKinectSettings(); 
        }
    }

    static void adjustYawRotation(double deltaT, vr::VRActionHandle_t & h_horizontalPos) {
        using namespace VRInput;
        if (moveHorizontallyData.bActive) {
            float x = moveHorizontallyData.x;
            KinectSettings::kinectRadRotation.v[1] += deltaScaled(3.0, deltaT) * x;
        }
    }
    static void adjustPitchRotation(double deltaT, vr::VRActionHandle_t & h_verticalPos) {
        using namespace VRInput;
        if (moveVerticallyData.bActive) {
            float y = moveVerticallyData.y;
            KinectSettings::kinectRadRotation.v[0] += deltaScaled(3.0, deltaT) * y;
        }
    }
    static void confirmRotation(vr::VRActionHandle_t & h_confirmRot, GUIHandler & guiRef) {
        using namespace VRInput;
        if (confirmCalibrationData.bState) {
            KinectSettings::adjustingKinectRepresentationRot = false;
            KinectSettings::sensorConfigChanged = true;
            guiRef.toggleRotButton();
            //controller.setHapticPulse(.15, 1000, 0); // Still need to figure out how to send rumbles
            // This prevents most people's issues with configs not saving due to program crash or steamVR force closing
            KinectSettings::writeKinectSettings();
        }
    }
    
};