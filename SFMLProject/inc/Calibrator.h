#pragma once
#include "stdafx.h"

#include "VRController.h"
#include "GamepadController.h"
#include "GUIHandler.h"

class Calibrator {
    static void Calibrate(
        double deltaT,
        KinectHandlerBase kinect,
        vr::VRActionHandle_t &h_horizontalPos,
        vr::VRActionHandle_t &h_verticalPos,
        vr::VRActionHandle_t &h_confirmPos,
        GUIHandler &guiRef) {}
    static void Calibrate(double deltaT, GamepadController gamepad, GUIHandler &guiRef) {}
};