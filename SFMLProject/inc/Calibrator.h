#pragma once
#include "stdafx.h"

#include "VRController.h"
#include "GamepadController.h"
#include "GUIHandler.h"

class Calibrator {
    static void Calibrate(
        double deltaT,
        KinectHandlerBase kinect,
        GUIHandler &guiRef) {}
    static void Calibrate(double deltaT, GamepadController gamepad, GUIHandler &guiRef) {}
};