#pragma once
#include "stdafx.h"

#include "VRController.h"
#include "GamepadController.h"
#include "GUIHandler.h"

class Calibrator {
    static void Calibrate(double deltaT, VRcontroller leftController, VRcontroller rightController, GUIHandler &guiRef) {}
    static void Calibrate(double deltaT, GamepadController gamepad, GUIHandler &guiRef) {}
};