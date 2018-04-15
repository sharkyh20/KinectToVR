#pragma once
#include "VRHelper.h"
#include "VRController.h"
#include "KinectSettings.h"

class PlayspaceMovementAdjuster {
public:
    PlayspaceMovementAdjuster( vrinputemulator::VRInputEmulator* vrinputEmulator )
		: changedPlayspace(false),
        lastLeftPosition(0,0,0),
        lastRightPosition(0,0,0)
    {
		inputEmulator = vrinputEmulator;
        // FIXME: frame 0 to 1, delta position will be wrong, if they're holding the movement buttons, they'll get zipped into space.
        ensureWorkingCopyReverted();
        vr::VRChaperoneSetup()->GetWorkingStandingZeroPoseToRawTrackingPose(&currentChaperoneMatrix);
        initialPlayspaceMatrix = currentChaperoneMatrix;
		SetUniverseOrigin(currentChaperoneMatrix, sf::Vector3f(0, 0, 0), *inputEmulator, virtualDeviceIndexes);
    }
    ~PlayspaceMovementAdjuster() {

    }

    void update(VRcontroller& leftController, VRcontroller& rightController, std::vector<uint32_t> latestVirtualDeviceIndexes) {
        virtualDeviceIndexes = latestVirtualDeviceIndexes;
        playspaceMovementUpdate(leftController, rightController);
    }
    
    void resetPlayspaceAdjustments() {
		SetUniverseOrigin(initialPlayspaceMatrix, sf::Vector3f(0, 0, 0), *inputEmulator, virtualDeviceIndexes);
    }

private:
    sf::Vector3f lastLeftPosition;
    sf::Vector3f lastRightPosition;
    vr::HmdMatrix34_t initialPlayspaceMatrix;
    vr::HmdMatrix34_t currentChaperoneMatrix;
	vrinputemulator::VRInputEmulator* inputEmulator;
    bool changedPlayspace;
    std::vector<uint32_t> virtualDeviceIndexes;

    bool poseIsUsable(vr::TrackedDevicePose_t pose) {
        return pose.bPoseIsValid && pose.bDeviceIsConnected;
    }
    void playspaceMovementUpdate(VRcontroller& leftController, VRcontroller& rightController) {

        vr::TrackedDevicePose_t leftPose = leftController.GetPose();
        vr::TrackedDevicePose_t rightPose = rightController.GetPose();

        vr::HmdMatrix34_t* leftMat = &(leftPose.mDeviceToAbsoluteTracking);
        vr::HmdMatrix34_t* rightMat = &(rightPose.mDeviceToAbsoluteTracking);

        sf::Vector3f leftPos = sf::Vector3f(leftMat->m[0][3], leftMat->m[1][3], leftMat->m[2][3]);
        sf::Vector3f rightPos = sf::Vector3f(rightMat->m[0][3], rightMat->m[1][3], rightMat->m[2][3]);

        sf::Vector3f positionDelta = sf::Vector3f(0, 0, 0);
        if (KinectSettings::leftHandPlayspaceMovementButton && leftController.GetPress((vr::EVRButtonId)(KinectSettings::leftHandPlayspaceMovementButton - 1)) && poseIsUsable(leftPose)) {
            positionDelta = leftPos - lastLeftPosition;
        }
        if (KinectSettings::rightHandPlayspaceMovementButton && rightController.GetPress((vr::EVRButtonId)(KinectSettings::rightHandPlayspaceMovementButton - 1)) && poseIsUsable(rightPose)) {
            positionDelta = rightPos - lastRightPosition;
        }
        if (poseIsUsable(leftPose)) {
            lastLeftPosition = leftPos - positionDelta; // Take into account the playspace suddenly moving.
        }
        if (poseIsUsable(rightPose)) {
            lastRightPosition = rightPos - positionDelta;
        }
        if (positionDelta.x + positionDelta.y + positionDelta.z != 0) {
            MoveUniverseOrigin(currentChaperoneMatrix, positionDelta, *inputEmulator, virtualDeviceIndexes);
        }
    }
    void ensureWorkingCopyReverted() {
        vr::VRChaperoneSetup()->RevertWorkingCopy();
		// TODO: Probably send out an error if we never get a good calibration state. if someone bumped their lighthouses this will freeze the app..
		while (vr::VRChaperone()->GetCalibrationState() != vr::ChaperoneCalibrationState_OK) {
            vr::VRChaperoneSetup()->RevertWorkingCopy();
        };
    }
};