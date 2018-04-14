#pragma once
#include "VRHelper.h"
#include "VRController.h"
#include "KinectSettings.h"

class PlayspaceMovementAdjuster {
public:
    PlayspaceMovementAdjuster() 
        : changedPlayspace(false),
        lastLeftPosition(0,0,0),
        lastRightPosition(0,0,0)
    {
        // FIXME: frame 0 to 1, delta position will be wrong, if they're holding the movement buttons, they'll get zipped into space.
        ensureWorkingCopyReverted();
        vr::VRChaperoneSetup()->GetWorkingStandingZeroPoseToRawTrackingPose(&currentChaperoneMatrix);
        initialPlayspaceMatrix = currentChaperoneMatrix;
    }
    ~PlayspaceMovementAdjuster() {

    }

    void update(VRcontroller& leftController, VRcontroller& rightController) {
        ensureWorkingCopyReverted();
        changedPlayspace = changedPlayspace || playspaceMovementUpdate( leftController, rightController);
        if (changedPlayspace) {
            if (vr::VRChaperone()->GetCalibrationState() == vr::ChaperoneCalibrationState_OK) {
                //vr::VRChaperoneSetup()->SetWorkingSeatedZeroPoseToRawTrackingPose(&currentChaperoneMatrix);
                vr::VRChaperoneSetup()->SetWorkingStandingZeroPoseToRawTrackingPose(&currentChaperoneMatrix);
                while (!vr::VRChaperoneSetup()->CommitWorkingCopy(vr::EChaperoneConfigFile_Temp)) {}
                changedPlayspace = false;
            }
        }
    }
    
    void resetPlayspaceAdjustments() {
        // Reloads the chaperone data from before adjustments were made
        // Do NOT write to the actual Chaperone cfg, or original playspace is overwritten
        ensureWorkingCopyReverted();
        vr::VRChaperoneSetup()->SetWorkingStandingZeroPoseToRawTrackingPose(&initialPlayspaceMatrix);
        vr::VRChaperoneSetup()->CommitWorkingCopy(vr::EChaperoneConfigFile_Temp);
        
        vr::VRChaperone()->ReloadInfo(); 
    }

private:
    sf::Vector3f lastLeftPosition;
    sf::Vector3f lastRightPosition;
    vr::HmdMatrix34_t initialPlayspaceMatrix;
    vr::HmdMatrix34_t currentChaperoneMatrix;
    bool changedPlayspace;

    bool poseIsUsable(vr::TrackedDevicePose_t pose) {
        return pose.bPoseIsValid && pose.bDeviceIsConnected;
    }
    bool playspaceMovementUpdate(VRcontroller& leftController, VRcontroller& rightController) {

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
            return MoveUniverseOrigin(currentChaperoneMatrix, positionDelta);
        }
        return false;
    }
    void ensureWorkingCopyReverted() {
        vr::VRChaperoneSetup()->RevertWorkingCopy(); while (vr::VRChaperone()->GetCalibrationState() != vr::ChaperoneCalibrationState_OK) {
            vr::VRChaperoneSetup()->RevertWorkingCopy();
        };
    }
};