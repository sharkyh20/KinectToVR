#pragma once

#include <iostream>
#include <iomanip>
#include <chrono>
#include <cstring>
#include <thread>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/transform2.hpp>
#include <glm/mat4x4.hpp>
#include <glm/detail/type_vec3.hpp>
#include <glm/detail/type_vec4.hpp>
#include <glm/detail/type_vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp> 
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <Eigen/Dense>
#include <PSMoveClient_CAPI.h>
#include <ClientConstants.h>
#include <SharedConstants.h>
#include <openvr.h>

#include "DeviceHandler.h"
#include "TrackingPoolManager.h"
#include "TrackedDeviceInputData.h"

#define M_PI_2 1.57079632679

int r = 0;
int g = 0;
int b = 0;

class PSMoveHandler : public DeviceHandler{
    // Heavily based off of the example template program
    // test_console_CAPI.cpp in the MoveService project
public:
    PSMoveHandler() 
        : m_started(false),
        m_keepRunning(true) {
    }
    ~PSMoveHandler() {}

    std::vector<std::string> connectionMessages{
        "Connected!",
        "Failed to startup the PSMoveClient. Check if PSMoveService is running...",
    };

    int initialise() {
        try {
            if (startup()) {
                active = true;
                return 0;
            }
            else {
                LOG(INFO) << "PSMoveService is not running.";
            }
        }
        catch (std::exception& e) {
            LOG(ERROR) << e.what();
        }
        return 1;
    }
    void identify(int globalId, bool on) {
        int controllerId = 0;
        for (MoveWrapper_PSM & t : v_controllers) {
            if (t.id.globalID == globalId)
                controllerId = t.id.internalID;
        }
        flashControllerBulb(controllerId, on);
        float rumbleIntensity = 0;
        if (on)
            rumbleIntensity = .9f;

        PSM_SetControllerRumble(controllerId, PSMControllerRumbleChannel_All, rumbleIntensity);
        LOG(INFO) << "Set rumble identify to " << on << "for controller " << controllerId;
    }
    static void flashRainbow(int id) {
        int red, green, blue;
        for (int color = 0; color < 6; color++) {
            switch (color) {

            case 0:
                red = 255; green = 0; blue = 0;
                break;

            case 1:
                red = 255; green = 255; blue = 0;
                break;

            case 2:
                red = 0; green = 255; blue = 0;
                break;

            case 3:
                red = 0; green = 255; blue = 255;
                break;

            case 4:
                red = 0; green = 0; blue = 255;
                break;

            case 5:
                red = 255; green = 0; blue = 255;
                break;

            }
            while (r != red || g != green || b != blue) {
                if (r < red) r += 1;
                if (r > red) r -= 1;

                if (g < green) g += 1;
                if (g > green) g -= 1;

                if (b < blue) b += 1;
                if (b > blue) b -= 1;

                PSM_SetControllerLEDOverrideColor(id, r, g, b);
                std::this_thread::sleep_for(std::chrono::microseconds(1));
            }
        }
        PSM_SetControllerLEDOverrideColor(id, 0, 0, 0);
    }
    void flashControllerBulb(int controllerId, bool on) {
        char r = 255;
        char g = 0;
        char b = 0;

        r = (r) % 255;
        g = (g + 0) % 255;
        b = (b + 0) % 255;
        
        if (!on) {
            r = 0;
        }

        PSM_SetControllerLEDOverrideColor(controllerId, r, g, b);
    }
    std::string PSMResultToString(PSMResult result) {
        switch (result) {
        case PSMResult_Error: 
            return "General Error Result";
        case PSMResult_Success:	
            return "General Success Result";
        case PSMResult_Timeout:	
            return "Requested Timed Out";
        case PSMResult_RequestSent:	
            return "Request Successfully Sent";
        case PSMResult_Canceled:	
            return "Request Canceled ";
        case PSMResult_NoData:	
            return " Request Returned No Data";
        default: return "PSMRESULTTOSTRING INVALID!!!!";
        }
    }
    // Attempt to start and run the client
    int run() {
        update();

        return m_keepRunning;
    }
    void shutdown()
    {
        if (!PSM_GetIsConnected()) {
            LOG(ERROR) << "Attempted PSM shutdown on disconnected PSMoveService!";
            return;
        }
        if (controllerList.count > 0)
        {
            for (int i = 0; i < controllerList.count; ++i) {
                PSM_StopControllerDataStream(controllerList.controller_id[i], PSM_DEFAULT_TIMEOUT);
                PSM_FreeControllerListener(controllerList.controller_id[i]);
            }
        }
        // No tracker data streams started
        // No HMD data streams started

        active = false;

        auto result = PSM_Shutdown();

        //typedef enum
        //{
        //   PSMResult_Error = -1, 	///< General Error Result
        //    PSMResult_Success = 0,	///< General Success Result
        //    PSMResult_Timeout = 1,	///< Requested Timed Out
        //    PSMResult_RequestSent = 2,	///< Request Successfully Sent
        //   PSMResult_Canceled = 3,	///< Request Canceled 
        //    PSMResult_NoData = 4,	///< Request Returned No Data
        //} PSMResult;
        LOG(INFO) << "PSMoveService attempted shutdown with PSMResult: " << PSMResultToString(result);
    }


    int controllersConnected() {
        return v_controllers.size();
    }

    vr::HmdQuaternion_t getMoveOrientation(int localControllerId) {
        if (localControllerId < v_controllers.size()) {
            auto controllerState = v_controllers[localControllerId].controller->ControllerState.PSMoveState;
            if (controllerState.bIsOrientationValid) {
                vr::HmdQuaternion_t q;
                PSMQuatf PSq = controllerState.Pose.Orientation;

                q.w = PSq.w;
                q.x = PSq.x;
                q.y = PSq.y;
                q.z = PSq.z;

                return q;
            }
        }
        return { 1,0,0,0 }; // Identity quaternion, means something is afoot
    }
    vr::HmdVector3d_t getMovePosition(int localControllerId) {
        if (localControllerId < v_controllers.size()) {
            auto controllerState = v_controllers[localControllerId].controller->ControllerState.PSMoveState;
            if (controllerState.bIsPositionValid) {
                vr::HmdVector3d_t pos;
                PSMVector3f pspos = controllerState.Pose.Position;

                pos.v[0] = pspos.x;
                pos.v[1] = pspos.y;
                pos.v[2] = pspos.z;

                return pos;
            }
        }
    return { 0,0,0 };
    }
    static PSMQuatf openvrMatrixExtractPSMQuatf(const vr::HmdMatrix34_t &openVRTransform)
    {
        PSMQuatf q;

        const float(&a)[3][4] = openVRTransform.m;
        const float trace = a[0][0] + a[1][1] + a[2][2];

        if (trace > 0)
        {
            const float s = 0.5f / sqrtf(trace + 1.0f);

            q.w = 0.25f / s;
            q.x = (a[2][1] - a[1][2]) * s;
            q.y = (a[0][2] - a[2][0]) * s;
            q.z = (a[1][0] - a[0][1]) * s;
        }
        else
        {
            if (a[0][0] > a[1][1] && a[0][0] > a[2][2])
            {
                const float s = 2.0f * sqrtf(1.0f + a[0][0] - a[1][1] - a[2][2]);

                q.w = (a[2][1] - a[1][2]) / s;
                q.x = 0.25f * s;
                q.y = (a[0][1] + a[1][0]) / s;
                q.z = (a[0][2] + a[2][0]) / s;
            }
            else if (a[1][1] > a[2][2])
            {
                const float s = 2.0f * sqrtf(1.0f + a[1][1] - a[0][0] - a[2][2]);

                q.w = (a[0][2] - a[2][0]) / s;
                q.x = (a[0][1] + a[1][0]) / s;
                q.y = 0.25f * s;
                q.z = (a[1][2] + a[2][1]) / s;
            }
            else
            {
                const float s = 2.0f * sqrtf(1.0f + a[2][2] - a[0][0] - a[1][1]);

                q.w = (a[1][0] - a[0][1]) / s;
                q.x = (a[0][2] + a[2][0]) / s;
                q.y = (a[1][2] + a[2][1]) / s;
                q.z = 0.25f * s;
            }
        }

        q = PSM_QuatfNormalizeWithDefault(&q, k_psm_quaternion_identity);

        return q;
    }

    static PSMVector3f openvrMatrixExtractPSMVector3f(const vr::HmdMatrix34_t &openVRTransform)
    {
        const float(&a)[3][4] = openVRTransform.m;
        PSMVector3f pos = { a[0][3], a[1][3], a[2][3] };

        return pos;
    }

    static PSMPosef openvrMatrixExtractPSMPosef(const vr::HmdMatrix34_t &openVRTransform)
    {
        PSMPosef pose;
        pose.Orientation = openvrMatrixExtractPSMQuatf(openVRTransform);
        pose.Position = openvrMatrixExtractPSMVector3f(openVRTransform);

        return pose;
    }
    PSMQuatf ExtractHMDYawQuaternion(const PSMQuatf &q)
    {
        // Convert the quaternion to a basis matrix
        const PSMMatrix3f hmd_orientation = PSM_Matrix3fCreateFromQuatf(&q);

        // Extract the forward (z-axis) vector from the basis
        const PSMVector3f forward = PSM_Matrix3fBasisZ(&hmd_orientation);
        PSMVector3f forward2d = { forward.x, 0.f, forward.z };
        forward2d = PSM_Vector3fNormalizeWithDefault(&forward2d, k_psm_float_vector3_k);

        // Compute the yaw angle (amount the z-axis has been rotated to it's current facing)
        const float cos_yaw = PSM_Vector3fDot(&forward, k_psm_float_vector3_k);
        float half_yaw = acosf(fminf(fmaxf(cos_yaw, -1.f), 1.f)) / 2.f;

        // Flip the sign of the yaw angle depending on if forward2d is to the left or right of global forward
        PSMVector3f yaw_axis = PSM_Vector3fCross(k_psm_float_vector3_k, &forward2d);
        if (PSM_Vector3fDot(&yaw_axis, k_psm_float_vector3_j) < 0)
        {
            half_yaw = -half_yaw;
        }

        // Convert this yaw rotation back into a quaternion
        PSMQuatf yaw_quaternion =
            PSM_QuatfCreate(
                cosf(half_yaw), // w = cos(theta/2)
                0.f, sinf(half_yaw), 0.f); // (x, y, z) = sin(theta/2)*axis, where axis = (0, 1, 0)

        return yaw_quaternion;
    }
    PSMQuatf ExtractPSMoveYawQuaternion(const PSMQuatf &q)
    {
        // Convert the quaternion to a basis matrix
        const PSMMatrix3f psmove_basis = PSM_Matrix3fCreateFromQuatf(&q);

        // Extract the forward (negative z-axis) vector from the basis
        const PSMVector3f global_forward = { 0.f, 0.f, -1.f };
        const PSMVector3f &forward = PSM_Matrix3fBasisY(&psmove_basis);
        PSMVector3f forward2d = { forward.x, 0.f, forward.z };
        forward2d = PSM_Vector3fNormalizeWithDefault(&forward2d, &global_forward);

        // Compute the yaw angle (amount the z-axis has been rotated to it's current facing)
        const float cos_yaw = PSM_Vector3fDot(&forward, &global_forward);
        float yaw = acosf(fminf(fmaxf(cos_yaw, -1.f), 1.f));

        // Flip the sign of the yaw angle depending on if forward2d is to the left or right of global forward
        const PSMVector3f &global_up = *k_psm_float_vector3_j;
        PSMVector3f yaw_axis = PSM_Vector3fCross(&global_forward, &forward2d);
        if (PSM_Vector3fDot(&yaw_axis, &global_up) < 0)
        {
            yaw = -yaw;
        }

        // Convert this yaw rotation back into a quaternion
        PSMVector3f eulerPitch = { (float)1.57079632679489661923, 0.f, 0.f }; // pitch 90 up first
        PSMVector3f eulerYaw = { 0, yaw, 0 };
        PSMQuatf quatPitch = PSM_QuatfCreateFromAngles(&eulerPitch);
        PSMQuatf quatYaw = PSM_QuatfCreateFromAngles(&eulerYaw);
        PSMQuatf yaw_quaternion =
            PSM_QuatfConcat(
                &quatPitch, // pitch 90 up first
                &quatYaw); // Then apply the yaw

        return yaw_quaternion;
    }
    
    void alignPSMoveAndHMDTrackingSpace(PSMController * controller) {
        if (m_bDisableHMDAlignmentGesture)
        {
            return;
        }

        vr::TrackedDeviceIndex_t hmd_device_index = 0;

        PSMPosef hmd_pose_meters;
        hmd_pose_meters = openvrMatrixExtractPSMPosef(KinectSettings::hmdAbsoluteTracking);

        // Make the HMD orientation only contain a yaw
        hmd_pose_meters.Orientation = ExtractHMDYawQuaternion(hmd_pose_meters.Orientation);

        auto m_PSMControllerType = controller->ControllerType;
        // We have the transform of the HMD in world space. 
        // However the HMD and the controller aren't quite aligned depending on the controller type:
        PSMQuatf controllerOrientationInHmdSpaceQuat = *k_psm_quaternion_identity;
        PSMVector3f controllerLocalOffsetFromHmdPosition = *k_psm_float_vector3_zero;
        if (m_PSMControllerType == PSMControllerType::PSMController_Move)
        {
            // Rotation) The controller's local -Z axis (from the center to the glowing ball) is currently pointed 
            //    in the direction of the HMD's local +Y axis, 
            // Translation) The controller's position is a few inches ahead of the HMD's on the HMD's local -Z axis. 
            PSMVector3f eulerPitch = { (float)M_PI_2, 0.0f, 0.0f };
            controllerOrientationInHmdSpaceQuat = PSM_QuatfCreateFromAngles(&eulerPitch);
            controllerLocalOffsetFromHmdPosition = { 0.0f, 0.0f, -1.0f * m_fControllerMetersInFrontOfHmdAtCalibration };
        }
        else if (m_PSMControllerType == PSMControllerType::PSMController_DualShock4 ||
            m_PSMControllerType == PSMControllerType::PSMController_Virtual)
        {
            // Translation) The controller's position is a few inches ahead of the HMD's on the HMD's local -Z axis. 
            controllerLocalOffsetFromHmdPosition = { 0.0f, 0.0f, -1.0f * m_fControllerMetersInFrontOfHmdAtCalibration };
        }

        // Transform the HMD's world space transform to where we expect the controller's world space transform to be.
        PSMPosef controllerPoseRelativeToHMD =
            PSM_PosefCreate(&controllerLocalOffsetFromHmdPosition, &controllerOrientationInHmdSpaceQuat);


        // Compute the expected controller pose in HMD tracking space (i.e. "World Space")
        PSMPosef controller_world_space_pose = PSM_PosefConcat(&controllerPoseRelativeToHMD, &hmd_pose_meters);

        /*
        We now have the transform of the controller in world space -- controller_world_space_pose
        We also have the transform of the controller in driver space -- psmove_pose_meters
        We need the transform that goes from driver space to world space -- driver_pose_to_world_pose
        psmove_pose_meters * driver_pose_to_world_pose = controller_world_space_pose
        psmove_pose_meters.inverse() * psmove_pose_meters * driver_pose_to_world_pose = psmove_pose_meters.inverse() * controller_world_space_pose
        driver_pose_to_world_pose = psmove_pose_meters.inverse() * controller_world_space_pose
        */

        // Get the current pose from the controller view instead of using the driver's cached
        // value because the user may have triggered a pose reset, in which case the driver's
        // cached pose might not yet be up to date by the time this callback is triggered.
        PSMPosef controller_pose_meters = *k_psm_pose_identity;
        PSM_GetControllerPose(controller->ControllerID, &controller_pose_meters);

        // PSMove Position is in cm, but OpenVR stores position in meters
        controller_pose_meters.Position = PSM_Vector3fScale(&controller_pose_meters.Position, k_fScalePSMoveAPIToMeters);

        if (m_PSMControllerType == PSMControllerType::PSMController_Move)
        {
            if (m_bUseControllerOrientationInHMDAlignment)
            {
                // Extract only the yaw from the controller orientation (assume it's mostly held upright)
                controller_pose_meters.Orientation = ExtractPSMoveYawQuaternion(controller_pose_meters.Orientation);
            }
            else
            {
                const PSMVector3f eulerPitch = { (float)M_PI_2, 0.0f, 0.0f };

                controller_pose_meters.Orientation = PSM_QuatfCreateFromAngles(&eulerPitch);
            }
        }
        else if (m_PSMControllerType == PSMControllerType::PSMController_DualShock4 ||
            m_PSMControllerType == PSMControllerType::PSMController_Virtual)
        {
            controller_pose_meters.Orientation = *k_psm_quaternion_identity;
        }

        PSMPosef controller_pose_inv = PSM_PosefInverse(&controller_pose_meters);

        PSMPosef driver_pose_to_world_pose = PSM_PosefConcat(&controller_pose_inv, &controller_world_space_pose);

        PSMPosef test_composed_controller_world_space = PSM_PosefConcat(&controller_pose_meters, &driver_pose_to_world_pose);

        // TODO, ACTUALLY MAKE THIS UPDATE THE CONTROLLERS WITH THEIR NEW POSITION
        // Can't do driver stuff, because that would offset everything.
        //v_controllers[0].controller->ControllerState.PSMoveState.Pose = driver_pose_to_world_pose;


        // Update the tracking space with pose
        // from SteamVRBridge server_driver.cpp
        // SetHMDTrackingSpace()
        {
            //Logger::Info("Begin CServerDriver_PSMoveService::SetHMDTrackingSpace()\n");

            m_hasCalibratedWorldFromDriverPose = true;
            //driver_pose_to_world_pose.Position.y -= 1.73f; //Input Emulator's y origin is actually the users set height
            m_worldFromDriverPose = driver_pose_to_world_pose;

            saveTrackingCalibration();
        }
    }

    vr::HmdVector3d_t getPSMovePosition(int controllerId) {
        const PSMPSMove &view = v_controllers[controllerId].controller->ControllerState.PSMoveState;
        // Set position
        vr::HmdVector3d_t raw_position{};
        
        {
            const PSMVector3f &position = view.Pose.Position;

            raw_position.v[0] = position.x * k_fScalePSMoveAPIToMeters;
            raw_position.v[1] = position.y * k_fScalePSMoveAPIToMeters;
            raw_position.v[2] = position.z * k_fScalePSMoveAPIToMeters;
        }
        return raw_position;
    }
    vr::HmdQuaternion_t getPSMoveRotation(int controllerId) {
        const PSMPSMove &view = v_controllers[controllerId].controller->ControllerState.PSMoveState;

        vr::HmdQuaternion_t qRotation{};

        // Set rotational coordinates
        bool m_fVirtuallyRotateController = false;
        {
            const PSMQuatf &orientation = view.Pose.Orientation;

            qRotation.w = m_fVirtuallyRotateController ? -orientation.w : orientation.w;
            qRotation.x = orientation.x;
            qRotation.y = orientation.y;
            qRotation.z = m_fVirtuallyRotateController ? -orientation.z : orientation.z;
        }
        return qRotation;
    }
    vr::DriverPose_t getPSEyeDriverPose(int psEyeId) {
        // Taken from driver_psmoveservice.cpp line 3313, credit to HipsterSloth
        const auto &view = v_eyeTrackers[psEyeId].trackerInfo;


        vr::DriverPose_t m_Pose = defaultReadyDriverPose();

        m_Pose.result = vr::TrackingResult_Running_OK;

        m_Pose.deviceIsConnected = true;

        // These should always be false from any modern driver.  These are for Oculus DK1-like
        // rotation-only tracking.  Support for that has likely rotted in vrserver.
        m_Pose.willDriftInYaw = false;
        m_Pose.shouldApplyHeadModel = false;

        // No prediction since that's already handled in the psmove service
        m_Pose.poseTimeOffset = 0.f;

        // Use HMD calibrated space if available, otherwise it's the default vector/quaternion
        m_Pose.qDriverFromHeadRotation.w = 1.f;
        m_Pose.qDriverFromHeadRotation.x = 0.0f;
        m_Pose.qDriverFromHeadRotation.y = 0.0f;
        m_Pose.qDriverFromHeadRotation.z = 0.0f;

        m_Pose.vecDriverFromHeadTranslation[0] = 0.f;
        m_Pose.vecDriverFromHeadTranslation[1] = 0.f;
        m_Pose.vecDriverFromHeadTranslation[2] = 0.f;

        ///*
        m_Pose.qWorldFromDriverRotation.w = m_worldFromDriverPose.Orientation.w;
        m_Pose.qWorldFromDriverRotation.x = m_worldFromDriverPose.Orientation.x;
        m_Pose.qWorldFromDriverRotation.y = m_worldFromDriverPose.Orientation.y;
        m_Pose.qWorldFromDriverRotation.z = m_worldFromDriverPose.Orientation.z;
        m_Pose.vecWorldFromDriverTranslation[0] = m_worldFromDriverPose.Position.x;
        m_Pose.vecWorldFromDriverTranslation[1] = m_worldFromDriverPose.Position.y;
        m_Pose.vecWorldFromDriverTranslation[2] = m_worldFromDriverPose.Position.z;
        //LOG(INFO) << "worldFromDriver: PS : " << m_Pose.vecWorldFromDriverTranslation[0] << ", " << m_Pose.vecWorldFromDriverTranslation[1] << ", " << m_Pose.vecWorldFromDriverTranslation[2];


        m_Pose.vecAngularVelocity[0] = 0;
        m_Pose.vecAngularVelocity[1] = 0;
        m_Pose.vecAngularVelocity[2] = 0;

        m_Pose.vecVelocity[0] = 0;
        m_Pose.vecVelocity[1] = 0;
        m_Pose.vecVelocity[2] = 0;

        // Set position

        {
            const PSMVector3f &position = view.tracker_pose.Position;

            m_Pose.vecPosition[0] = position.x * k_fScalePSMoveAPIToMeters;
            m_Pose.vecPosition[1] = position.y * k_fScalePSMoveAPIToMeters;
            m_Pose.vecPosition[2] = position.z * k_fScalePSMoveAPIToMeters;
        }

        // Set rotational coordinates
        bool m_fVirtuallyRotateController = false;
        {
            const PSMQuatf &orientation = view.tracker_pose.Orientation;

            m_Pose.qRotation.w = m_fVirtuallyRotateController ? -orientation.w : orientation.w;
            m_Pose.qRotation.x = orientation.x;
            m_Pose.qRotation.y = orientation.y;
            m_Pose.qRotation.z = m_fVirtuallyRotateController ? -orientation.z : orientation.z;
        }

        m_Pose.poseIsValid = true;

        return m_Pose;
    }
    vr::DriverPose_t getDriverPose(int controllerId) {
        auto controller = v_controllers[controllerId].controller;
        switch (controller->ControllerType) {
        case PSMController_Virtual:
        {
            const PSMVirtualController &view = v_controllers[controllerId].controller->ControllerState.VirtualController;
            auto const & pos = view.Pose.Position;
            auto const & rot = view.Pose.Orientation;
            auto const & physics = view.PhysicsData;
            bool const & valid = view.bIsPositionValid;
            return calculateDriverPose(controllerId, pos, rot, physics, valid);
            break;
        }
        case PSMController_Move:
        {
            const PSMPSMove &view = v_controllers[controllerId].controller->ControllerState.PSMoveState;
            const auto & pos = view.Pose.Position;
            const auto & rot = view.Pose.Orientation;
            const auto & physics = view.PhysicsData;
            bool const & valid = view.bIsPositionValid;
            return calculateDriverPose(controllerId, pos, rot, physics, valid);
            break;
        }
        case PSMController_Navi:
        {
            return defaultReadyDriverPose();
            break;
        }
        case PSMController_DualShock4:
        {
            return defaultReadyDriverPose();
            break;
        }
        case PSMController_None:
        {
            return defaultReadyDriverPose();
            break;
        }
        default:
            LOG(FATAL) << "!!! getDriverPose called on a PSMoveHandler device with no known type !!!"; // Not supposed to happen, and may be impossible
            break;
        }
    }
    vr::DriverPose_t getVirtualControllerDriverPose(const int & controllerId) {
        
    }
    vr::DriverPose_t calculateDriverPose(
        const bool & controllerId,
        const PSMVector3f & position,
        const PSMQuatf & orientation,
        const PSMPhysicsData & physicsData,
        const bool & poseIsValid) {
        // Taken from driver_psmoveservice.cpp line 3313, credit to HipsterSloth

        vr::DriverPose_t m_Pose;

        m_Pose.result = vr::TrackingResult_Running_OK;

        m_Pose.deviceIsConnected = v_controllers[controllerId].controller->IsConnected;

        // These should always be false from any modern driver.  These are for Oculus DK1-like
        // rotation-only tracking.  Support for that has likely rotted in vrserver.
        m_Pose.willDriftInYaw = false;
        m_Pose.shouldApplyHeadModel = false;

        // No prediction since that's already handled in the psmove service
        m_Pose.poseTimeOffset = 0.f;

        // Use HMD calibrated space if available, otherwise it's the default vector/quaternion
        m_Pose.qDriverFromHeadRotation.w = 1.f;
        m_Pose.qDriverFromHeadRotation.x = 0.0f;
        m_Pose.qDriverFromHeadRotation.y = 0.0f;
        m_Pose.qDriverFromHeadRotation.z = 0.0f;

        m_Pose.vecDriverFromHeadTranslation[0] = 0.f;
        m_Pose.vecDriverFromHeadTranslation[1] = 0.f;
        m_Pose.vecDriverFromHeadTranslation[2] = 0.f;

        ///*
        m_Pose.qWorldFromDriverRotation.w = m_worldFromDriverPose.Orientation.w;
        m_Pose.qWorldFromDriverRotation.x = m_worldFromDriverPose.Orientation.x;
        m_Pose.qWorldFromDriverRotation.y = m_worldFromDriverPose.Orientation.y;
        m_Pose.qWorldFromDriverRotation.z = m_worldFromDriverPose.Orientation.z;
        m_Pose.vecWorldFromDriverTranslation[0] = m_worldFromDriverPose.Position.x;
        m_Pose.vecWorldFromDriverTranslation[1] = m_worldFromDriverPose.Position.y;
        m_Pose.vecWorldFromDriverTranslation[2] = m_worldFromDriverPose.Position.z;
        //LOG(INFO) << "worldFromDriver: PS : " << m_Pose.vecWorldFromDriverTranslation[0] << ", " << m_Pose.vecWorldFromDriverTranslation[1] << ", " << m_Pose.vecWorldFromDriverTranslation[2];
        // Set position
        
        {
            //const PSMVector3f &position = view.Pose.Position;

            m_Pose.vecPosition[0] = position.x * k_fScalePSMoveAPIToMeters * finalPSMoveScale;
            m_Pose.vecPosition[1] = position.y * k_fScalePSMoveAPIToMeters* finalPSMoveScale;
            m_Pose.vecPosition[2] = position.z * k_fScalePSMoveAPIToMeters* finalPSMoveScale;
        }

        // virtual extend controllers
        float m_fVirtuallExtendControllersYMeters = 0.f;
        float m_fVirtuallExtendControllersZMeters = 0.f;
        if (m_fVirtuallExtendControllersYMeters != 0.0f || m_fVirtuallExtendControllersZMeters != 0.0f)
        {
            //const PSMQuatf &orientation = view.Pose.Orientation;

            PSMVector3f shift = { (float)m_Pose.vecPosition[0], (float)m_Pose.vecPosition[1], (float)m_Pose.vecPosition[2] };

            if (m_fVirtuallExtendControllersZMeters != 0.0f) {

                PSMVector3f local_forward = { 0, 0, -1 };
                PSMVector3f global_forward = PSM_QuatfRotateVector(&orientation, &local_forward);

                shift = PSM_Vector3fScaleAndAdd(&global_forward, m_fVirtuallExtendControllersZMeters, &shift);
            }

            if (m_fVirtuallExtendControllersYMeters != 0.0f) {

                PSMVector3f local_forward = { 0, -1, 0 };
                PSMVector3f global_forward = PSM_QuatfRotateVector(&orientation, &local_forward);

                shift = PSM_Vector3fScaleAndAdd(&global_forward, m_fVirtuallExtendControllersYMeters, &shift);
            }

            m_Pose.vecPosition[0] = shift.x;
            m_Pose.vecPosition[1] = shift.y;
            m_Pose.vecPosition[2] = shift.z;
        }

        // Set rotational coordinates
        bool m_fVirtuallyRotateController = false;
        {
            //const PSMQuatf &orientation = view.Pose.Orientation;

            m_Pose.qRotation.w = m_fVirtuallyRotateController ? -orientation.w : orientation.w;
            m_Pose.qRotation.x = orientation.x;
            m_Pose.qRotation.y = orientation.y;
            m_Pose.qRotation.z = m_fVirtuallyRotateController ? -orientation.z : orientation.z;
        }

        // Set the physics state of the controller
        {
            static float m_fLinearVelocityMultiplier = 1.f;
            static float m_fLinearVelocityExponent = 0.f;

            //const PSMPhysicsData &physicsData = view.PhysicsData;

            m_Pose.vecVelocity[0] = physicsData.LinearVelocityCmPerSec.x
                * abs(pow(abs(physicsData.LinearVelocityCmPerSec.x), m_fLinearVelocityExponent))
                * k_fScalePSMoveAPIToMeters * m_fLinearVelocityMultiplier;
            m_Pose.vecVelocity[1] = physicsData.LinearVelocityCmPerSec.y
                * abs(pow(abs(physicsData.LinearVelocityCmPerSec.y), m_fLinearVelocityExponent))
                * k_fScalePSMoveAPIToMeters * m_fLinearVelocityMultiplier;
            m_Pose.vecVelocity[2] = physicsData.LinearVelocityCmPerSec.z
                * abs(pow(abs(physicsData.LinearVelocityCmPerSec.z), m_fLinearVelocityExponent))
                * k_fScalePSMoveAPIToMeters * m_fLinearVelocityMultiplier;

            m_Pose.vecAcceleration[0] = physicsData.LinearAccelerationCmPerSecSqr.x * k_fScalePSMoveAPIToMeters;
            m_Pose.vecAcceleration[1] = physicsData.LinearAccelerationCmPerSecSqr.y * k_fScalePSMoveAPIToMeters;
            m_Pose.vecAcceleration[2] = physicsData.LinearAccelerationCmPerSecSqr.z * k_fScalePSMoveAPIToMeters;

            m_Pose.vecAngularVelocity[0] = physicsData.AngularVelocityRadPerSec.x;
            m_Pose.vecAngularVelocity[1] = physicsData.AngularVelocityRadPerSec.y;
            m_Pose.vecAngularVelocity[2] = physicsData.AngularVelocityRadPerSec.z;

            m_Pose.vecAngularAcceleration[0] = physicsData.AngularAccelerationRadPerSecSqr.x;
            m_Pose.vecAngularAcceleration[1] = physicsData.AngularAccelerationRadPerSecSqr.y;
            m_Pose.vecAngularAcceleration[2] = physicsData.AngularAccelerationRadPerSecSqr.z;
        }

        m_Pose.poseIsValid =
            poseIsValid;

        return m_Pose;
    }

private:
    bool startup() {

        bool success = true;

        if (PSM_Initialize(PSMOVESERVICE_DEFAULT_ADDRESS, PSMOVESERVICE_DEFAULT_PORT, PSM_DEFAULT_TIMEOUT) == PSMResult_Success)
        {
            LOG(INFO) << "PSMoveConsoleClient::startup() - Initialized client version - " << PSM_GetClientVersionString();
        }
        else
        {
            LOG(INFO) << "PSMoveConsoleClient::startup() - Failed to initialize the client network manager";
            success = false;
        }

        if (success)
        {
            loadTrackingCalibration();

            rebuildControllerList();
            rebuildTrackerList();
            rebuildHmdList();

            // Register as listener and start stream for each controller
            unsigned int data_stream_flags =
                PSMControllerDataStreamFlags::PSMStreamFlags_includePositionData |
                PSMControllerDataStreamFlags::PSMStreamFlags_includePhysicsData |
                PSMControllerDataStreamFlags::PSMStreamFlags_includeCalibratedSensorData |
                PSMControllerDataStreamFlags::PSMStreamFlags_includeRawTrackerData;

            if (controllerList.count > 0 || trackerList.count > 0) {
                for (int i = 0; i < controllerList.count; ++i) { 
                    // In order for the controllers to report more than their IMU stuff, and turn on the light, they all have to go through this
                    if (PSM_AllocateControllerListener(controllerList.controller_id[i]) != PSMResult_Success) {
                        success = false;
                    }
                    if (PSM_StartControllerDataStream(controllerList.controller_id[i], data_stream_flags, PSM_DEFAULT_TIMEOUT) != PSMResult_Success) {
                        success = false;
                    }
                }
                rebuildPSMovesForPool();
                rebuildPSEyesForPool();
            }
            else {
                LOG(INFO) << "PSMoveConsoleClient::startup() - No controllers found.";
                success = false;
            }
        }

        if (success)
        {
            last_report_fps_timestamp =
                std::chrono::duration_cast< std::chrono::milliseconds >(
                    std::chrono::system_clock::now().time_since_epoch());
        }

        return success;
    }
    void update()
    {
        rebuildPSMoveLists();

        // Get the controller data for each controller
        if (m_keepRunning)
        {
            processKeyInputs();
            for (int i = 0; i < v_controllers.size(); ++i) {
                //const PSMPSMove &view = v_controllers[i].controller->ControllerState.PSMoveState;
                //LOG(INFO) << "Controller " << i << " has a battery level of " << (int)view.BatteryValue;
                KVR::TrackedDeviceInputData data = defaultDeviceData(i);
                data.pose = getDriverPose(i);
                // Reuse, instead of recalling for PSMoveState
                data.position = { 
                    data.pose.vecPosition[0],
                    data.pose.vecPosition[1],
                    data.pose.vecPosition[2] }; // Pose stores as array
                data.rotation = data.pose.qRotation;
                
                TrackingPoolManager::updatePoolWithDevice(data, data.deviceId);
            }
            for (int i = 0; i < v_eyeTrackers.size(); ++i) {
                KVR::TrackedDeviceInputData data = defaultDeviceData_PSEYE(i);
                data.pose = getPSEyeDriverPose(i);
                // Reuse, instead of recalling for PSMoveState
                data.position = {
                    data.pose.vecPosition[0],
                    data.pose.vecPosition[1],
                    data.pose.vecPosition[2] }; // Pose stores as array

                //LOG(INFO) << "PSEYE " << i << data.position.v[0] << ", " << data.position.v[1] << ", " << data.position.v[2];
                data.rotation = data.pose.qRotation;

                TrackingPoolManager::updatePoolWithDevice(data, data.deviceId);
            }
        }
    }

    bool rb = false;
    void processKeyInputs() {
        if (controllerList.count == 0 || v_controllers.size() == 0) { return; }
        bool inputAvailable = false;
        for (MoveWrapper_PSM& wrapper : v_controllers) {
            if (wrapper.controller->ControllerType == PSMControllerType::PSMController_Move) {
                inputAvailable = true;
            }
            else
                inputAvailable = false;

            if (!inputAvailable) {
                continue;
            }
            auto& controller = wrapper.controller->ControllerState.PSMoveState;
            bool bStartRealignHMDTriggered =
                (controller.StartButton == PSMButtonState_PRESSED
                    || controller.StartButton == PSMButtonState_DOWN) && (controller.SelectButton == PSMButtonState_PRESSED
                        || controller.SelectButton == PSMButtonState_DOWN);
            bool bStartVRRatioCalibrationTriggered = (
                controller.CrossButton == PSMButtonState_PRESSED

                ) &&
                (controller.CircleButton == PSMButtonState_PRESSED
                    );
            
            KinectSettings::KVRPSMoveData[wrapper.controller->ControllerID].PSMoveData = controller;
            KinectSettings::KVRPSMoveData[wrapper.controller->ControllerID].isValidController = true;

            for (int psmid = 0; psmid < KinectSettings::psmindexidpsm[0].size(); psmid++) {
                if (KinectSettings::psmindexidpsm[0].at(psmid) == KinectSettings::psmh
                    && wrapper.controller->ControllerID == KinectSettings::psmindexidpsm[1].at(psmid)) {
                    KinectSettings::hidariMove = controller;
                    if (KinectSettings::psmindexidpsm[0].at(psmid) == KinectSettings::flashnow[0] && KinectSettings::flashnow[1]) {
                        std::thread* rb = new std::thread(flashRainbow, wrapper.controller->ControllerID);
                        KinectSettings::flashnow[1] = false;
                    }
                }

                if (KinectSettings::psmindexidpsm[0].at(psmid) == KinectSettings::psmm
                    && wrapper.controller->ControllerID == KinectSettings::psmindexidpsm[1].at(psmid)) {
                    KinectSettings::migiMove = controller;
                    if (KinectSettings::psmindexidpsm[0].at(psmid) == KinectSettings::flashnow[0] && KinectSettings::flashnow[1]) {
                        std::thread* rb = new std::thread(flashRainbow, wrapper.controller->ControllerID);
                        KinectSettings::flashnow[1] = false;
                    }
                }

                if (KinectSettings::psmindexidpsm[0].at(psmid) == KinectSettings::psmhidari
                    && wrapper.controller->ControllerID == KinectSettings::psmindexidpsm[1].at(psmid)) {
                    KinectSettings::hidariashimove = controller;
                    if (KinectSettings::psmindexidpsm[0].at(psmid) == KinectSettings::flashnow[0] && KinectSettings::flashnow[1]) {
                        std::thread* rb = new std::thread(flashRainbow, wrapper.controller->ControllerID);
                        KinectSettings::flashnow[1] = false;
                    }
                }

                if (KinectSettings::psmindexidpsm[0].at(psmid) == KinectSettings::psmmigi
                    && wrapper.controller->ControllerID == KinectSettings::psmindexidpsm[1].at(psmid)) {
                    KinectSettings::migiashimove = controller;
                    if (KinectSettings::psmindexidpsm[0].at(psmid) == KinectSettings::flashnow[0] && KinectSettings::flashnow[1]) {
                        std::thread* rb = new std::thread(flashRainbow, wrapper.controller->ControllerID);
                        KinectSettings::flashnow[1] = false;
                    }
                }

                if (KinectSettings::psmindexidpsm[0].at(psmid) == KinectSettings::psmyobu
                    && wrapper.controller->ControllerID == KinectSettings::psmindexidpsm[1].at(psmid)) {
                    KinectSettings::yobumove = controller;
                    if (KinectSettings::psmindexidpsm[0].at(psmid) == KinectSettings::flashnow[0] && KinectSettings::flashnow[1]) {
                        std::thread* rb = new std::thread(flashRainbow, wrapper.controller->ControllerID);
                        KinectSettings::flashnow[1] = false;
                    }
                }

                if (KinectSettings::psmindexidpsm[0].at(psmid) == KinectSettings::psmatama
                    && wrapper.controller->ControllerID == KinectSettings::psmindexidpsm[1].at(psmid)) {
                    KinectSettings::atamamove = controller;
                    if (KinectSettings::psmindexidpsm[0].at(psmid) == KinectSettings::flashnow[0] && KinectSettings::flashnow[1]) {
                        std::thread* rb = new std::thread(flashRainbow, wrapper.controller->ControllerID);
                        KinectSettings::flashnow[1] = false;
                    }
                }
            }

            if (bStartRealignHMDTriggered) {
                PSMVector3f controllerBallPointedUpEuler = { (float)M_PI_2, 0.0f, 0.0f };

                PSMQuatf controllerBallPointedUpQuat = PSM_QuatfCreateFromAngles(&controllerBallPointedUpEuler);

                PSM_ResetControllerOrientationAsync(wrapper.controller->ControllerID, &controllerBallPointedUpQuat, nullptr);

                alignPSMoveAndHMDTrackingSpace(wrapper.controller);
            }
            bool enabledRatioCalibration = false;
            if (bStartVRRatioCalibrationTriggered && enabledRatioCalibration) {
                static bool zeroCalibrationAttempted = false;
                static PSMVector3f zeroPos = {0,0,0};
                static PSMVector3f endPos = {0,0,0};
                static const double calibrationDistance = 20; // centimeters, real life distance expected

                if (zeroCalibrationAttempted) {
                    endPos = controller.Pose.Position;
                    zeroCalibrationAttempted = false;

                    auto zeroToEndVector = PSM_Vector3fSubtract(&endPos, &zeroPos);
                    double length = PSM_Vector3fLength(&zeroToEndVector);
                    finalPSMoveScale = calibrationDistance / length;
                    LOG(INFO) << "Set PSMove Scale Factor to " << finalPSMoveScale;

                    zeroPos = { 0,0,0 };
                    endPos = { 0,0,0 };
                }
                else {
                    zeroPos = controller.Pose.Position;
                    zeroCalibrationAttempted = true;
                }
                

            }
        }

    }

    void loadTrackingCalibration() {
        KVR::TrackingSystemCalibration calibration = KVR::retrieveSystemCalibration(k_trackingSystemName);
        m_worldFromDriverPose.Orientation.w = calibration.driverFromWorldRotation.w;
        m_worldFromDriverPose.Orientation.x = calibration.driverFromWorldRotation.x;
        m_worldFromDriverPose.Orientation.y = calibration.driverFromWorldRotation.y;
        m_worldFromDriverPose.Orientation.z = calibration.driverFromWorldRotation.z;

        m_worldFromDriverPose.Position.x = calibration.driverFromWorldPosition.v[0];
        m_worldFromDriverPose.Position.y = calibration.driverFromWorldPosition.v[1];
        m_worldFromDriverPose.Position.z = calibration.driverFromWorldPosition.v[2];
    }
    void saveTrackingCalibration() {
        KVR::TrackingSystemCalibration calibration;
        calibration.driverFromWorldRotation.w = m_worldFromDriverPose.Orientation.w;
        calibration.driverFromWorldRotation.x = m_worldFromDriverPose.Orientation.x;
        calibration.driverFromWorldRotation.y = m_worldFromDriverPose.Orientation.y;
        calibration.driverFromWorldRotation.z = m_worldFromDriverPose.Orientation.z;

        calibration.driverFromWorldPosition.v[0] = m_worldFromDriverPose.Position.x;
        calibration.driverFromWorldPosition.v[1] = m_worldFromDriverPose.Position.y;
        calibration.driverFromWorldPosition.v[2] = m_worldFromDriverPose.Position.z;

        KVR::saveSystemCalibration(k_trackingSystemName, calibration);
    }
    KVR::TrackedDeviceInputData defaultDeviceData(uint32_t localID) {
        KVR::TrackedDeviceInputData data;
        data.parentHandler = dynamic_cast<DeviceHandler*>(this);
        data.deviceName = "PSMOVE " + std::to_string(localID);
        data.deviceId = v_controllers[localID].id.globalID;
        data.serial = controllerList.controller_serial[localID];
        data.customModelName = "{k2vr}psmove_controller";
        return data;
    }
    KVR::TrackedDeviceInputData defaultDeviceData_PSEYE(uint32_t localID) {
        KVR::TrackedDeviceInputData data;
        data.parentHandler = dynamic_cast<DeviceHandler*>(this);
        data.deviceName = "PSEYE " + std::to_string(localID);
        data.deviceId = v_eyeTrackers[localID].id.globalID;
        data.serial = v_eyeTrackers[localID].trackerInfo.device_path; // For the eyes, the path is basically their serial number
        data.customModelName = "{k2vr}ps3eye_tracker";
        return data;
    }
    std::string batteryValueString(PSMBatteryState battery) {
        switch (battery) {
        case PSMBattery_0:
            return "~0%";
        case PSMBattery_20:
            return "~20%";
        case PSMBattery_40:
            return "~40%";
        case PSMBattery_60:
            return "~60%";
        case PSMBattery_80:
            return "~80%";
        case PSMBattery_100:
            return "~100%";
        case PSMBattery_Charging:
            return "Charging...";
        case PSMBattery_Charged:
            return "Charged!";
        default:
            LOG(ERROR) << "INVALID BATTERY VALUE!!!";
            return "INVALID";
        }
    }
    void rebuildPSEyesForPool() {
        for (int i = 0; i < v_eyeTrackers.size(); ++i) {
            TrackingPoolManager::clearDeviceInPool(v_eyeTrackers[i].id.globalID);
        } // Clear last controllers in pool

        v_eyeTrackers.clear(); // All old controllers must be gone
        for (int i = 0; i < trackerList.count; ++i) {
            auto eye = trackerList.trackers[i];
            // Check that it's actually a Psmove/Virtual, as there could be dualshock's connected
            TrackerWrapper_PSM wrapper;
            wrapper.trackerInfo = eye;
            v_eyeTrackers.push_back(wrapper);
        }

        for (int i = 0; i < v_eyeTrackers.size(); ++i) {
            // Redo the loop over the successfully excised trackers (PS Move's only)
            // But this time edit the wrapper with the tracking pool id's
            v_eyeTrackers[i].id.internalID = i;
            KVR::TrackedDeviceInputData data = defaultDeviceData_PSEYE(i);
            uint32_t gID = k_invalidTrackerID;
            TrackingPoolManager::addDeviceToPool(data, gID);
            v_eyeTrackers[i].id.globalID = gID;
        }
    }
    void rebuildPSMovesForPool() {
        for (int i = 0; i < v_controllers.size(); ++i) {
            TrackingPoolManager::clearDeviceInPool(v_controllers[i].id.globalID);
        } // Clear last controllers in pool

        v_controllers.clear(); // All old controllers must be gone
        for (int i = 0; i < controllerList.count; ++i) {
            auto controller = PSM_GetController(controllerList.controller_id[i]);
            // Check that it's actually a Psmove/Virtual, as there could be dualshock's connected
            if (controller->ControllerType == PSMController_Move ||
                controller->ControllerType == PSMController_Virtual) {
                MoveWrapper_PSM wrapper;
                wrapper.controller = controller;
                v_controllers.push_back(wrapper);
            }
        }

        for (int i = 0; i < v_controllers.size(); ++i) {
            // Redo the loop over the successfully excised trackers (PS Move's only)
            // But this time edit the wrapper with the tracking pool id's
            v_controllers[i].id.internalID = i;
            KVR::TrackedDeviceInputData data = defaultDeviceData(i);
            uint32_t gID = k_invalidTrackerID;
            TrackingPoolManager::addDeviceToPool(data, gID);
            v_controllers[i].id.globalID = gID;

            if (v_controllers[i].controller->ControllerType == PSMController_Move) {
                auto value = v_controllers[i].controller->ControllerState.PSMoveState.BatteryValue;

                LOG(INFO) << "Controller " << i << " has battery level: " << batteryValueString(value);
            }
        }
    }
    void rebuildPSMoveLists() {
        // Unneccessary holdover from inspiration for implementation
        //std::chrono::milliseconds now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
        //std::chrono::milliseconds diff = now - last_report_fps_timestamp;

        // Polls events and updates controller state
        if (PSM_Update() == PSMResult_Success) {
            m_keepRunning = true;
        }
        else {
            m_keepRunning = false;
        }

        // See if we need to rebuild the controller list
        if (m_keepRunning && PSM_HasControllerListChanged())
        {
            unsigned int data_stream_flags =
                PSMControllerDataStreamFlags::PSMStreamFlags_includePositionData |
                PSMControllerDataStreamFlags::PSMStreamFlags_includePhysicsData |
                PSMControllerDataStreamFlags::PSMStreamFlags_includeCalibratedSensorData |
                PSMControllerDataStreamFlags::PSMStreamFlags_includeRawTrackerData;

            // Stop all controller streams
            for (int i = 0; i < controllerList.count; ++i) {
                PSM_StopControllerDataStream(controllerList.controller_id[i], PSM_DEFAULT_TIMEOUT);
            }

            // Get the current controller list
            rebuildControllerList();

            // Restart the controller streams
            if (controllerList.count > 0) {
                for (int i = 0; i < controllerList.count; ++i) {
                    if (PSM_StartControllerDataStream(controllerList.controller_id[i], data_stream_flags, PSM_DEFAULT_TIMEOUT) != PSMResult_Success) {
                        m_keepRunning = false;
                        LOG (ERROR) << "Controller stream " << i << " failed to start!";
                    }
                }
                // Rebuild K2VR Controller List for Trackers
                rebuildPSMovesForPool(); // Here, because of timing issue, where controllers will report as 'None' occasionally when uninitialised properly
                
            }
            else {
                LOG(INFO) << "PSMoveConsoleClient::startup() - No controllers found.";
                m_keepRunning = false;
            }
        }

        // See if we need to rebuild the tracker list
        if (m_keepRunning && PSM_HasTrackerListChanged())
        {
            rebuildTrackerList();
            rebuildPSEyesForPool();
        }

        // See if we need to rebuild the hmd list
        if (m_keepRunning && PSM_HasHMDListChanged())
        {
            rebuildTrackerList();
        }
    }
   
    void rebuildControllerList()
    {
        memset(&controllerList, 0, sizeof(PSMControllerList));
        int timeout = 2000;
        auto result = PSM_GetControllerList(&controllerList, timeout);
        LOG(INFO) << "PSM_GetControllerList returned " << PSMResultToString(result);

        LOG(INFO) << "Found " << controllerList.count << " controllers.";

        for (int cntlr_ix = 0; cntlr_ix<controllerList.count; ++cntlr_ix)
        {
            const char *controller_type = "NONE";

            switch (controllerList.controller_type[cntlr_ix])
            {
            case PSMController_Move:
                controller_type = "PSMove";
                break;
            case PSMController_Navi:
                controller_type = "PSNavi";
                break;
            case PSMController_DualShock4:
                controller_type = "DualShock4";
                break;
            case PSMController_Virtual:
                controller_type = "Virtual";
                break;
            }

            LOG(INFO) << "  Controller ID: " << controllerList.controller_id[cntlr_ix] << " is a " << controller_type;
        }
    }

    void rebuildTrackerList()
    {
        memset(&trackerList, 0, sizeof(PSMTrackerList));
        int timeout = 2000;
        auto result = PSM_GetTrackerList(&trackerList, timeout);
        LOG(INFO) << "PSM_GetTrackerList returned " << PSMResultToString(result);

        LOG(INFO) << "Found " << trackerList.count << " trackers.";

        for (int tracker_ix = 0; tracker_ix<trackerList.count; ++tracker_ix)
        {
            const char *tracker_type = "NONE";

            switch (trackerList.trackers[tracker_ix].tracker_type)
            {
            case PSMTracker_PS3Eye:
                tracker_type = "PS3Eye";
                break;
            }

            LOG(INFO) << "  Tracker ID: " << trackerList.trackers[tracker_ix].tracker_id << " is a " << tracker_type;
        }
    }

    void rebuildHmdList()
    {
        memset(&hmdList, 0, sizeof(PSMHmdList));
        PSM_GetHmdList(&hmdList, PSM_DEFAULT_TIMEOUT);

        LOG(INFO) << "Found " << hmdList.count << " HMDs.";

        for (int hmd_ix = 0; hmd_ix<hmdList.count; ++hmd_ix)
        {
            const char *hmd_type = "NONE";

            switch (hmdList.hmd_type[hmd_ix])
            {
            case PSMHmd_Morpheus:
                hmd_type = "Morpheus";
                break;
            case PSMHmd_Virtual:
                hmd_type = "Virtual";
                break;
            }

            LOG(INFO) << "  HMD ID: " << hmdList.hmd_id[hmd_ix] << " is a " << hmd_type;
        }
    }

    std::chrono::milliseconds last_report_fps_timestamp;

    struct MoveWrapper_PSM {
        PSMController* controller = nullptr;
        TrackerIDs id;
    };
    std::vector<MoveWrapper_PSM> v_controllers;

    struct TrackerWrapper_PSM {
        PSMClientTrackerInfo trackerInfo;
        TrackerIDs id;
    };
    std::vector<TrackerWrapper_PSM> v_eyeTrackers;

    PSMControllerList controllerList;
    PSMTrackerList trackerList;
    PSMHmdList hmdList;
    bool m_started;
    bool m_keepRunning;

    //Vars
    bool m_bDisableHMDAlignmentGesture = false;
    float m_fControllerMetersInFrontOfHmdAtCalibration = 0.06f;
    bool m_bUseControllerOrientationInHMDAlignment = false; // HMD *may* influence the magnetic field and disrupt readings
    PSMPosef m_worldFromDriverPose = { {0,0,0}, {1,0,0,0} };
    bool m_hasCalibratedWorldFromDriverPose = false;
    //Constants
    const float k_fScalePSMoveAPIToMeters = 0.01f;
    double finalPSMoveScale = 1.0;
    const std::string k_trackingSystemName = "psmove";
};