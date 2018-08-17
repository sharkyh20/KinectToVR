#pragma once

#include <iostream>
#include <iomanip>
#include <chrono>
#include <cstring>

#include <PSMoveClient_CAPI.h>
#include <ClientConstants.h>
#include <SharedConstants.h>

#include <openvr.h>

#define M_PI_2 1.57079632679

class PSMoveHandler {
    // Heavily based off of the example template program
    // test_console_CAPI.cpp in the MoveService project
public:
    PSMoveHandler() 
        : m_started(false),
        m_keepRunning(true)
    {
        try {
            if (startup()) {

            }
            else {
                std::cerr << "Failed to startup the PSMoveClient!!!" << std::endl;
            }
        }
        catch (std::exception& e) {
            std::cerr << e.what() << std::endl;
        }
    }

    // Attempt to start and run the client
    int run() {
        if (m_keepRunning) {
            update();

            //_PAUSE(1); // Maybe uneccessary with all the other processing?
        }
        return 0;
    }
    void shutdown()
    {
        if (controllerList.count > 0)
        {
            PSM_StopControllerDataStream(controllerList.controller_id[0], PSM_DEFAULT_TIMEOUT);
            PSM_FreeControllerListener(controllerList.controller_id[0]);
        }
        // No tracker data streams started
        // No HMD data streams started

        PSM_Shutdown();
    }

    vr::HmdQuaternion_t getMoveOrientation(int controllerId) {
        if (controllerId < v_controllers.size()) {
            auto controllerState = v_controllers[controllerId]->ControllerState.PSMoveState;
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
    vr::HmdVector3d_t getMovePosition(int controllerId) {
        if (controllerId < v_controllers.size()) {
            auto controllerState = v_controllers[controllerId]->ControllerState.PSMoveState;
            if (controllerState.bIsPositionValid) {
                vr::HmdVector3d_t pos;
                PSMVector3f pspos = controllerState.Pose.Position;

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

    void alignPSMoveAndHMDTrackingSpace() {
        if (m_bDisableHMDAlignmentGesture)
        {
            return;
        }

        vr::TrackedDeviceIndex_t hmd_device_index = 0;
        
        PSMPosef hmd_pose_meters;
        hmd_pose_meters = openvrMatrixExtractPSMPosef(KinectSettings::hmdAbsoluteTracking);

        // Make the HMD orientation only contain a yaw
        hmd_pose_meters.Orientation = ExtractHMDYawQuaternion(hmd_pose_meters.Orientation);

        auto m_PSMControllerType = v_controllers[0]->ControllerType;
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
        PSM_GetControllerPose(v_controllers[0]->ControllerID, &controller_pose_meters);

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
        
    }

    vr::DriverPose_t getPSMoveDriverPose(int controllerId) {
        // Taken from driver_psmoveservice.cpp line 3313, credit to HipsterSloth
        const PSMPSMove &view = v_controllers[controllerId]->ControllerState.PSMoveState;

        vr::DriverPose_t m_Pose;

        m_Pose.result = vr::TrackingResult_Running_OK;

        m_Pose.deviceIsConnected = v_controllers[controllerId]->IsConnected;

        // These should always be false from any modern driver.  These are for Oculus DK1-like
        // rotation-only tracking.  Support for that has likely rotted in vrserver.
        m_Pose.willDriftInYaw = false;
        m_Pose.shouldApplyHeadModel = false;

        // No prediction since that's already handled in the psmove service
        m_Pose.poseTimeOffset = 0.f;

        // No transform due to the current HMD orientation
        m_Pose.qDriverFromHeadRotation.w = 1.f;
        m_Pose.qDriverFromHeadRotation.x = 0.0f;
        m_Pose.qDriverFromHeadRotation.y = 0.0f;
        m_Pose.qDriverFromHeadRotation.z = 0.0f;
        m_Pose.vecDriverFromHeadTranslation[0] = 0.f;
        m_Pose.vecDriverFromHeadTranslation[1] = 0.f;
        m_Pose.vecDriverFromHeadTranslation[2] = 0.f;

        // Set position
        
        {
            const PSMVector3f &position = view.Pose.Position;

            m_Pose.vecPosition[0] = position.x * k_fScalePSMoveAPIToMeters;
            m_Pose.vecPosition[1] = position.y * k_fScalePSMoveAPIToMeters;
            m_Pose.vecPosition[2] = position.z * k_fScalePSMoveAPIToMeters;
        }

        // virtual extend controllers
        float m_fVirtuallExtendControllersYMeters = 0.0f;
        float m_fVirtuallExtendControllersZMeters = 0.0f;
        if (m_fVirtuallExtendControllersYMeters != 0.0f || m_fVirtuallExtendControllersZMeters != 0.0f)
        {
            const PSMQuatf &orientation = view.Pose.Orientation;

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
            const PSMQuatf &orientation = view.Pose.Orientation;

            m_Pose.qRotation.w = m_fVirtuallyRotateController ? -orientation.w : orientation.w;
            m_Pose.qRotation.x = orientation.x;
            m_Pose.qRotation.y = orientation.y;
            m_Pose.qRotation.z = m_fVirtuallyRotateController ? -orientation.z : orientation.z;
        }

        // Set the physics state of the controller
        {
            float m_fLinearVelocityMultiplier;
            float m_fLinearVelocityExponent;

            const PSMPhysicsData &physicsData = view.PhysicsData;

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
            v_controllers[controllerId]->ControllerState.PSMoveState.bIsPositionValid &&
            v_controllers[controllerId]->ControllerState.PSMoveState.bIsOrientationValid;

        return m_Pose;
    }

private:
    bool startup() {

        bool success = true;

        if (PSM_Initialize(PSMOVESERVICE_DEFAULT_ADDRESS, PSMOVESERVICE_DEFAULT_PORT, PSM_DEFAULT_TIMEOUT) == PSMResult_Success)
        {
            std::cout << "PSMoveConsoleClient::startup() - Initialized client version - " << PSM_GetClientVersionString() << std::endl;
        }
        else
        {
            std::cout << "PSMoveConsoleClient::startup() - Failed to initialize the client network manager" << std::endl;
            success = false;
        }

        if (success)
        {
            rebuildControllerList();
            rebuildTrackerList();
            rebuildHmdList();

            // Register as listener and start stream for each controller
            unsigned int data_stream_flags =
                PSMControllerDataStreamFlags::PSMStreamFlags_includePositionData |
                PSMControllerDataStreamFlags::PSMStreamFlags_includePhysicsData |
                PSMControllerDataStreamFlags::PSMStreamFlags_includeCalibratedSensorData |
                PSMControllerDataStreamFlags::PSMStreamFlags_includeRawTrackerData;

            if (controllerList.count > 0) {
                if (PSM_AllocateControllerListener(controllerList.controller_id[0]) != PSMResult_Success) {
                    success = false;
                }
                if (PSM_StartControllerDataStream(controllerList.controller_id[0], data_stream_flags, PSM_DEFAULT_TIMEOUT) != PSMResult_Success) {
                    success = false;
                }
            }
            else {
                std::cout << "PSMoveConsoleClient::startup() - No controllers found." << std::endl;
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

        // Get the controller data for the first controller
        if (m_keepRunning)
        {
            v_controllers.push_back(PSM_GetController(controllerList.controller_id[0]));
            v_controllers.push_back(PSM_GetController(controllerList.controller_id[1]));
            v_controllers.push_back(PSM_GetController(controllerList.controller_id[2]));

            PSMQuatf orientation = v_controllers[0]->ControllerState.PSMoveState.Pose.Orientation;
            /*
            std::cout << "Controller 0 (AGMXYZ):  ";

            std::cout << std::setw(12) << std::right << std::setprecision(6) << calibsens.Accelerometer.x;
            std::cout << std::setw(12) << std::right << std::setprecision(6) << calibsens.Accelerometer.y;
            std::cout << std::setw(12) << std::right << std::setprecision(6) << calibsens.Accelerometer.z;

            std::cout << std::setw(12) << std::right << std::setprecision(6) << calibsens.Gyroscope.x;
            std::cout << std::setw(12) << std::right << std::setprecision(6) << calibsens.Gyroscope.y;
            std::cout << std::setw(12) << std::right << std::setprecision(6) << calibsens.Gyroscope.z;

            std::cout << std::setw(12) << std::right << std::setprecision(6) << calibsens.Magnetometer.x;
            std::cout << std::setw(12) << std::right << std::setprecision(6) << calibsens.Magnetometer.y;
            std::cout << std::setw(12) << std::right << std::setprecision(6) << calibsens.Magnetometer.z;

            PSMVector3f position = controller0->ControllerState.PSMoveState.RawTrackerData.RelativePositionCm;
            std::cout << std::setw(12) << std::right << std::setprecision(6) << position.x;
            std::cout << std::setw(12) << std::right << std::setprecision(6) << position.y;
            std::cout << std::setw(12) << std::right << std::setprecision(6) << position.z;

            std::cout << std::endl;
            */

            // Don't need it to shutdown when button is pressed
            //if (v_controllers[0]->ControllerState.PSMoveState.CrossButton != PSMButtonState_UP)
            //{
            //    m_keepRunning = false;
            //}
            
        }
    }
    void rebuildPSMoveLists() {
        std::chrono::milliseconds now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
        std::chrono::milliseconds diff = now - last_report_fps_timestamp;

        // Polls events and updates controller state
        if (PSM_Update() != PSMResult_Success)
        {
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
            PSM_StopControllerDataStream(controllerList.controller_id[0], PSM_DEFAULT_TIMEOUT);

            // Get the current controller list
            rebuildControllerList();

            // Restart the controller streams
            if (controllerList.count > 0) {
                if (PSM_StartControllerDataStream(controllerList.controller_id[0], data_stream_flags, PSM_DEFAULT_TIMEOUT) != PSMResult_Success) {
                    m_keepRunning = false;
                }
            }
            else {
                std::cout << "PSMoveConsoleClient::startup() - No controllers found." << std::endl;
                m_keepRunning = false;
            }
        }

        // See if we need to rebuild the tracker list
        if (m_keepRunning && PSM_HasTrackerListChanged())
        {
            rebuildTrackerList();
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
        PSM_GetControllerList(&controllerList, PSM_DEFAULT_TIMEOUT);

        std::cout << "Found " << controllerList.count << " controllers." << std::endl;

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

            std::cout << "  Controller ID: " << controllerList.controller_id[cntlr_ix] << " is a " << controller_type << std::endl;
        }
    }

    void rebuildTrackerList()
    {
        memset(&trackerList, 0, sizeof(PSMTrackerList));
        PSM_GetTrackerList(&trackerList, PSM_DEFAULT_TIMEOUT);

        std::cout << "Found " << trackerList.count << " trackers." << std::endl;

        for (int tracker_ix = 0; tracker_ix<trackerList.count; ++tracker_ix)
        {
            const char *tracker_type = "NONE";

            switch (trackerList.trackers[tracker_ix].tracker_type)
            {
            case PSMTracker_PS3Eye:
                tracker_type = "PS3Eye";
                break;
            }

            std::cout << "  Tracker ID: " << trackerList.trackers[tracker_ix].tracker_id << " is a " << tracker_type << std::endl;
        }
    }

    void rebuildHmdList()
    {
        memset(&hmdList, 0, sizeof(PSMHmdList));
        PSM_GetHmdList(&hmdList, PSM_DEFAULT_TIMEOUT);

        std::cout << "Found " << hmdList.count << " HMDs." << std::endl;

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

            std::cout << "  HMD ID: " << hmdList.hmd_id[hmd_ix] << " is a " << hmd_type << std::endl;
        }
    }

    std::chrono::milliseconds last_report_fps_timestamp;
    std::vector<PSMController*> v_controllers;
    PSMControllerList controllerList;
    PSMTrackerList trackerList;
    PSMHmdList hmdList;
    bool m_started;
    bool m_keepRunning;

    //Vars
    bool m_bDisableHMDAlignmentGesture = false;
    float m_fControllerMetersInFrontOfHmdAtCalibration;
    bool m_bUseControllerOrientationInHMDAlignment = false;
    //Constants
    const float k_fScalePSMoveAPIToMeters = 0.01f;
};