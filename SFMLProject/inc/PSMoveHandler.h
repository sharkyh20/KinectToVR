#pragma once

#include <iostream>
#include <iomanip>
#include <chrono>
#include <cstring>

#include <PSMoveClient_CAPI.h>
#include <ClientConstants.h>

#include <openvr.h>

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
};