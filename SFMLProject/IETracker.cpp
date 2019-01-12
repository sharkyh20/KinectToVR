#include "stdafx.h"
#include "IETracker.h"
#include "KinectSettings.h"
#include <iostream>

uint32_t initTracker(vrinputemulator::VRInputEmulator &inputEmulator, bool connected) {

    uint32_t deviceId = 0;
    bool revivingPastDevice = false;

    uint32_t count = inputEmulator.getVirtualDeviceCount();

    if (count > 0) { // Use dead trackers so that SteamVR doesn't get clogged up with all of them
        for (int i = 0; i < count; ++i) {
            vrinputemulator::VirtualDeviceInfo info = inputEmulator.getVirtualDeviceInfo(i);
            if (info.openvrDeviceId == vr::k_unTrackedDeviceIndexInvalid) // Usually the latest spawned trackers will be technically invalid
                continue;
            if (!vr::VRSystem()->IsTrackedDeviceConnected(info.openvrDeviceId)) {
                std::cout << "Found disconnected device at " << info.openvrDeviceId << "(VR) " << info.virtualDeviceId << " (IE)\n";
                deviceId = info.virtualDeviceId;
                revivingPastDevice = true; // RISE FROM YOUR GRAVE
                break;
            }
        }
    }

    if (revivingPastDevice) {
        setTrackerDefaultProperties(inputEmulator, deviceId);
    }
    else {
        try {
            deviceId = count;
            inputEmulator.addVirtualDevice(vrinputemulator::VirtualDeviceType::TrackedController, std::to_string(deviceId), false);
            inputEmulator.enableDeviceOffsets(deviceId, true);
            setTrackerDefaultProperties(inputEmulator, deviceId); // These properties MUST be set before publishing the device, or it throws
            inputEmulator.publishVirtualDevice(deviceId);
        }
        catch (vrinputemulator::vrinputemulator_exception& e) {
            std::cerr << e.what() << '\n';
        }
    }
    
    
    //Connect device
    auto pose = inputEmulator.getVirtualDevicePose(deviceId);
    if (pose.deviceIsConnected != connected) {
        pose.deviceIsConnected = connected;
        pose.poseIsValid = connected;
        inputEmulator.setVirtualDevicePose(deviceId, pose);
    }
    return deviceId;
}

void setTrackerDefaultProperties(vrinputemulator::VRInputEmulator &ie, uint32_t &vrDeviceId) {
    using namespace vr;
    setDeviceProperty(ie, vrDeviceId, Prop_TrackingSystemName_String, "string", "psvr"); // Necessary for auto calibration to only apply to these trackers
    setDeviceProperty(ie, vrDeviceId, Prop_ModelNumber_String, "string", "Vive Controller MV");
    setDeviceProperty(ie, vrDeviceId, Prop_RenderModelName_String, "string", "vr_controller_vive_1_5"); // Changed for specific devices, but for now, 
    setDeviceProperty(ie, vrDeviceId, Prop_WillDriftInYaw_Bool, "bool", "0");
    setDeviceProperty(ie, vrDeviceId, Prop_ManufacturerName_String, "string", "HTC");
    setDeviceProperty(ie, vrDeviceId, Prop_TrackingFirmwareVersion_String, "string", "1465809478 htcvrsoftware@firmware-win32 2016-06-13 FPGA 1.6/0/0 VRC 1465809477 Radio 1466630404");
    setDeviceProperty(ie, vrDeviceId, Prop_HardwareRevision_String, "string", "product 129 rev 1.5.0 lot 2000/0/0 0");
    setDeviceProperty(ie, vrDeviceId, Prop_DeviceIsWireless_Bool, "bool", "1");
    setDeviceProperty(ie, vrDeviceId, Prop_HardwareRevision_Uint64, "uint64", "2164327680");
    setDeviceProperty(ie, vrDeviceId, Prop_FirmwareVersion_Uint64, "uint64", "1465809478");
    setDeviceProperty(ie, vrDeviceId, Prop_DeviceClass_Int32, "int32", "2"); 
    //setDeviceProperty(ie, vrDeviceId, Prop_SupportedButtons_Uint64, "uint64", "12884901895");
    setDeviceProperty(ie, vrDeviceId, Prop_Axis0Type_Int32, "int32", "1");
    setDeviceProperty(ie, vrDeviceId, Prop_Axis1Type_Int32, "int32", "3");
    setDeviceProperty(ie, vrDeviceId, Prop_Axis2Type_Int32, "int32", "0");
    setDeviceProperty(ie, vrDeviceId, Prop_Axis3Type_Int32, "int32", "0");
    setDeviceProperty(ie, vrDeviceId, Prop_Axis4Type_Int32, "int32", "0");
    setDeviceProperty(ie, vrDeviceId, Prop_ControllerRoleHint_Int32, "int32", "3");
    setDeviceProperty(ie, vrDeviceId, Prop_IconPathName_String, "string", "icons");
    setDeviceProperty(ie, vrDeviceId, Prop_NamedIconPathDeviceOff_String, "string", "{htc}controller_status_off.png");
    setDeviceProperty(ie, vrDeviceId, Prop_NamedIconPathDeviceSearching_String, "string", "{htc}controller_status_searching.gif");
    setDeviceProperty(ie, vrDeviceId, Prop_NamedIconPathDeviceSearchingAlert_String, "string", "{htc}controller_status_searching_alert.gif");
    setDeviceProperty(ie, vrDeviceId, Prop_NamedIconPathDeviceReady_String, "string", "{htc}controller_status_ready.png");
    setDeviceProperty(ie, vrDeviceId, Prop_NamedIconPathDeviceReadyAlert_String, "string", "{htc}controller_status_ready_alert.png");
    setDeviceProperty(ie, vrDeviceId, Prop_NamedIconPathDeviceNotReady_String, "string", "{htc}controller_status_error.png");
    setDeviceProperty(ie, vrDeviceId, Prop_NamedIconPathDeviceStandby_String, "string", "{htc}controller_status_standby.png");
    setDeviceProperty(ie, vrDeviceId, Prop_NamedIconPathDeviceAlertLow_String, "string", "{htc}controller_status_ready_low.png");
    setDeviceProperty(ie, vrDeviceId, Prop_ControllerType_String, "string", "kinect_device");
    static bool test_trackerProperties = true;
    if (test_trackerProperties) {
        // Debug for the purposes of testing if the new input system actually solved the tracker bug
        setDeviceProperty(ie, vrDeviceId, Prop_DeviceClass_Int32, "int32", "3");
        removeDeviceProperty(ie, vrDeviceId, Prop_ControllerRoleHint_Int32);
        setDeviceProperty(ie, vrDeviceId, Prop_InputProfilePath_String, "string", KVR::inputDirForOpenVR("kinect_device_profile.json"));
    }
}
void setDeviceProperty(vrinputemulator::VRInputEmulator &ie, uint32_t deviceId, int dProp, std::string type, std::string value) {
    vr::ETrackedDeviceProperty deviceProperty = (vr::ETrackedDeviceProperty)dProp;
    if (std::strcmp(type.c_str(), "int32") == 0) {
        ie.setVirtualDeviceProperty(deviceId, deviceProperty, (int32_t)std::atoi(value.c_str()));
    }
    else if (std::strcmp(type.c_str(), "uint64") == 0) {
        ie.setVirtualDeviceProperty(deviceId, deviceProperty, (uint64_t)std::atoll(value.c_str()));
    }
    else if (std::strcmp(type.c_str(), "float") == 0) {
        ie.setVirtualDeviceProperty(deviceId, deviceProperty, (float)std::atof(value.c_str()));
    }
    else if (std::strcmp(type.c_str(), "bool") == 0) {
        ie.setVirtualDeviceProperty(deviceId, deviceProperty, std::atoi(value.c_str()) != 0);
    }
    else if (std::strcmp(type.c_str(), "string") == 0) {
        ie.setVirtualDeviceProperty(deviceId, deviceProperty, value.c_str());
    }
    else {
        throw std::runtime_error("Unknown value type.");
    }
}
void removeAllTrackerProperties(vrinputemulator::VRInputEmulator &ie, uint32_t &vrDeviceId) {
    using namespace vr;
    removeDeviceProperty(ie, vrDeviceId, Prop_TrackingSystemName_String);
    removeDeviceProperty(ie, vrDeviceId, Prop_ModelNumber_String);
    removeDeviceProperty(ie, vrDeviceId, Prop_RenderModelName_String);
    removeDeviceProperty(ie, vrDeviceId, Prop_WillDriftInYaw_Bool);
    removeDeviceProperty(ie, vrDeviceId, Prop_ManufacturerName_String);
    removeDeviceProperty(ie, vrDeviceId, Prop_TrackingFirmwareVersion_String);
    removeDeviceProperty(ie, vrDeviceId, Prop_HardwareRevision_String);
    removeDeviceProperty(ie, vrDeviceId, Prop_DeviceIsWireless_Bool);
    removeDeviceProperty(ie, vrDeviceId, Prop_HardwareRevision_Uint64);
    removeDeviceProperty(ie, vrDeviceId, Prop_FirmwareVersion_Uint64);
    removeDeviceProperty(ie, vrDeviceId, Prop_DeviceClass_Int32);
    removeDeviceProperty(ie, vrDeviceId, Prop_SupportedButtons_Uint64);
    removeDeviceProperty(ie, vrDeviceId, Prop_Axis0Type_Int32);
    removeDeviceProperty(ie, vrDeviceId, Prop_Axis1Type_Int32);
    removeDeviceProperty(ie, vrDeviceId, Prop_Axis2Type_Int32);
    removeDeviceProperty(ie, vrDeviceId, Prop_Axis3Type_Int32);
    removeDeviceProperty(ie, vrDeviceId, Prop_Axis4Type_Int32);
    removeDeviceProperty(ie, vrDeviceId, Prop_ControllerRoleHint_Int32);
    removeDeviceProperty(ie, vrDeviceId, Prop_IconPathName_String);
    removeDeviceProperty(ie, vrDeviceId, Prop_NamedIconPathDeviceOff_String);
    removeDeviceProperty(ie, vrDeviceId, Prop_NamedIconPathDeviceSearching_String);
    removeDeviceProperty(ie, vrDeviceId, Prop_NamedIconPathDeviceSearchingAlert_String);
    removeDeviceProperty(ie, vrDeviceId, Prop_NamedIconPathDeviceReady_String);
    removeDeviceProperty(ie, vrDeviceId, Prop_NamedIconPathDeviceReadyAlert_String);
    removeDeviceProperty(ie, vrDeviceId, Prop_NamedIconPathDeviceNotReady_String);
    removeDeviceProperty(ie, vrDeviceId, Prop_NamedIconPathDeviceStandby_String);
    removeDeviceProperty(ie, vrDeviceId, Prop_NamedIconPathDeviceAlertLow_String);
    removeDeviceProperty(ie, vrDeviceId, Prop_ControllerType_String);
}
void removeDeviceProperty(vrinputemulator::VRInputEmulator &ie, uint32_t vrDeviceId, int dProp) {
    vr::ETrackedDeviceProperty deviceProperty = (vr::ETrackedDeviceProperty)dProp;

    ie.removeVirtualDeviceProperty(vrDeviceId, deviceProperty);
}
void destroyTracker(vrinputemulator::VRInputEmulator& inputEmulator, uint32_t ID) {
    std::cerr << "DESTROYING TRACKER " << ID << "!!!\n";
    auto pose = inputEmulator.getVirtualDevicePose(ID);
    if (pose.deviceIsConnected) {
        pose.deviceIsConnected = false;
        pose.poseIsValid = false;
        inputEmulator.setVirtualDevicePose(ID, pose);
    }
}
void setKinectTrackerProperties(vrinputemulator::VRInputEmulator &ie, uint32_t deviceId) {
    setDeviceProperty(ie, deviceId, vr::Prop_RenderModelName_String, "string", "arrow");
    setDeviceProperty(ie, deviceId, vr::Prop_DeviceClass_Int32, "int32", "4");
    //setDeviceProperty(ie, deviceId, vr::Prop_ControllerRoleHint_Int32, "int32", "0");
    //removeDeviceProperty(ie, deviceId, vr::Prop_ControllerType_String);
    //removeDeviceProperty(ie, deviceId, vr::Prop_InputProfilePath_String);
}
