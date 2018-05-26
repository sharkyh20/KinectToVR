#include "stdafx.h"
#include "IETracker.h"
#include <iostream>

uint32_t initTracker(vrinputemulator::VRInputEmulator &inputEmulator, bool connected) {
    uint32_t deviceId = inputEmulator.getVirtualDeviceCount();
    inputEmulator.addVirtualDevice(vrinputemulator::VirtualDeviceType::TrackedController, std::to_string(deviceId), false);
    setTrackerDefaultProperties(inputEmulator, deviceId);
    inputEmulator.publishVirtualDevice(deviceId);
    //Connect device
    auto pose = inputEmulator.getVirtualDevicePose(deviceId);
    if (pose.deviceIsConnected != connected) {
        pose.deviceIsConnected = connected;
        pose.poseIsValid = connected;
        inputEmulator.setVirtualDevicePose(deviceId, pose);
    }
    return deviceId;
}

void setTrackerDefaultProperties(vrinputemulator::VRInputEmulator &ie, uint32_t &deviceId) {
    using namespace vr;
    setDeviceProperty(ie, deviceId, Prop_TrackingSystemName_String, "string", "lighthouse");
    setDeviceProperty(ie, deviceId, Prop_ModelNumber_String, "string", "Vive Controller MV");
    setDeviceProperty(ie, deviceId, Prop_RenderModelName_String, "string", "vr_controller_vive_1_5");
    setDeviceProperty(ie, deviceId, Prop_WillDriftInYaw_Bool, "bool", "0");
    setDeviceProperty(ie, deviceId, Prop_ManufacturerName_String, "string", "HTC");
    setDeviceProperty(ie, deviceId, Prop_TrackingFirmwareVersion_String, "string", "1465809478 htcvrsoftware@firmware-win32 2016-06-13 FPGA 1.6/0/0 VRC 1465809477 Radio 1466630404");
    setDeviceProperty(ie, deviceId, Prop_HardwareRevision_String, "string", "product 129 rev 1.5.0 lot 2000/0/0 0");
    setDeviceProperty(ie, deviceId, Prop_DeviceIsWireless_Bool, "bool", "1");
    setDeviceProperty(ie, deviceId, Prop_HardwareRevision_Uint64, "uint64", "2164327680");
    setDeviceProperty(ie, deviceId, Prop_FirmwareVersion_Uint64, "uint64", "1465809478");
    setDeviceProperty(ie, deviceId, Prop_DeviceClass_Int32, "int32", "3");
    setDeviceProperty(ie, deviceId, Prop_SupportedButtons_Uint64, "uint64", "12884901895");
    setDeviceProperty(ie, deviceId, Prop_Axis0Type_Int32, "int32", "1");
    setDeviceProperty(ie, deviceId, Prop_Axis1Type_Int32, "int32", "3");
    setDeviceProperty(ie, deviceId, Prop_Axis2Type_Int32, "int32", "0");
    setDeviceProperty(ie, deviceId, Prop_Axis3Type_Int32, "int32", "0");
    setDeviceProperty(ie, deviceId, Prop_Axis4Type_Int32, "int32", "0");
    setDeviceProperty(ie, deviceId, Prop_ControllerRoleHint_Int32, "int32", "0");
    setDeviceProperty(ie, deviceId, Prop_IconPathName_String, "string", "icons");
    setDeviceProperty(ie, deviceId, Prop_NamedIconPathDeviceOff_String, "string", "{htc}controller_status_off.png");
    setDeviceProperty(ie, deviceId, Prop_NamedIconPathDeviceSearching_String, "string", "{htc}controller_status_searching.gif");
    setDeviceProperty(ie, deviceId, Prop_NamedIconPathDeviceSearchingAlert_String, "string", "{htc}controller_status_searching_alert.gif");
    setDeviceProperty(ie, deviceId, Prop_NamedIconPathDeviceReady_String, "string", "{htc}controller_status_ready.png");
    setDeviceProperty(ie, deviceId, Prop_NamedIconPathDeviceReadyAlert_String, "string", "{htc}controller_status_ready_alert.png");
    setDeviceProperty(ie, deviceId, Prop_NamedIconPathDeviceNotReady_String, "string", "{htc}controller_status_error.png");
    setDeviceProperty(ie, deviceId, Prop_NamedIconPathDeviceStandby_String, "string", "{htc}controller_status_standby.png");
    setDeviceProperty(ie, deviceId, Prop_NamedIconPathDeviceAlertLow_String, "string", "{htc}controller_status_ready_low.png");
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
void removeAllTrackerProperties(uint32_t &deviceId) {
    using namespace vr;
    removeDeviceProperty( deviceId, Prop_TrackingSystemName_String, "string", "lighthouse");
    removeDeviceProperty(deviceId, Prop_ModelNumber_String, "string", "Vive Controller MV");
    removeDeviceProperty(deviceId, Prop_RenderModelName_String, "string", "vr_controller_vive_1_5");
    removeDeviceProperty(deviceId, Prop_WillDriftInYaw_Bool, "bool", "0");
    removeDeviceProperty(deviceId, Prop_ManufacturerName_String, "string", "HTC");
    removeDeviceProperty(deviceId, Prop_TrackingFirmwareVersion_String, "string", "1465809478 htcvrsoftware@firmware-win32 2016-06-13 FPGA 1.6/0/0 VRC 1465809477 Radio 1466630404");
    removeDeviceProperty(deviceId, Prop_HardwareRevision_String, "string", "product 129 rev 1.5.0 lot 2000/0/0 0");
    removeDeviceProperty(deviceId, Prop_DeviceIsWireless_Bool, "bool", "1");
    removeDeviceProperty(deviceId, Prop_HardwareRevision_Uint64, "uint64", "2164327680");
    removeDeviceProperty(deviceId, Prop_FirmwareVersion_Uint64, "uint64", "1465809478");
    removeDeviceProperty(deviceId, Prop_DeviceClass_Int32, "int32", "3");
    removeDeviceProperty(deviceId, Prop_SupportedButtons_Uint64, "uint64", "12884901895");
    removeDeviceProperty(deviceId, Prop_Axis0Type_Int32, "int32", "1");
    removeDeviceProperty(deviceId, Prop_Axis1Type_Int32, "int32", "3");
    removeDeviceProperty(deviceId, Prop_Axis2Type_Int32, "int32", "0");
    removeDeviceProperty(deviceId, Prop_Axis3Type_Int32, "int32", "0");
    removeDeviceProperty(deviceId, Prop_Axis4Type_Int32, "int32", "0");
    removeDeviceProperty(deviceId, Prop_ControllerRoleHint_Int32, "int32", "0");
    removeDeviceProperty(deviceId, Prop_IconPathName_String, "string", "icons");
    removeDeviceProperty(deviceId, Prop_NamedIconPathDeviceOff_String, "string", "{htc}controller_status_off.png");
    removeDeviceProperty(deviceId, Prop_NamedIconPathDeviceSearching_String, "string", "{htc}controller_status_searching.gif");
    removeDeviceProperty(deviceId, Prop_NamedIconPathDeviceSearchingAlert_String, "string", "{htc}controller_status_searching_alert.gif");
    removeDeviceProperty(deviceId, Prop_NamedIconPathDeviceReady_String, "string", "{htc}controller_status_ready.png");
    removeDeviceProperty(deviceId, Prop_NamedIconPathDeviceReadyAlert_String, "string", "{htc}controller_status_ready_alert.png");
    removeDeviceProperty(deviceId, Prop_NamedIconPathDeviceNotReady_String, "string", "{htc}controller_status_error.png");
    removeDeviceProperty(deviceId, Prop_NamedIconPathDeviceStandby_String, "string", "{htc}controller_status_standby.png");
    removeDeviceProperty(deviceId, Prop_NamedIconPathDeviceAlertLow_String, "string", "{htc}controller_status_ready_low.png");
}
void removeDeviceProperty(uint32_t deviceId, int dProp, std::string type, std::string value) {
    vr::ETrackedDeviceProperty deviceProperty = (vr::ETrackedDeviceProperty)dProp;
    vrinputemulator::VRInputEmulator inputEmulator;
    inputEmulator.connect();
    if (std::strcmp(type.c_str(), "int32") == 0) {
        inputEmulator.removeVirtualDeviceProperty(deviceId, deviceProperty);
    }
    else if (std::strcmp(type.c_str(), "uint64") == 0) {
        inputEmulator.removeVirtualDeviceProperty(deviceId, deviceProperty);
    }
    else if (std::strcmp(type.c_str(), "float") == 0) {
        inputEmulator.removeVirtualDeviceProperty(deviceId, deviceProperty);
    }
    else if (std::strcmp(type.c_str(), "bool") == 0) {
        inputEmulator.removeVirtualDeviceProperty(deviceId, deviceProperty);
    }
    else if (std::strcmp(type.c_str(), "string") == 0) {
        inputEmulator.removeVirtualDeviceProperty(deviceId, deviceProperty);
    }
    else {
        throw std::runtime_error("Unknown value type.");
    }
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
    setDeviceProperty(ie, deviceId, 1003, "string", "arrow");
    setDeviceProperty(ie, deviceId, 1029, "int32", "4");
}
