#include "stdafx.h"
#include "IETracker.h"
#include <iostream>

uint32_t initTracker(vrinputemulator::VRInputEmulator &inputEmulator, bool connected) {
    uint32_t deviceId = inputEmulator.getVirtualDeviceCount();
    inputEmulator.addVirtualDevice(vrinputemulator::VirtualDeviceType::TrackedController, std::to_string(deviceId), false);
    setTrackerDefaultProperties(deviceId);
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

void setTrackerDefaultProperties(uint32_t &deviceId) {
    setDeviceProperty(deviceId, 1000, "string", "lighthouse");
    setDeviceProperty(deviceId, 1001, "string", "Vive Controller MV");
    setDeviceProperty(deviceId, 1003, "string", "vr_controller_vive_1_5");
    setDeviceProperty(deviceId, 1004, "bool", "0");
    setDeviceProperty(deviceId, 1005, "string", "HTC");
    setDeviceProperty(deviceId, 1006, "string", "1465809478 htcvrsoftware@firmware-win32 2016-06-13 FPGA 1.6/0/0 VRC 1465809477 Radio 1466630404");
    setDeviceProperty(deviceId, 1007, "string", "product 129 rev 1.5.0 lot 2000/0/0 0");
    setDeviceProperty(deviceId, 1010, "bool", "1");
    setDeviceProperty(deviceId, 1017, "uint64", "2164327680");
    setDeviceProperty(deviceId, 1018, "uint64", "1465809478");
    setDeviceProperty(deviceId, 1029, "int32", "3");
    setDeviceProperty(deviceId, 3001, "uint64", "12884901895");
    setDeviceProperty(deviceId, 3002, "int32", "1");
    setDeviceProperty(deviceId, 3003, "int32", "3");
    setDeviceProperty(deviceId, 3004, "int32", "0");
    setDeviceProperty(deviceId, 3005, "int32", "0");
    setDeviceProperty(deviceId, 3006, "int32", "0");
    setDeviceProperty(deviceId, 3007, "int32", "0");
    setDeviceProperty(deviceId, 5000, "string", "icons");
    setDeviceProperty(deviceId, 5001, "string", "{htc}controller_status_off.png");
    setDeviceProperty(deviceId, 5002, "string", "{htc}controller_status_searching.gif");
    setDeviceProperty(deviceId, 5003, "string", "{htc}controller_status_searching_alert.gif");
    setDeviceProperty(deviceId, 5004, "string", "{htc}controller_status_ready.png");
    setDeviceProperty(deviceId, 5005, "string", "{htc}controller_status_ready_alert.png");
    setDeviceProperty(deviceId, 5006, "string", "{htc}controller_status_error.png");
    setDeviceProperty(deviceId, 5007, "string", "{htc}controller_status_standby.png");
    setDeviceProperty(deviceId, 5008, "string", "{htc}controller_status_ready_low.png");
}
void setDeviceProperty(uint32_t deviceId, int dProp, std::string type, std::string value) {
    vr::ETrackedDeviceProperty deviceProperty = (vr::ETrackedDeviceProperty)dProp;
    vrinputemulator::VRInputEmulator inputEmulator;
    inputEmulator.connect();
    if (std::strcmp(type.c_str(), "int32") == 0) {
        inputEmulator.setVirtualDeviceProperty(deviceId, deviceProperty, (int32_t)std::atoi(value.c_str()));
    }
    else if (std::strcmp(type.c_str(), "uint64") == 0) {
        inputEmulator.setVirtualDeviceProperty(deviceId, deviceProperty, (uint64_t)std::atoll(value.c_str()));
    }
    else if (std::strcmp(type.c_str(), "float") == 0) {
        inputEmulator.setVirtualDeviceProperty(deviceId, deviceProperty, (float)std::atof(value.c_str()));
    }
    else if (std::strcmp(type.c_str(), "bool") == 0) {
        inputEmulator.setVirtualDeviceProperty(deviceId, deviceProperty, std::atoi(value.c_str()) != 0);
    }
    else if (std::strcmp(type.c_str(), "string") == 0) {
        inputEmulator.setVirtualDeviceProperty(deviceId, deviceProperty, value.c_str());
    }
    else {
        throw std::runtime_error("Unknown value type.");
    }
}
void removeAllTrackerProperties(uint32_t &deviceId) {
    removeDeviceProperty(deviceId, 1000, "string", "lighthouse");
    removeDeviceProperty(deviceId, 1001, "string", "Vive Controller MV");
    removeDeviceProperty(deviceId, 1003, "string", "vr_controller_vive_1_5");
    removeDeviceProperty(deviceId, 1004, "bool", "0");
    removeDeviceProperty(deviceId, 1005, "string", "HTC");
    removeDeviceProperty(deviceId, 1006, "string", "1465809478 htcvrsoftware@firmware-win32 2016-06-13 FPGA 1.6/0/0 VRC 1465809477 Radio 1466630404");
    removeDeviceProperty(deviceId, 1007, "string", "product 129 rev 1.5.0 lot 2000/0/0 0");
    removeDeviceProperty(deviceId, 1010, "bool", "1");
    removeDeviceProperty(deviceId, 1017, "uint64", "2164327680");
    removeDeviceProperty(deviceId, 1018, "uint64", "1465809478");
    removeDeviceProperty(deviceId, 1029, "int32", "3");
    removeDeviceProperty(deviceId, 3001, "uint64", "12884901895");
    removeDeviceProperty(deviceId, 3002, "int32", "1");
    removeDeviceProperty(deviceId, 3003, "int32", "3");
    removeDeviceProperty(deviceId, 3004, "int32", "0");
    removeDeviceProperty(deviceId, 3005, "int32", "0");
    removeDeviceProperty(deviceId, 3006, "int32", "0");
    removeDeviceProperty(deviceId, 3007, "int32", "0");
    removeDeviceProperty(deviceId, 5000, "string", "icons");
    removeDeviceProperty(deviceId, 5001, "string", "{htc}controller_status_off.png");
    removeDeviceProperty(deviceId, 5002, "string", "{htc}controller_status_searching.gif");
    removeDeviceProperty(deviceId, 5003, "string", "{htc}controller_status_searching_alert.gif");
    removeDeviceProperty(deviceId, 5004, "string", "{htc}controller_status_ready.png");
    removeDeviceProperty(deviceId, 5005, "string", "{htc}controller_status_ready_alert.png");
    removeDeviceProperty(deviceId, 5006, "string", "{htc}controller_status_error.png");
    removeDeviceProperty(deviceId, 5007, "string", "{htc}controller_status_standby.png");
    removeDeviceProperty(deviceId, 5008, "string", "{htc}controller_status_ready_low.png");
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
void setKinectTrackerProperties(uint32_t deviceId) {
    setDeviceProperty(deviceId, 1003, "string", "arrow");
    setDeviceProperty(deviceId, 1029, "int32", "4");
}
