#include "BodyTracker.h"
#include <iostream>
#include <string>
#include <sstream>
#include <Windows.h>
#include <thread>
#include <Eigen/Dense>
#include "soft_knuckles_device.h"

DriverPose_t BodyTracker::dlpose;
DriverPose_t dlposeh, dlposem, dlposep;

using namespace Eigen;

BodyTracker::BodyTracker(const std::string argv) :
	_pose({0})
{
	// Create some random but unique serial
	_serial = "Puck_" + argv;
	dest = argv;

	// Set up some default rotation pointing to -z
	_pose.qRotation.w = 1.0;
	_pose.qRotation.x = 0.0;
	_pose.qRotation.y = 0.0;
	_pose.qRotation.z = 0.0;

	_pose.qWorldFromDriverRotation.w = 1.0;
	_pose.qWorldFromDriverRotation.x = 0.0;
	_pose.qWorldFromDriverRotation.y = 0.0;
	_pose.qWorldFromDriverRotation.z = 0.0;

	_pose.qDriverFromHeadRotation.w = 1.0;
	_pose.qDriverFromHeadRotation.x = 0.0;
	_pose.qDriverFromHeadRotation.y = 0.0;
	_pose.qDriverFromHeadRotation.z = 0.0;

	// To ensure no complaints about tracking
	_pose.poseIsValid = true;
	_pose.result = TrackingResult_Running_OK;
	_pose.deviceIsConnected = true;


	dlposeh = {0};
	dlposeh.poseIsValid = true;
	dlposeh.result = TrackingResult_Running_OK;
	dlposeh.deviceIsConnected = true;
	dlposeh.qWorldFromDriverRotation.w = 1;
	dlposeh.qWorldFromDriverRotation.x = 0;
	dlposeh.qWorldFromDriverRotation.y = 0;
	dlposeh.qWorldFromDriverRotation.z = 0;
	dlposeh.qDriverFromHeadRotation.w = 1;
	dlposeh.qDriverFromHeadRotation.x = 0;
	dlposeh.qDriverFromHeadRotation.y = 0;
	dlposeh.qDriverFromHeadRotation.z = 0;

	dlposeh.vecPosition[0] = 0;
	dlposeh.vecPosition[1] = 0;
	dlposeh.vecPosition[2] = 0;

	dlposem = {0};
	dlposem.poseIsValid = true;
	dlposem.result = TrackingResult_Running_OK;
	dlposem.deviceIsConnected = true;
	dlposem.qWorldFromDriverRotation.w = 1;
	dlposem.qWorldFromDriverRotation.x = 0;
	dlposem.qWorldFromDriverRotation.y = 0;
	dlposem.qWorldFromDriverRotation.z = 0;
	dlposem.qDriverFromHeadRotation.w = 1;
	dlposem.qDriverFromHeadRotation.x = 0;
	dlposem.qDriverFromHeadRotation.y = 0;
	dlposem.qDriverFromHeadRotation.z = 0;

	dlposem.vecPosition[0] = 0;
	dlposem.vecPosition[1] = 0;
	dlposem.vecPosition[2] = 0;

	dlposep = {0};
	dlposep.poseIsValid = true;
	dlposep.result = TrackingResult_Running_OK;
	dlposep.deviceIsConnected = true;
	dlposep.qWorldFromDriverRotation.w = 1;
	dlposep.qWorldFromDriverRotation.x = 0;
	dlposep.qWorldFromDriverRotation.y = 0;
	dlposep.qWorldFromDriverRotation.z = 0;
	dlposep.qDriverFromHeadRotation.w = 1;
	dlposep.qDriverFromHeadRotation.x = 0;
	dlposep.qDriverFromHeadRotation.y = 0;
	dlposep.qDriverFromHeadRotation.z = 0;

	dlposep.vecPosition[0] = 0;
	dlposep.vecPosition[1] = 0;
	dlposep.vecPosition[2] = 0;
}

std::shared_ptr<BodyTracker> BodyTracker::make_new(std::string destination)
{
	return std::make_shared<BodyTracker>(destination);
}

std::string BodyTracker::get_serial() const
{
	return _serial;
}

static std::string ExtractString(std::string source, std::string start, std::string end)
{
	std::size_t startIndex = source.find(start);
	if (startIndex == std::string::npos)
	{
		return std::string("0");
	}

	startIndex += start.length();
	std::string::size_type endIndex = source.find(end, startIndex);

	return source.substr(startIndex, endIndex - startIndex);
}

static int nstr(std::string s, const char* whatc)
{
	s = ExtractString(std::string(s), whatc, "/");
	std::stringstream str_strm;
	str_strm << s;
	std::string temp_str;
	int temp_int;
	while (!str_strm.eof())
	{
		str_strm >> temp_str;
		if (std::stringstream(temp_str) >> temp_int)
		{
			return temp_int;
		}
		temp_str = "";
	}
}

void BodyTracker::poseset(BodyTracker* pthis)
{
	while (true)
	{
		auto t1 = std::chrono::high_resolution_clock::now();

		if (pthis->dest == "LFOOT")
		{
			VRServerDriverHost()->TrackedDevicePoseUpdated(pthis->_index, soft_knuckles::hposet,
			                                               sizeof(soft_knuckles::hposet));
		}
		else if (pthis->dest == "RFOOT")
		{
			VRServerDriverHost()->TrackedDevicePoseUpdated(pthis->_index, soft_knuckles::mposet,
			                                               sizeof(soft_knuckles::mposet));
		}
		else if (pthis->dest == "HIP")
		{
			VRServerDriverHost()->TrackedDevicePoseUpdated(pthis->_index, soft_knuckles::pposet,
			                                               sizeof(soft_knuckles::pposet));
		}

		auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
			std::chrono::high_resolution_clock::now() - t1).count();
		if (duration <= 9000000.f)
		{
			std::this_thread::sleep_for(std::chrono::nanoseconds(9000000 - duration));
		}
	}
}

void BodyTracker::update()
{
}

TrackedDeviceIndex_t BodyTracker::get_index() const
{
	return _index;
}

void BodyTracker::process_event(const VREvent_t& event)
{
}

EVRInitError BodyTracker::Activate(TrackedDeviceIndex_t index)
{
	// Save the device index
	_index = index;
	std::thread* t2 = new std::thread(poseset, this);

	if (dest == "LFOOT")
	{
		VRSettings()->SetString(k_pch_Trackers_Section, std::string("/devices/KinectToVR/" + _serial).c_str(),
		                        "TrackerRole_LeftFoot");
		m_serial = "LHR-CB9AD1T2";
	}
	if (dest == "RFOOT")
	{
		VRSettings()->SetString(k_pch_Trackers_Section, std::string("/devices/KinectToVR/" + _serial).c_str(),
		                        "TrackerRole_RightFoot");
		m_serial = "LHR-CB1441A7";
	}
	if (dest == "HIP")
	{
		VRSettings()->SetString(k_pch_Trackers_Section, std::string("/devices/KinectToVR/" + _serial).c_str(),
		                        "TrackerRole_Waist");
		m_serial = "LHR-CB11ABEC";
	}

	// Get the properties handle for our controller
	_props = VRProperties()->TrackedDeviceToPropertyContainer(_index);

	// Set our universe ID
	VRProperties()->SetUint64Property(_props, Prop_CurrentUniverseId_Uint64, 2);

	// Add our controller components. (These are the same as the regular vive controller)
	VRDriverInput()->CreateBooleanComponent(_props, "/input/system/click", &_components._system_click);
	VRDriverInput()->CreateHapticComponent(_props, "/output/haptic", &_components._haptic);

	//**NEW!** Register device properties **NEW!**

	VRProperties()->SetStringProperty(_props, Prop_TrackingSystemName_String, "lighthouse");
	VRProperties()->SetStringProperty(_props, Prop_ModelNumber_String, "Vive Tracker Pro MV");
	VRProperties()->SetStringProperty(_props, Prop_SerialNumber_String, m_serial.c_str()); // Changed

	VRProperties()->SetStringProperty(_props, Prop_RenderModelName_String, "{htc}vr_tracker_vive_1_0");
	//VRProperties()->SetStringProperty(_props, Prop_RenderModelName_String, "vr_controller_vive_1_5");// {htc} / rendermodels / vr_tracker_vive_1_0.obj");

	VRProperties()->SetBoolProperty(_props, Prop_WillDriftInYaw_Bool, false);
	VRProperties()->SetStringProperty(_props, Prop_ManufacturerName_String, "HTC");
	VRProperties()->SetStringProperty(_props, Prop_TrackingFirmwareVersion_String,
	                                  "1541800000 RUNNER-WATCHMAN$runner-watchman@runner-watchman 2018-01-01 FPGA 512(2.56/0/0) BL 0 VRC 1541800000 Radio 1518800000");
	// Changed
	VRProperties()->SetStringProperty(_props, Prop_HardwareRevision_String, "product 128 rev 2.5.6 lot 2000/0/0 0");
	// Changed
	VRProperties()->SetStringProperty(_props, Prop_ConnectedWirelessDongle_String, "D0000BE000"); // Changed
	VRProperties()->SetBoolProperty(_props, Prop_DeviceIsWireless_Bool, true);
	VRProperties()->SetBoolProperty(_props, Prop_DeviceIsCharging_Bool, false);
	VRProperties()->SetFloatProperty(_props, Prop_DeviceBatteryPercentage_Float, 1.f); // Always charged

	HmdMatrix34_t l_transform = {-1.f, 0.f, 0.f, 0.f, 0.f, 0.f, -1.f, 0.f, 0.f, -1.f, 0.f, 0.f};
	VRProperties()->SetProperty(_props, Prop_StatusDisplayTransform_Matrix34, &l_transform, sizeof(HmdMatrix34_t),
	                            k_unHmdMatrix34PropertyTag);

	VRProperties()->SetBoolProperty(_props, Prop_Firmware_UpdateAvailable_Bool, false);
	VRProperties()->SetBoolProperty(_props, Prop_Firmware_ManualUpdate_Bool, false);
	VRProperties()->SetStringProperty(_props, Prop_Firmware_ManualUpdateURL_String,
	                                  "https://developer.valvesoftware.com/wiki/SteamVR/HowTo_Update_Firmware");
	VRProperties()->SetUint64Property(_props, Prop_HardwareRevision_Uint64, 2214720000); // Changed
	VRProperties()->SetUint64Property(_props, Prop_FirmwareVersion_Uint64, 1541800000); // Changed
	VRProperties()->SetUint64Property(_props, Prop_FPGAVersion_Uint64, 512); // Changed
	VRProperties()->SetUint64Property(_props, Prop_VRCVersion_Uint64, 1514800000); // Changed
	VRProperties()->SetUint64Property(_props, Prop_RadioVersion_Uint64, 1518800000); // Changed
	VRProperties()->SetUint64Property(_props, Prop_DongleVersion_Uint64, 8933539758);
	// Changed, based on Prop_ConnectedWirelessDongle_String above
	VRProperties()->SetBoolProperty(_props, Prop_DeviceProvidesBatteryStatus_Bool, true);
	VRProperties()->SetBoolProperty(_props, Prop_DeviceCanPowerOff_Bool, true);
	VRProperties()->SetStringProperty(_props, Prop_Firmware_ProgrammingTarget_String, m_serial.c_str());
	VRProperties()->SetInt32Property(_props, Prop_DeviceClass_Int32, TrackedDeviceClass_GenericTracker);
	VRProperties()->SetBoolProperty(_props, Prop_Firmware_ForceUpdateRequired_Bool, false);
	//VRProperties()->SetUint64Property(_props, Prop_ParentDriver_Uint64, 8589934597); // Strange value from dump
	VRProperties()->SetStringProperty(_props, Prop_ResourceRoot_String, "htc");

	std::string l_registeredType("htc/vive_tracker");
	l_registeredType.append(m_serial);
	VRProperties()->SetStringProperty(_props, Prop_RegisteredDeviceType_String, l_registeredType.c_str());
	VRProperties()->SetStringProperty(_props, Prop_InputProfilePath_String, "{htc}/input/vive_tracker_profile.json");
	VRProperties()->SetBoolProperty(_props, Prop_Identifiable_Bool, false);
	VRProperties()->SetBoolProperty(_props, Prop_Firmware_RemindUpdate_Bool, false);
	VRProperties()->SetInt32Property(_props, Prop_ControllerRoleHint_Int32, TrackedControllerRole_Invalid);
	VRProperties()->SetStringProperty(_props, Prop_ControllerType_String, "vive_tracker_handed");
	VRProperties()->SetInt32Property(_props, Prop_ControllerHandSelectionPriority_Int32, -1);

	VRProperties()->SetStringProperty(_props, Prop_NamedIconPathDeviceOff_String, "{htc}/icons/tracker_status_off.png");
	VRProperties()->SetStringProperty(_props, Prop_NamedIconPathDeviceSearching_String,
	                                  "{htc}/icons/tracker_status_searching.gif");
	VRProperties()->SetStringProperty(_props, Prop_NamedIconPathDeviceSearchingAlert_String,
	                                  "{htc}/icons/tracker_status_searching_alert.gif");
	VRProperties()->SetStringProperty(_props, Prop_NamedIconPathDeviceReady_String,
	                                  "{htc}/icons/tracker_status_ready.png");
	VRProperties()->SetStringProperty(_props, Prop_NamedIconPathDeviceReadyAlert_String,
	                                  "{htc}/icons/tracker_status_ready_alert.png");
	VRProperties()->SetStringProperty(_props, Prop_NamedIconPathDeviceNotReady_String,
	                                  "{htc}/icons/tracker_status_error.png");
	VRProperties()->SetStringProperty(_props, Prop_NamedIconPathDeviceStandby_String,
	                                  "{htc}/icons/tracker_status_standby.png");
	VRProperties()->SetStringProperty(_props, Prop_NamedIconPathDeviceAlertLow_String,
	                                  "{htc}/icons/tracker_status_ready_low.png");

	//VRProperties()->SetStringProperty(_props, Prop_NamedIconPathDeviceOff_String, "{KinectToVR}/icons/tracker_status_off.png");
	//VRProperties()->SetStringProperty(_props, Prop_NamedIconPathDeviceSearching_String, "{KinectToVR}/icons/tracker_status_searching.gif");
	//VRProperties()->SetStringProperty(_props, Prop_NamedIconPathDeviceSearchingAlert_String, "{KinectToVR}/icons/tracker_status_searching_alert.gif");
	//VRProperties()->SetStringProperty(_props, Prop_NamedIconPathDeviceReadyAlert_String, "{KinectToVR}/icons/tracker_status_ready_alert.png");
	//VRProperties()->SetStringProperty(_props, Prop_NamedIconPathDeviceNotReady_String, "{KinectToVR}/icons/tracker_status_standby.png");
	//VRProperties()->SetStringProperty(_props, Prop_NamedIconPathDeviceStandby_String, "{KinectToVR}/icons/tracker_status_standby.png");
	//VRProperties()->SetStringProperty(_props, Prop_NamedIconPathDeviceAlertLow_String, "{KinectToVR}/icons/tracker_status_ready_low.png");
	//VRProperties()->SetStringProperty(_props, Prop_NamedIconPathDeviceReady_String, "{KinectToVR}/icons/tracker_status_ready.png");

	VRProperties()->SetBoolProperty(_props, Prop_HasDisplayComponent_Bool, false);
	VRProperties()->SetBoolProperty(_props, Prop_HasCameraComponent_Bool, false);
	VRProperties()->SetBoolProperty(_props, Prop_HasDriverDirectModeComponent_Bool, false);
	VRProperties()->SetBoolProperty(_props, Prop_HasVirtualDisplayComponent_Bool, false);

	return VRInitError_None;
}

void BodyTracker::Deactivate()
{
	// Clear device id
	_index = k_unTrackedDeviceIndexInvalid;
}

void BodyTracker::EnterStandby()
{
}

void* BodyTracker::GetComponent(const char* component)
{
	// No extra components on this device so always return nullptr
	return nullptr;
}

void BodyTracker::DebugRequest(const char* request, char* response_buffer, uint32_t response_buffer_size)
{
	// No custom debug requests defined
	if (response_buffer_size >= 1)
		response_buffer[0] = 0;
}

DriverPose_t BodyTracker::GetPose()
{
	return _pose;
}

std::string BodyTracker::GetDest()
{
	return dest;
}

void BodyTracker::set_pose(DriverPose_t new_pose)
{
	_pose = new_pose;
}

void BodyTracker::set_dest(std::string new_dest)
{
	dest = new_dest;
}
