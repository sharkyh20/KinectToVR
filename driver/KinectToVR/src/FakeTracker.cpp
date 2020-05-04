#include "FakeTracker.hpp"
#include <iostream>
#include <string>
#include <sstream>
#include <Windows.h>
#include <thread>
#include <glm/vec3.hpp>
#include <glm/gtc/quaternion.hpp> 
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/glm.hpp>
#include <eigen-3.3.7/Eigen/Geometry>

vr::DriverPose_t FakeTracker::dlpose;
vr::DriverPose_t dlposeh;
vr::DriverPose_t dlposem;
vr::DriverPose_t dlposep;
using namespace Eigen;

std::chrono::milliseconds FakeTracker::dlpose_timestamp;

FakeTracker::FakeTracker(std::string argv) : 
	_pose( {0} )
{
	// Create some random but unique serial
	_serial = "Puck_" + argv;
	dest = argv;

	// Set up some defalt rotation pointing down -z
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
	_pose.result = vr::ETrackingResult::TrackingResult_Running_OK;
	_pose.deviceIsConnected = true;


	dlposeh = { 0 };
	dlposeh.poseIsValid = true;
	dlposeh.result = vr::TrackingResult_Running_OK;
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

	dlposem = { 0 };
	dlposem.poseIsValid = true;
	dlposem.result = vr::TrackingResult_Running_OK;
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

	dlposep = { 0 };
	dlposep.poseIsValid = true;
	dlposep.result = vr::TrackingResult_Running_OK;
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

std::shared_ptr<FakeTracker> FakeTracker::make_new(std::string destination)
{
	return std::shared_ptr<FakeTracker>(new FakeTracker(destination));
}

std::string FakeTracker::get_serial() const
{
	return _serial;
}

std::string ExtractString(std::string source, std::string start, std::string end)
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
int nstr(std::string s, const char* whatc) {
	s = ExtractString(std::string(s), whatc, "/");
	std::stringstream str_strm;
	str_strm << s;
	std::string temp_str;
	int temp_int;
	while (!str_strm.eof()) {
		str_strm >> temp_str;
		if (std::stringstream(temp_str) >> temp_int) {
			return temp_int;
		}
		temp_str = "";
	}
}

void FakeTracker::pipedl(FakeTracker* pthis) {
	while (1) {

		//// Update time delta (for working out velocity)
		//std::chrono::milliseconds time_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
		//double time_since_epoch_seconds = time_since_epoch.count() / 1000.0;
		//double pose_time_delta_seconds = (time_since_epoch - dlpose_timestamp).count() / 1000.0;

		//// Update pose timestamp
		//dlpose_timestamp = time_since_epoch;

		//// Copy the previous position data
		//double previous_position[3] = { 0 };
		//std::copy(std::begin(dlpose.vecPosition), std::end(dlpose.vecPosition), std::begin(previous_position));

		//// Update the position with our new data
		//dlpose.vecPosition[0] = 0.5 * std::sin(time_since_epoch_seconds);
		//dlpose.vecPosition[1] = 1 + 0.5 * std::cos(time_since_epoch_seconds);;
		//dlpose.vecPosition[2] = -2;// +2 * std::cos(time_since_epoch_seconds);


		// Update the velocity
		//_pose.vecVelocity[0] = (_pose.vecPosition[0] - previous_position[0]) / pose_time_delta_seconds;
		//_pose.vecVelocity[1] = (_pose.vecPosition[1] - previous_position[1]) / pose_time_delta_seconds;
		//_pose.vecVelocity[2] = (_pose.vecPosition[2] - previous_position[2]) / pose_time_delta_seconds;


		HANDLE pipeOf = CreateNamedPipe(TEXT("\\\\.\\pipe\\LogPipeTracker"), PIPE_ACCESS_INBOUND | PIPE_ACCESS_OUTBOUND, PIPE_TYPE_MESSAGE | PIPE_READMODE_MESSAGE, 1, 1024, 1024, 120 * 1000, NULL);
		

		char OfD[1024];
		DWORD Of = DWORD();

		ConnectNamedPipe(pipeOf, NULL);
		ReadFile(pipeOf, OfD, 1024, &Of, NULL);

		//DisconnectNamedPipe(pipeOf);
		CloseHandle(pipeOf);

		std::string OfS = OfD;
		float rof = (float)nstr(OfS, "WRW") / (float)10000;

		dlposeh.vecPosition[0] = (float)nstr(OfS, "HX") / (float)10000;
		dlposeh.vecPosition[1] = (float)nstr(OfS, "HY") / (float)10000;
		dlposeh.vecPosition[2] = (float)nstr(OfS, "HZ") / (float)10000;
		dlposem.vecPosition[0] = (float)nstr(OfS, "MX") / (float)10000;
		dlposem.vecPosition[1] = (float)nstr(OfS, "MY") / (float)10000;
		dlposem.vecPosition[2] = (float)nstr(OfS, "MZ") / (float)10000;
		dlposep.vecPosition[0] = (float)nstr(OfS, "PX") / (float)10000;
		dlposep.vecPosition[1] = (float)nstr(OfS, "PY") / (float)10000;
		dlposep.vecPosition[2] = (float)nstr(OfS, "PZ") / (float)10000;

		float vhrotx = (float)nstr(OfS, "HRX") / (float)10000;
		float vhroty = (float)nstr(OfS, "HRY") / (float)10000;
		float vhrotz = (float)nstr(OfS, "HRZ") / (float)10000;
		float vmrotx = (float)nstr(OfS, "MRX") / (float)10000;
		float vmroty = (float)nstr(OfS, "MRY") / (float)10000;
		float vmrotz = (float)nstr(OfS, "MRZ") / (float)10000;
		float vprotx = (float)nstr(OfS, "PRX") / (float)10000;
		float vproty = (float)nstr(OfS, "PRY") / (float)10000;
		float vprotz = (float)nstr(OfS, "PRZ") / (float)10000;

		Quaternionf rothconv, rotmconv, rotpconv;
		rothconv = AngleAxisf(vhrotx, Vector3f::UnitX())
			* AngleAxisf(vhroty, Vector3f::UnitY())
			* AngleAxisf(vhrotz, Vector3f::UnitZ());
		rotmconv = AngleAxisf(vmrotx, Vector3f::UnitX())
			* AngleAxisf(vmroty, Vector3f::UnitY())
			* AngleAxisf(vmrotz, Vector3f::UnitZ());
		rotpconv = AngleAxisf(vprotx, Vector3f::UnitX())
			* AngleAxisf(vproty, Vector3f::UnitY())
			* AngleAxisf(vprotz, Vector3f::UnitZ());

		vr::HmdQuaternion_t hquat = { rothconv.w(), rothconv.x(), rothconv.y(), rothconv.z() };
		vr::HmdQuaternion_t mquat = { rotmconv.w(), rotmconv.x(), rotmconv.y(), rotmconv.z() };
		vr::HmdQuaternion_t pquat = { rotpconv.w(), rotpconv.x(), rotpconv.y(), rotpconv.z() };

		if (dlposeh.vecPosition[0] != 0 && dlposeh.vecPosition[1] != 0 && dlposeh.vecPosition[2] != 0) {
			glm::vec3 hrt = glm::rotateY(glm::vec4(glm::vec3(dlposeh.vecPosition[0], dlposeh.vecPosition[1], dlposeh.vecPosition[2]), 1), glm::f32(glm::radians(rof)));
			dlposeh.vecPosition[0] = hrt.x + (float)nstr(OfS, "HOX") / (float)10000;
			dlposeh.vecPosition[1] = hrt.y + (float)nstr(OfS, "HOY") / (float)10000;
			dlposeh.vecPosition[2] = hrt.z + (float)nstr(OfS, "HOZ") / (float)10000;
			dlposeh.qRotation = hquat;
		}
		if (dlposem.vecPosition[0] != 0 && dlposem.vecPosition[1] != 0 && dlposem.vecPosition[2] != 0) {
			glm::vec3 mrt = glm::rotateY(glm::vec4(glm::vec3(dlposem.vecPosition[0], dlposem.vecPosition[1], dlposem.vecPosition[2]), 1), glm::f32(glm::radians(rof)));
			dlposem.vecPosition[0] = mrt.x + (float)nstr(OfS, "MOX") / (float)10000;
			dlposem.vecPosition[1] = mrt.y + (float)nstr(OfS, "MOY") / (float)10000;
			dlposem.vecPosition[2] = mrt.z + (float)nstr(OfS, "MOZ") / (float)10000;
			dlposem.qRotation = mquat;
		}
		if (dlposep.vecPosition[0] != 0 && dlposep.vecPosition[1] != 0 && dlposep.vecPosition[2] != 0) {
			glm::vec3 prt = glm::rotateY(glm::vec4(glm::vec3(dlposep.vecPosition[0], dlposep.vecPosition[1], dlposep.vecPosition[2]), 1), glm::f32(glm::radians(rof)));
			dlposep.vecPosition[0] = prt.x + (float)nstr(OfS, "POX") / (float)10000;
			dlposep.vecPosition[1] = prt.y + (float)nstr(OfS, "POY") / (float)10000;
			dlposep.vecPosition[2] = prt.z + (float)nstr(OfS, "POZ") / (float)10000;
			dlposep.qRotation = pquat;
		}
		
		std::this_thread::sleep_for(std::chrono::milliseconds(27));
	}
}

void FakeTracker::upr(FakeTracker* pthis) {
	while (1) {
		pthis->set_pose(dlposem);
		std::this_thread::sleep_for(std::chrono::milliseconds(31));
	}
}

void FakeTracker::upl(FakeTracker* pthis) {
	while (1) {
		pthis->set_pose(dlposeh);
		std::this_thread::sleep_for(std::chrono::milliseconds(31));
	}
}

void FakeTracker::uph(FakeTracker* pthis) {
	while (1) {
		pthis->set_pose(dlposep);
		std::this_thread::sleep_for(std::chrono::milliseconds(31));
	}
}

void FakeTracker::update()
{
	//// Update time delta (for working out velocity)
	//std::chrono::milliseconds time_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
	//double time_since_epoch_seconds = time_since_epoch.count() / 1000.0;
	//double pose_time_delta_seconds = (time_since_epoch - _pose_timestamp).count() / 1000.0;

	//// Update pose timestamp
	//_pose_timestamp = time_since_epoch;

	//// Copy the previous position data
	//double previous_position[3] = { 0 };
	//std::copy(std::begin(_pose.vecPosition), std::end(_pose.vecPosition), std::begin(previous_position));

	//// Update the position with our new data
	//_pose.vecPosition[0] = 0.5 * std::sin(time_since_epoch_seconds);
	//_pose.vecPosition[1] = 1 + 0.5 * std::cos(time_since_epoch_seconds);;
	//_pose.vecPosition[2] = -2;// +2 * std::cos(time_since_epoch_seconds);

	//// Update the velocity
	////_pose.vecVelocity[0] = (_pose.vecPosition[0] - previous_position[0]) / pose_time_delta_seconds;
	////_pose.vecVelocity[1] = (_pose.vecPosition[1] - previous_position[1]) / pose_time_delta_seconds;
	////_pose.vecVelocity[2] = (_pose.vecPosition[2] - previous_position[2]) / pose_time_delta_seconds;

	// If we are still tracking, update openvr with our new pose data
	if (_index != vr::k_unTrackedDeviceIndexInvalid)
	{
		vr::VRServerDriverHost()->TrackedDevicePoseUpdated(_index, _pose, sizeof(vr::DriverPose_t));
	}

}

vr::TrackedDeviceIndex_t FakeTracker::get_index() const
{
	return _index;
}

void FakeTracker::process_event(const vr::VREvent_t& event)
{
}

vr::EVRInitError FakeTracker::Activate(vr::TrackedDeviceIndex_t index)
{
	// Save the device index
	_index = index;
	std::thread* t1 = new std::thread(pipedl, this);

	if (dest == "LFOOT") {
		std::thread* th = new std::thread(upr, this);

	}
	else if (dest == "RFOOT") {
		std::thread* tm = new std::thread(upl, this);

	}
	else if (dest == "HIP") {
		std::thread* tp = new std::thread(uph, this);

	}

	
	// Get the properties handle for our controller
	_props = vr::VRProperties()->TrackedDeviceToPropertyContainer(_index);

	// Set our universe ID
	vr::VRProperties()->SetUint64Property(_props, vr::Prop_CurrentUniverseId_Uint64, 2);

	// Add our controller components. (These are the same as the regular vive controller)
	vr::VRDriverInput()->CreateBooleanComponent(_props, "/input/system/click", &_components._system_click);
	vr::VRDriverInput()->CreateHapticComponent(_props, "/output/haptic", &_components._haptic);

	//{htc}/icons/tracker_status_ready.b4bfb144.png

	// Set our controller to use the vive controller render model
	vr::VRProperties()->SetStringProperty(_props, vr::Prop_RenderModelName_String, "vr_controller_vive_1_5");// {htc} / rendermodels / vr_tracker_vive_1_0.obj");

	vr::VRProperties()->SetStringProperty(_props, vr::Prop_NamedIconPathDeviceOff_String, "{htc}/icons/tracker_status_off.b4bfb144.png");
	vr::VRProperties()->SetStringProperty(_props, vr::Prop_NamedIconPathDeviceSearching_String, "{htc}/icons/tracker_status_searching.b4bfb144.png");
	vr::VRProperties()->SetStringProperty(_props, vr::Prop_NamedIconPathDeviceSearchingAlert_String, "{htc}/icons/tracker_status_searching_alert.b4bfb144.png");
	vr::VRProperties()->SetStringProperty(_props, vr::Prop_NamedIconPathDeviceReadyAlert_String, "{htc}/icons/tracker_status_ready_alert.b4bfb144.png");
	vr::VRProperties()->SetStringProperty(_props, vr::Prop_NamedIconPathDeviceNotReady_String, "{htc}/icons/tracker_status_standby.b4bfb144.png");
	vr::VRProperties()->SetStringProperty(_props, vr::Prop_NamedIconPathDeviceStandby_String, "{htc}/icons/tracker_status_standby.b4bfb144.png");
	vr::VRProperties()->SetStringProperty(_props, vr::Prop_NamedIconPathDeviceAlertLow_String, "{htc}/icons/tracker_status_ready_low.b4bfb144.png");
	vr::VRProperties()->SetStringProperty(_props, vr::Prop_NamedIconPathDeviceReady_String, "{htc}/icons/tracker_status_ready.b4bfb144.png");
	
	if (dest == "LFOOT") {
		vr::VRSettings()->SetString(vr::k_pch_Trackers_Section, std::string("/devices/KinectToVR/" + _serial).c_str(), "TrackerRole_LeftFoot");
	}
	if (dest == "RFOOT") {
		vr::VRSettings()->SetString(vr::k_pch_Trackers_Section, std::string("/devices/KinectToVR/" + _serial).c_str(), "TrackerRole_RightFoot");
	}
	if (dest == "HIP") {
		vr::VRSettings()->SetString(vr::k_pch_Trackers_Section, std::string("/devices/KinectToVR/" + _serial).c_str(), "TrackerRole_Waist");
	}

	return vr::VRInitError_None;
}

void FakeTracker::Deactivate()
{
	// Clear device id
	_index = vr::k_unTrackedDeviceIndexInvalid;
}

void FakeTracker::EnterStandby()
{
}

void * FakeTracker::GetComponent(const char * component)
{
	// No extra components on this device so always return nullptr
	return nullptr;
}

void FakeTracker::DebugRequest(const char * request, char* response_buffer, uint32_t response_buffer_size)
{
	// No custom debug requests defined
	if (response_buffer_size >= 1)
		response_buffer[0] = 0;
}

vr::DriverPose_t FakeTracker::GetPose()
{
	return _pose;
}

std::string FakeTracker::GetDest() 
{
	return dest;
}

void FakeTracker::set_pose(vr::DriverPose_t new_pose)
{
	_pose = new_pose;
}

void FakeTracker::set_dest(std::string new_dest)
{
	dest = new_dest;
}
