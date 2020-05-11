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
#include <boost/lexical_cast.hpp>
#include <eigen-3.3.7/Eigen/Geometry>
#include "kalman/kalman.hpp"
#include <vector>
#include <fstream>

vr::DriverPose_t FakeTracker::dlpose;
vr::DriverPose_t dlposeh;
vr::DriverPose_t dlposem;
vr::DriverPose_t dlposep;

using namespace Eigen;

std::chrono::milliseconds FakeTracker::dlpose_timestamp;
void Log(const std::string& text) {
	std::ofstream log("C:\\log.txt", std::ofstream::app | std::ofstream::out);
	log << text << std::endl;
}

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

	std::clock_t current_ticks, delta_ticks;
	clock_t fps = 0;
	
	while (1) {
		current_ticks = clock();

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

		if (pthis->dest == "LFOOT") {
			Quaternionf rothconv = AngleAxisf(vhrotx, Vector3f::UnitX())
				* AngleAxisf(vhroty, Vector3f::UnitY())
				* AngleAxisf(vhrotz, Vector3f::UnitZ());
			vr::HmdQuaternion_t hquat = { rothconv.w(), rothconv.x(), rothconv.y(), rothconv.z() };
			dlposeh.vecPosition[0] += (float)nstr(OfS, "HOX") / (float)10000;
			dlposeh.vecPosition[1] += (float)nstr(OfS, "HOY") / (float)10000;
			dlposeh.vecPosition[2] += (float)nstr(OfS, "HOZ") / (float)10000;
			dlposeh.qRotation = hquat;
		}
		else if (pthis->dest == "RFOOT") {
			Quaternionf rotmconv = AngleAxisf(vmrotx, Vector3f::UnitX())
				* AngleAxisf(vmroty, Vector3f::UnitY())
				* AngleAxisf(vmrotz, Vector3f::UnitZ());
			vr::HmdQuaternion_t mquat = { rotmconv.w(), rotmconv.x(), rotmconv.y(), rotmconv.z() };
			dlposem.vecPosition[0] += (float)nstr(OfS, "MOX") / (float)10000;
			dlposem.vecPosition[1] += (float)nstr(OfS, "MOY") / (float)10000;
			dlposem.vecPosition[2] += (float)nstr(OfS, "MOZ") / (float)10000;
			dlposem.qRotation = mquat;
		}
		else if (pthis->dest == "HIP") {
			Quaternionf rotpconv = AngleAxisf(vprotx, Vector3f::UnitX())
				* AngleAxisf(vproty, Vector3f::UnitY())
				* AngleAxisf(vprotz, Vector3f::UnitZ());
			vr::HmdQuaternion_t pquat = { rotpconv.w(), rotpconv.x(), rotpconv.y(), rotpconv.z() };
			dlposep.vecPosition[0] += (float)nstr(OfS, "POX") / (float)10000;
			dlposep.vecPosition[1] += (float)nstr(OfS, "POY") / (float)10000;
			dlposep.vecPosition[2] += (float)nstr(OfS, "POZ") / (float)10000;
			dlposep.qRotation = pquat;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(3));

		delta_ticks = clock() - current_ticks;
		if (delta_ticks > 0)
			fps = CLOCKS_PER_SEC / delta_ticks;
		//Log(boost::lexical_cast<std::string>(fps));
	}
}

void FakeTracker::poseset(FakeTracker* pthis) {

	int n = 3; // Number of states
	int m = 1; // Number of measurements

	double dt = 1.0 / 30; // Time step

	Eigen::MatrixXd A(n, n); // System dynamics matrix
	Eigen::MatrixXd C(m, n); // Output matrix
	Eigen::MatrixXd Q(n, n); // Process noise covariance
	Eigen::MatrixXd R(m, m); // Measurement noise covariance
	Eigen::MatrixXd P(n, n); // Estimate error covariance

	// Discrete LTI projectile motion, measuring position only
	A << 1, dt, 0, 0, 1, dt, 0, 0, 1;
	C << 1, 0, 0;

	// Reasonable covariance matrices
	Q << .05, .05, .0, .05, .05, .0, .0, .0, .0;
	R << 5;
	P << .1, .1, .1, .1, 10000, 10, .1, 10, 100;

	// Construct the filter
	KalmanFilter posef[3][3] = { 
		{ KalmanFilter(dt, A, C, Q, R, P), KalmanFilter(dt, A, C, Q, R, P), KalmanFilter(dt, A, C, Q, R, P) },
		{ KalmanFilter(dt, A, C, Q, R, P), KalmanFilter(dt, A, C, Q, R, P), KalmanFilter(dt, A, C, Q, R, P) },
		{ KalmanFilter(dt, A, C, Q, R, P), KalmanFilter(dt, A, C, Q, R, P), KalmanFilter(dt, A, C, Q, R, P) } };

	Eigen::VectorXd x0(n);
	x0 << 0.f, 0.f, 0.f;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			posef[i][j].init(0.f, x0);
		}
	}

	double t[3][3] = { { 0,0,0 }, { 0,0,0 }, { 0,0,0 } };
	Eigen::VectorXd y[3][3] = { 
		{ Eigen::VectorXd(m), Eigen::VectorXd(m), Eigen::VectorXd(m) },
		{ Eigen::VectorXd(m), Eigen::VectorXd(m), Eigen::VectorXd(m) },
		{ Eigen::VectorXd(m), Eigen::VectorXd(m), Eigen::VectorXd(m) } };

	while (1) {

		if (pthis->dest == "LFOOT") {
			vr::DriverPose_t tmpdlposeh = dlposeh;

			for (int i = 0; i < 3; i++) {
				t[0][i] += dt;
				y[0][i] << tmpdlposeh.vecPosition[i];
				posef[0][i].update(y[0][i]);
				tmpdlposeh.vecPosition[i] = posef[0][i].state().x();
			}

			vr::VRServerDriverHost()->TrackedDevicePoseUpdated(pthis->_index, tmpdlposeh, sizeof(vr::DriverPose_t));

		}
		else if (pthis->dest == "RFOOT") {
			vr::DriverPose_t tmpdlposem = dlposem;

			for (int i = 0; i < 3; i++) {
				t[1][i] += dt;
				y[1][i] << tmpdlposem.vecPosition[i];
				posef[1][i].update(y[1][i]);
				tmpdlposem.vecPosition[i] = posef[1][i].state().x();
			}

			vr::VRServerDriverHost()->TrackedDevicePoseUpdated(pthis->_index, tmpdlposem, sizeof(vr::DriverPose_t));

		}
		else if (pthis->dest == "HIP") {
			vr::DriverPose_t tmpdlposep = dlposep;

			for (int i = 0; i < 3; i++) {
				t[2][i] += dt;
				y[2][i] << tmpdlposep.vecPosition[i];
				posef[2][i].update(y[2][i]);
				tmpdlposep.vecPosition[i] = posef[2][i].state().x();
			}

			vr::VRServerDriverHost()->TrackedDevicePoseUpdated(pthis->_index, tmpdlposep, sizeof(vr::DriverPose_t));

		}

		std::this_thread::sleep_for(std::chrono::milliseconds(5));

	}
}

void FakeTracker::update()
{
	/*     _
	   .__(.)< (MEOW)
		\___)
	~~~~~~~~~~~~~~~~~~
	*/
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
	std::thread* t2 = new std::thread(poseset, this);

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

	std::ofstream log("C:\\log.txt");
	Log("KinecToVR tracker Activated! Objectt SVR ID: " + boost::lexical_cast<std::string>(_index));
	Log("SteamVR tracker serial number:  " + boost::lexical_cast<std::string>(_serial));
	Log("SteamVR tracker identification number:  " + boost::lexical_cast<std::string>("/devices/KinectToVR/" + _serial));

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

