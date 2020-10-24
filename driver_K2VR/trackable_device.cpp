#include "trackable_device.h"
#include "logger.h"

namespace soft_knuckles
{
	TrackableDevice::TrackableDevice()
		: m_ulPropertyContainer(vr::k_ulInvalidPropertyContainer)
		  , m_unSteamVRTrackedDeviceId(vr::k_unTrackedDeviceIndexInvalid)
	{
		memset(&m_Pose, 0, sizeof(m_Pose));
		m_Pose.result = vr::TrackingResult_Uninitialized;

		// By default, assume that the tracked devices are in the tracking space as OpenVR
		m_Pose.qWorldFromDriverRotation.w = 1.f;
		m_Pose.qWorldFromDriverRotation.x = 0.f;
		m_Pose.qWorldFromDriverRotation.y = 0.f;
		m_Pose.qWorldFromDriverRotation.z = 0.f;
		m_Pose.vecWorldFromDriverTranslation[0] = 0.f;
		m_Pose.vecWorldFromDriverTranslation[1] = 0.f;
		m_Pose.vecWorldFromDriverTranslation[2] = 0.f;

		m_firmware_revision = 0x0001;
		m_hardware_revision = 0x0001;
	}

	TrackableDevice::~TrackableDevice()
	{
	}

	// Shared Implementation of vr::ITrackedDeviceServerDriver
	vr::EVRInitError TrackableDevice::Activate(vr::TrackedDeviceIndex_t unObjectId)
	{
		vr::CVRPropertyHelpers* properties = vr::VRProperties();

		Logger::Info("CPSMoveTrackedDeviceLatest::Activate: %s is object id %d\n", GetSteamVRIdentifier(), unObjectId);
		m_ulPropertyContainer = properties->TrackedDeviceToPropertyContainer(unObjectId);
		m_unSteamVRTrackedDeviceId = unObjectId;

		properties->SetBoolProperty(m_ulPropertyContainer, vr::Prop_Firmware_UpdateAvailable_Bool, false);
		properties->SetBoolProperty(m_ulPropertyContainer, vr::Prop_Firmware_ManualUpdate_Bool, false);
		properties->SetBoolProperty(m_ulPropertyContainer, vr::Prop_ContainsProximitySensor_Bool, false);
		properties->SetBoolProperty(m_ulPropertyContainer, vr::Prop_HasCamera_Bool, false);
		properties->SetBoolProperty(m_ulPropertyContainer, vr::Prop_Firmware_ForceUpdateRequired_Bool, false);
		properties->SetBoolProperty(m_ulPropertyContainer, vr::Prop_DeviceCanPowerOff_Bool, false);
		//properties->SetUint64Property(m_ulPropertyContainer, vr::Prop_HardwareRevision_Uint64, m_hardware_revision);
		//properties->SetUint64Property(m_ulPropertyContainer, vr::Prop_FirmwareVersion_Uint64, m_firmware_revision);

		//if (CServerDriver_PSMoveService::getInstance()->IsHMDTrackingSpaceCalibrated())
		//{
		//	RefreshWorldFromDriverPose();
		//}

		return vr::VRInitError_None;
	}

	void TrackableDevice::Deactivate()
	{
		Logger::Info("CPSMoveTrackedDeviceLatest::Deactivate: %s was object id %d\n", GetSteamVRIdentifier(),
		             m_unSteamVRTrackedDeviceId);
		m_unSteamVRTrackedDeviceId = vr::k_unTrackedDeviceIndexInvalid;
	}

	void TrackableDevice::EnterStandby()
	{
		//###HipsterSloth $TODO - No good way to do this at the moment
	}

	void* TrackableDevice::GetComponent(const char* pchComponentNameAndVersion)
	{
		return nullptr;
	}

	void TrackableDevice::DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize)
	{
	}

	void TrackableDevice::UpdatePose(vr::TrackedDeviceIndex_t unObjectId)
	{
		vr::VRServerDriverHost()->TrackedDevicePoseUpdated(unObjectId, m_Pose, sizeof(vr::DriverPose_t));
	}

	vr::DriverPose_t TrackableDevice::GetPose()
	{
		// This is only called at startup to synchronize with the driver.
		// Future updates are driven by our thread calling TrackedDevicePoseUpdated()
		return m_Pose;
	}

	// TrackedDevice Interface
	vr::ETrackedDeviceClass TrackableDevice::GetTrackedDeviceClass() const
	{
		// TODO implement this properly
		return vr::TrackedDeviceClass_Invalid;
	}

	// Returns the tracked device's role. e.g. TrackedControllerRole_LeftHand
	vr::ETrackedControllerRole TrackableDevice::GetTrackedDeviceRole() const
	{
		return m_TrackedControllerRole;
	}

	// Will return true based on whether a TrackedDeviceIndex was assigned during Activate()
	bool TrackableDevice::IsActivated() const
	{
		return m_unSteamVRTrackedDeviceId != vr::k_unTrackedDeviceIndexInvalid;
	}

	// Updates the tracked device through OpenVR's IVRDriverInput from its current state.
	void TrackableDevice::Update()
	{
	}

	void TrackableDevice::RefreshWorldFromDriverPose(glm::vec3 pose, glm::vec4 quat)
	{
		//const PSMPosef worldFromDriverPose = CServerDriver_PSMoveService::getInstance()->GetWorldFromDriverPose();

		//soft_knuckles::Logger::Info("CServerDriver_PSMoveService::RefreshWorldFromDriverPose() for device %s\n", GetSteamVRIdentifier());
		//soft_knuckles::Logger::Info("  worldFromDriverPose: %s \n", soft_knuckles::Utils::PSMPosefToString(worldFromDriverPose).c_str());

		// Transform used to convert from PSMove Tracking space to OpenVR Tracking Space
		m_Pose.qWorldFromDriverRotation.w = quat.w; // worldFromDriverPose.Orientation.w;
		m_Pose.qWorldFromDriverRotation.x = quat.x; // worldFromDriverPose.Orientation.x;
		m_Pose.qWorldFromDriverRotation.y = quat.y; // worldFromDriverPose.Orientation.y;
		m_Pose.qWorldFromDriverRotation.z = quat.z; // worldFromDriverPose.Orientation.z;
		m_Pose.vecWorldFromDriverTranslation[0] = pose.x; // worldFromDriverPose.Position.x;
		m_Pose.vecWorldFromDriverTranslation[1] = pose.y; // worldFromDriverPose.Position.y;
		m_Pose.vecWorldFromDriverTranslation[2] = pose.z; // worldFromDriverPose.Position.z;
	}

	/*
	PSMPosef TrackableDevice::GetWorldFromDriverPose() {
		PSMVector3f psmToOpenVRTranslation = {
			(float)m_Pose.vecWorldFromDriverTranslation[0],
			(float)m_Pose.vecWorldFromDriverTranslation[1],
			(float)m_Pose.vecWorldFromDriverTranslation[2] };
		PSMQuatf psmToOpenVRRotation = PSM_QuatfCreate(
			(float)m_Pose.qWorldFromDriverRotation.w,
			(float)m_Pose.qWorldFromDriverRotation.x,
			(float)m_Pose.qWorldFromDriverRotation.y,
			(float)m_Pose.qWorldFromDriverRotation.x);
		PSMPosef psmToOpenVRPose = PSM_PosefCreate(&psmToOpenVRTranslation, &psmToOpenVRRotation);

		return psmToOpenVRPose;
	}
	*/

	const char* TrackableDevice::GetSteamVRIdentifier() const
	{
		return m_strSteamVRSerialNo.c_str();
	}

	const vr::TrackedDeviceIndex_t TrackableDevice::getTrackedDeviceIndex()
	{
		return m_unSteamVRTrackedDeviceId;
	}
}
