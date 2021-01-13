#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/detail/type_vec3.hpp>
#include <glm/detail/type_vec4.hpp>
#include <glm/detail/type_vec2.hpp>
#include "boost/lexical_cast.hpp"

#include <openvr_driver.h>
#include "BaseStation.h"

using namespace vr;
using namespace boost;
using namespace std;

namespace soft_knuckles
{
	BaseStation::BaseStation(const int id)
		: TrackableDevice()
		  , m_nTrackerId(id)
	{
		char buf[256];
		//Utils::GenerateTrackerSerialNumber(buf, sizeof(buf), trackerInfo->tracker_id);
		m_strSteamVRSerialNo = "base" + id; //buf;

		SetClientTrackerInfo(id);
	}

	BaseStation::~BaseStation()
	{
	}

	EVRInitError BaseStation::Activate(TrackedDeviceIndex_t unObjectId)
	{
		EVRInitError result = TrackableDevice::Activate(unObjectId);
		ObjId = unObjectId;

		if (result == VRInitError_None)
		{
			CVRPropertyHelpers* properties = VRProperties();

			properties->SetStringProperty(m_ulPropertyContainer, Prop_TrackingSystemName_String, "lighthouse");

			properties->SetFloatProperty(m_ulPropertyContainer, Prop_FieldOfViewLeftDegrees_Float, 120.0f);
			// m_tracker_info.tracker_hfov / 2.f);
			properties->SetFloatProperty(m_ulPropertyContainer, Prop_FieldOfViewRightDegrees_Float, 120.0f);
			// m_tracker_info.tracker_hfov / 2.f);
			properties->SetFloatProperty(m_ulPropertyContainer, Prop_FieldOfViewTopDegrees_Float, 120.0f);
			// m_tracker_info.tracker_vfov / 2.f);
			properties->SetFloatProperty(m_ulPropertyContainer, Prop_FieldOfViewBottomDegrees_Float, 120.0f);
			// m_tracker_info.tracker_vfov / 2.f);
			properties->SetFloatProperty(m_ulPropertyContainer, Prop_TrackingRangeMinimumMeters_Float, 0.0f);
			// m_tracker_info.tracker_znear);// *k_fScalePSMoveAPIToMeters);
			properties->SetFloatProperty(m_ulPropertyContainer, Prop_TrackingRangeMaximumMeters_Float, 100000.0f);
			// m_tracker_info.tracker_zfar);// *k_fScalePSMoveAPIToMeters);

			properties->SetInt32Property(m_ulPropertyContainer, Prop_DeviceClass_Int32,
			                             TrackedDeviceClass_TrackingReference);

			// The {psmove} syntax lets us refer to rendermodels that are installed
			// in the driver's own resources/rendermodels directory.  The driver can
			// still refer to SteamVR models like "generic_hmd".
			properties->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String,
			                              "{htc}/rendermodels/vr_tracker_vive_1_0/vr_tracker_vive_1_0.obj");

			char model_label[16] = "\0";
			snprintf(model_label, sizeof(model_label), "Lighthouse_%d", unObjectId); // m_tracker_info.tracker_id);
			properties->SetStringProperty(m_ulPropertyContainer, Prop_ModeLabel_String, model_label);

			properties->SetStringProperty(m_ulPropertyContainer, Prop_NamedIconPathDeviceOff_String,
			                              "{lighthouse}/icons/base_status_off.b4bfb144.png");
			properties->SetStringProperty(m_ulPropertyContainer, Prop_NamedIconPathDeviceSearching_String,
			                              "{lighthouse}/icons/base_status_ready.b4bfb144.png");
			properties->SetStringProperty(m_ulPropertyContainer, Prop_NamedIconPathDeviceSearchingAlert_String,
			                              "{lighthouse}/icons/base_status_ready_alert.b4bfb144.png");
			properties->SetStringProperty(m_ulPropertyContainer, Prop_NamedIconPathDeviceReady_String,
			                              "{lighthouse}/icons/base_status_ready.b4bfb144.png");
			properties->SetStringProperty(m_ulPropertyContainer, Prop_NamedIconPathDeviceReadyAlert_String,
			                              "{lighthouse}/icons/base_status_ready_alert.b4bfb144.png");
			properties->SetStringProperty(m_ulPropertyContainer, Prop_NamedIconPathDeviceNotReady_String,
			                              "{lighthouse}/icons/base_status_error.b4bfb144.png");
			properties->SetStringProperty(m_ulPropertyContainer, Prop_NamedIconPathDeviceStandby_String,
			                              "{lighthouse}/icons/base_status_standby.b4bfb144.png");
			properties->SetStringProperty(m_ulPropertyContainer, Prop_NamedIconPathDeviceAlertLow_String,
			                              "{lighthouse}/icons/base_status_ready_low.b4bfb144.png");

			// Poll the latest WorldFromDriverPose transform we got from the service
			// Transform used to convert from PSMove Tracking space to OpenVR Tracking Space
			RefreshWorldFromDriverPose(pose, glm::vec4(glm::quat(rots).x, glm::quat(rots).y, glm::quat(rots).z,
			                                           glm::quat(rots).w));
		}

		return result;
	}

	void BaseStation::Deactivate()
	{
	}

	void BaseStation::SetClientTrackerInfo(
		const int id)
	{
		//m_tracker_info = *trackerInfo;

		m_Pose.result = TrackingResult_Running_OK;

		m_Pose.deviceIsConnected = true;

		// Yaw can't drift because the tracker never moves (hopefully)
		m_Pose.willDriftInYaw = false;
		m_Pose.shouldApplyHeadModel = false;

		// No prediction since that's already handled in the psmove service
		m_Pose.poseTimeOffset = 0.f;

		// Poll the latest WorldFromDriverPose transform we got from the service
		// Transform used to convert from PSMove Tracking space to OpenVR Tracking Space
		//RefreshWorldFromDriverPose();

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
			//const PSMVector3f& position = m_tracker_info.tracker_pose.Position;
			glm::vec3 position(pose);

			m_Pose.vecPosition[0] = position.x; // *k_fScalePSMoveAPIToMeters;
			m_Pose.vecPosition[1] = position.y; // *k_fScalePSMoveAPIToMeters;
			m_Pose.vecPosition[2] = position.z; // *k_fScalePSMoveAPIToMeters;
		}

		// Set rotational coordinates
		{
			//const PSMQuatf& orientation = m_tracker_info.tracker_pose.Orientation;
			glm::quat orientation = glm::quat(rots);

			m_Pose.qRotation.w = orientation.w;
			m_Pose.qRotation.x = orientation.x;
			m_Pose.qRotation.y = orientation.y;
			m_Pose.qRotation.z = orientation.z;
		}

		m_Pose.poseIsValid = true;

		RefreshWorldFromDriverPose(pose, glm::vec4(glm::quat(rots).x, glm::quat(rots).y, glm::quat(rots).z,
		                                           glm::quat(rots).w));
	}

	void BaseStation::Update()
	{
		TrackableDevice::Update();
		TrackableDevice::UpdatePose(m_unSteamVRTrackedDeviceId);

		/*
		m_Pose.vecPosition[0] = pose.x;// *k_fScalePSMoveAPIToMeters;
		m_Pose.vecPosition[1] = pose.y;// *k_fScalePSMoveAPIToMeters;
		m_Pose.vecPosition[2] = pose.z;// *k_fScalePSMoveAPIToMeters;

		m_Pose.qRotation.w = glm::quat(rots).w;
		m_Pose.qRotation.x = glm::quat(rots).x;
		m_Pose.qRotation.y = glm::quat(rots).y;
		m_Pose.qRotation.z = glm::quat(rots).z;
		*/
		// This call posts this pose to shared memory, where all clients will have access to it the next
		// moment they want to predict a pose.
		//VRServerDriverHost()->TrackedDevicePoseUpdated(m_unSteamVRTrackedDeviceId, m_Pose, sizeof(DriverPose_t));
	}

	bool BaseStation::HasTrackerId(int TrackerID)
	{
		return TrackerID == m_nTrackerId;
	}
}
