#ifndef BASE_STATION_H
#define BASE_STATION_H
#pragma once

#include "trackable_device.h"
#include <openvr_driver.h>
#include <glm/glm.hpp>

using namespace vr;
using namespace std;

namespace soft_knuckles
{
	class BaseStation : public TrackableDevice
	{
	public:
		BaseStation(int id);
		virtual ~BaseStation();
		glm::vec3 pose;
		glm::vec3 rots;
		TrackedDeviceIndex_t ObjId;

		// Overridden Implementation of vr::ITrackedDeviceServerDriver
		EVRInitError Activate(TrackedDeviceIndex_t unObjectId) override;
		void Deactivate() override;

		// Overridden Implementation of CPSMoveTrackedDeviceLatest
		ETrackedDeviceClass GetTrackedDeviceClass() const override { return TrackedDeviceClass_TrackingReference; }
		void Update() override;

		bool HasTrackerId(int ControllerID);
		void SetClientTrackerInfo(int id);

	private:
		// Which tracker
		int m_nTrackerId;

		// The static information about this tracker
		// PSMClientTrackerInfo m_tracker_info;
	};
}

#endif
