#pragma once

#include "stdafx.h"

#include "KinectHandlerBase.h"
#include "TrackingMethod.h"

class IMU_PositionMethod : public TrackingMethod
{
public:
	IMU_PositionMethod()
	{
	}

	~IMU_PositionMethod()
	{
	}

	void initialise() override
	{
	}

	void activate() override
	{
	}

	void terminate() override
	{
	}

	void update(
		KinectHandlerBase& kinect,
		std::vector<KVR::KinectTrackedDevice>& v_trackers
	) override
	{
	}

	void updateTrackers(
		KinectHandlerBase& kinect,
		std::vector<KVR::KinectTrackedDevice>& v_trackers
	) override
	{
		for (int i = 0; i < v_trackers.size(); ++i)
		{
			auto& device = v_trackers[i];
			if (device.positionTrackingOption != KVR::JointPositionTrackingOption::IMU)
			{
				continue;
			}
			if (device.isSensor())
			{
				// Does not handle kinect representation
			}
			else
			{
				auto deviceData = TrackingPoolManager::getDeviceData(device.positionDevice_gId);

				device.setPositionForNextUpdate(deviceData.position);

				// Assume that if the user selects both position and rotation from the same device, it's entire pose will be used
				if (device.positionDevice_gId == device.rotationDevice_gId)
				{
					device.setPoseForNextUpdate(deviceData.pose, true);
				}
			}
		}
	}
};
