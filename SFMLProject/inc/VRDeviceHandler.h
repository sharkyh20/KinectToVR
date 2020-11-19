#pragma once


#include <openvr.h>

#include "DeviceHandler.h"
#include "TrackingPoolManager.h"
#include "TrackedDeviceInputData.h"
#include "VRHelper.h"

#include <openvr_math.h>
#include <Eigen/Geometry>
#include <vrinputemulator.h>

#include <cereal/archives/json.hpp>
#include <cereal/cereal.hpp>
#include <KinectSettings.h>

#include <cereal/cereal.hpp>
#include <cereal/archives/binary.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/split_free.hpp>

enum class VirtualHipMode
{
	Standing,
	Sitting,
	Lying
};

struct VirtualHipSettings
{
	bool followHmdYawRotation = true;
	bool followHmdRollRotation = false;
	bool followHmdPitchRotation = false;

	VirtualHipMode hipMode = VirtualHipMode::Standing;

	double positionLatency = 0.00;
	// Seconds. How far behind the tracked point should the hips be so that they don't instantly follow every tiny movement
	bool rtcalib = false;

	bool astartk = false;
	bool astarth = false;
	bool astartt = false;
	float tryawst, kinpitchst;
	int autosver = 1;
	int footOption, hipsOption, posOption = 3, conOption;
	int bodyTrackingOption = 1, headTrackingOption = 1;
	// --- Standing Settings ---
	bool positionFollowsHMDLean = false;
	// Determines whether the virtual hips in standing mode will stay above the foot trackers, or interpolate between the HMD and foot trackers on a direct slant
	float hmdegree = 0.0;
	float tdegree = 3;
	double heightFromHMD = 0.00;
	// Meters. Hips are by default projected downwards from the HMD, by 72cm (adjustable by user)
	bool positionAccountsForFootTrackers = false;
	// If false, Hip tracker always stays bolted to directly under the HMD with no horizontal shift

	Eigen::Matrix<float, 3, 3> rcR_matT;
	Eigen::Matrix<float, 3, 1> rcT_matT;
	Eigen::Matrix<float, 3, 1> hauoffset_s;
	Eigen::Matrix<float, 3, 1> mauoffset_s;
	Eigen::Vector3f caliborigin;

	float rcR_matT_S[3][3];
	float rcT_matT_S[3];
	float hauoffset_s_S[3];
	float mauoffset_s_S[3];
	float caliborigin_S[3];

	bool astarta = false;
	std::string comph, compm;
	// --- Sitting Settings ---
	double sittingMaxHeightThreshold = 0.00; // Meters. Under this height, mode is sitting


	// --- Lying Settings
	double hipThickness = 0.00;
	// Meters. Essentially how wide the hips are, so that when lying down, they are put slightly above the ground
	double lyingMaxHeightThreshold = 0.00; // Meters. Under this height, mode is lying

	template <class Archive>
	void serialize(Archive& archive)
	{
		archive(
			CEREAL_NVP(followHmdYawRotation),
			CEREAL_NVP(followHmdRollRotation),
			CEREAL_NVP(followHmdPitchRotation),
			CEREAL_NVP(hipMode),
			CEREAL_NVP(positionFollowsHMDLean),
			CEREAL_NVP(hmdegree),
			CEREAL_NVP(tdegree),
			CEREAL_NVP(astartk),
			CEREAL_NVP(footOption),
			CEREAL_NVP(hipsOption),
			CEREAL_NVP(posOption),
			CEREAL_NVP(conOption),
			CEREAL_NVP(bodyTrackingOption),
			CEREAL_NVP(headTrackingOption),
			CEREAL_NVP(comph),
			CEREAL_NVP(compm),
			CEREAL_NVP(astartt),
			CEREAL_NVP(astarta),
			CEREAL_NVP(autosver),
			CEREAL_NVP(astarth),
			CEREAL_NVP(tryawst),
			CEREAL_NVP(kinpitchst),

			CEREAL_NVP(rtcalib),
			CEREAL_NVP(heightFromHMD),
			CEREAL_NVP(positionAccountsForFootTrackers),
			CEREAL_NVP(sittingMaxHeightThreshold),
			CEREAL_NVP(hipThickness),
			CEREAL_NVP(lyingMaxHeightThreshold),

			CEREAL_NVP(caliborigin_S),
			CEREAL_NVP(hauoffset_s_S),
			CEREAL_NVP(mauoffset_s_S),
			CEREAL_NVP(rcR_matT_S),
			CEREAL_NVP(rcT_matT_S)
		);
	}
};

void decomposeEigen(VirtualHipSettings& settings)
{
	for (int i = 0; i < 3; i++)
	{
		settings.rcT_matT_S[i] = settings.rcT_matT[i];
		settings.hauoffset_s_S[i] = settings.hauoffset_s[i];
		settings.mauoffset_s_S[i] = settings.mauoffset_s[i];
		settings.caliborigin_S[i] = settings.caliborigin[i];
		for (int j = 0; j < 3; j++)
			settings.rcR_matT_S[i][j] = settings.rcR_matT.coeff(i, j);
	}
}

void recomposeEigen(VirtualHipSettings& settings)
{
	for (int i = 0; i < 3; i++)
	{
		settings.rcT_matT[i] = settings.rcT_matT_S[i];
		settings.hauoffset_s[i] = settings.hauoffset_s_S[i];
		settings.mauoffset_s[i] = settings.mauoffset_s_S[i];
		settings.caliborigin[i] = settings.caliborigin_S[i];
		for (int j = 0; j < 3; j++)
			settings.rcR_matT.coeffRef(i, j) = settings.rcR_matT_S[i][j];
	}
}

namespace VirtualHips
{
	VirtualHipSettings settings;
	static const std::wstring settingsConfig = L"ConfigSettings.cfg";

	void saveSettings()
	{
		decomposeEigen(settings);

		std::ofstream os(KVR::fileToDirPath(settingsConfig));
		if (os.fail())
		{
			//FAIL!!!
			LOG(ERROR) << "ERROR: COULD NOT WRITE TO SETTINGS FILE\n";
		}
		else
		{
			cereal::JSONOutputArchive archive(os);
			LOG(INFO) << "Attempted to save settings to file";
			try
			{
				archive(
					CEREAL_NVP(settings)
				);
			}
			catch (cereal::RapidJSONException e)
			{
				LOG(ERROR) << "CONFIG FILE SAVE JSON ERROR: " << e.what();
			}
		}
	}

	void retrieveSettings()
	{
		std::ifstream is(KVR::fileToDirPath(settingsConfig));
		LOG(INFO) << "Attempted to load settings at " << KVR::fileToDirPath(settingsConfig);

		if (is.fail())
		{
			LOG(ERROR) << "Settings file could not be found, generating a new one...";
			saveSettings();
		}
		else
		{
			LOG(INFO) << settingsConfig << " load attempted!";
			try
			{
				cereal::JSONInputArchive archive(is);
				archive(CEREAL_NVP(settings));

				recomposeEigen(settings);

				KinectSettings::hroffset = settings.hmdegree;
				KinectSettings::cpoints = settings.tdegree;

				KinectSettings::calibration_rotation = settings.rcR_matT;
				KinectSettings::calibration_translation = settings.rcT_matT;

				KinectSettings::huoffsets.v[0] = settings.heightFromHMD;
				KinectSettings::huoffsets.v[1] = settings.sittingMaxHeightThreshold;
				KinectSettings::huoffsets.v[2] = settings.lyingMaxHeightThreshold;

				KinectSettings::matrixes_calibrated = settings.rtcalib;
				KinectSettings::calibration_trackers_yaw = settings.tryawst;
				KinectSettings::calibration_kinect_pitch = settings.kinpitchst;

				KinectSettings::hauoffset.v[0] = settings.hauoffset_s(0);
				KinectSettings::hauoffset.v[1] = settings.hauoffset_s(1);
				KinectSettings::hauoffset.v[2] = settings.hauoffset_s(2);

				KinectSettings::mauoffset.v[0] = settings.mauoffset_s(0);
				KinectSettings::mauoffset.v[1] = settings.mauoffset_s(1);
				KinectSettings::mauoffset.v[2] = settings.mauoffset_s(2);

				footOrientationFilterOption.filterOption = static_cast<footRotationFilterOption>(settings.footOption);
				hipsOrientationFilterOption.filterOption = static_cast<hipsRotationFilterOption>(settings.hipsOption);
				positionFilterOption.filterOption = static_cast<positionalFilterOption>(settings.posOption);
				controllersTrackingOption_s.trackingOption = static_cast<controllersTrackingOption>(settings.conOption);
				bodyTrackingOption_s.trackingOption = static_cast<bodyTrackingOption>(settings.bodyTrackingOption);

				KinectSettings::calibration_origin = settings.caliborigin;

				LOG(INFO) << settings.tryawst << '\n' << settings.rcR_matT << '\n' << KinectSettings::calibration_trackers_yaw << '\n' <<
					KinectSettings::calibration_rotation << '\n';
			}
			catch (cereal::Exception e)
			{
				LOG(ERROR) << settingsConfig << "SETTINGS FILE LOAD JSON ERROR: " << e.what();
			}
		}
	}
}

class VRDeviceHandler : public DeviceHandler
{
	// Updates the tracking pool with data from the 
	// non-IE SteamVR devices - e.g. head position/rotation
public:
	VRDeviceHandler(vr::IVRSystem* & g_VRSystem)
		: m_VRSystem(g_VRSystem)
	{
	}

	~VRDeviceHandler()
	{
	}

	int initialise() override
	{
		// Add all devices that aren't sensors or virtual
		LOG(INFO) << "Initialising VR Device Handler...";

		vr::TrackedDevicePose_t devicePose[vr::k_unMaxTrackedDeviceCount];
		m_VRSystem->GetDeviceToAbsoluteTrackingPose(vr::ETrackingUniverseOrigin::TrackingUniverseStanding, 0,
		                                            devicePose, vr::k_unMaxTrackedDeviceCount);

		for (int i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i)
		{
			// If device virtual, skip
			if (virtualDevices[i])
				continue;

			// If device has no useful pos/rot data, skip
			auto deviceClass = m_VRSystem->GetTrackedDeviceClass(i);
			if (deviceClass == vr::TrackedDeviceClass_Invalid)
				break; // No point iterating past the last device
			if (deviceClass == vr::TrackedDeviceClass_TrackingReference)
				continue;

			KVR::TrackedDeviceInputData data = defaultDeviceData(i);
			uint32_t globalID = k_invalidTrackerID;
			TrackingPoolManager::addDeviceToPool(data, globalID);

			vrDeviceToPoolIds[i].internalID = i;
			vrDeviceToPoolIds[i].globalID = globalID;
		}

		initVirtualHips();

		active = true;
		return 0;
	}

	int run() override
	{
		vr::TrackedDevicePose_t devicePoses[vr::k_unMaxTrackedDeviceCount];
		m_VRSystem->GetDeviceToAbsoluteTrackingPose(vr::ETrackingUniverseOrigin::TrackingUniverseStanding, 0,
		                                            devicePoses, vr::k_unMaxTrackedDeviceCount);

		for (int i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i)
		{
			vr::ETrackedDeviceClass deviceClass = m_VRSystem->GetTrackedDeviceClass(i);
			if (deviceClass == vr::TrackedDeviceClass_Invalid)
				break; // No point iterating past the last device

			if (devicePoses[i].bPoseIsValid && vrDeviceToPoolIds[i].globalID != k_invalidTrackerID)
			{
				vr::TrackedDevicePose_t pose = devicePoses[i];
				vr::HmdVector3d_t position{};
				vr::HmdQuaternion_t rotation{};
				position = GetVRPositionFromMatrix(pose.mDeviceToAbsoluteTracking);
				rotation = GetVRRotationFromMatrix(pose.mDeviceToAbsoluteTracking);

				KVR::TrackedDeviceInputData data = defaultDeviceData(i);
				data.deviceId = vrDeviceToPoolIds[i].globalID;
				data.position = position;
				data.rotation = rotation;

				data.pose = trackedDeviceToDriverPose(pose);
				data.pose.vecPosition[0] = position.v[0];
				data.pose.vecPosition[1] = position.v[1];
				data.pose.vecPosition[2] = position.v[2];
				data.pose.qRotation = rotation;

				TrackingPoolManager::updatePoolWithDevice(data, vrDeviceToPoolIds[i].globalID);
			}
		}

		updateVirtualHips();

		return 0;
	}

	void updateVirtualDeviceList()
	{
		//int virtualDeviceCount = m_inputEmulator.getVirtualDeviceCount();
		//std::fill(virtualDevices, virtualDevices + vr::k_unMaxTrackedDeviceCount, false);
		//for (int i = 0; i < virtualDeviceCount; ++i) {
		//    vrinputemulator::VirtualDeviceInfo info = m_inputEmulator.getVirtualDeviceInfo(i);
		//    virtualDevices[info.openvrDeviceId] = true;
		//}
	}

private:
	vr::IVRSystem* & m_VRSystem;
	//vrinputemulator::VRInputEmulator & m_inputEmulator;

	int virtualDeviceCount = 0;
	bool virtualDevices[vr::k_unMaxTrackedDeviceCount]{false};
	TrackerIDs vrDeviceToPoolIds[vr::k_unMaxTrackedDeviceCount]{};
	TrackerIDs virtualHipsIds{};
	uint32_t virtualHipsLocalId = 420;

	void initVirtualHips()
	{
		LOG(INFO) << "Reading Strings...";
		VirtualHips::retrieveSettings();

		KVR::TrackedDeviceInputData data = defaultDeviceData(virtualHipsLocalId);

		uint32_t globalID = k_invalidTrackerID;
		TrackingPoolManager::addDeviceToPool(data, globalID);

		virtualHipsIds.internalID = virtualHipsLocalId;
		virtualHipsIds.globalID = globalID;
	}

	bool footTrackersAvailable()
	{
		return
			TrackingPoolManager::leftFootDevicePosGID != k_invalidTrackerID &&
			TrackingPoolManager::rightFootDevicePosGID != k_invalidTrackerID &&
			TrackingPoolManager::leftFootDeviceRotGID != k_invalidTrackerID &&
			TrackingPoolManager::rightFootDeviceRotGID != k_invalidTrackerID;
	}

	void calculateHipMode()
	{
		// Determine user mode from HMD position
		if (KinectSettings::hmdPosition.v[1] <= VirtualHips::settings.lyingMaxHeightThreshold)
			VirtualHips::settings.hipMode = VirtualHipMode::Lying;
		else if (KinectSettings::hmdPosition.v[1] <= VirtualHips::settings.sittingMaxHeightThreshold)
			VirtualHips::settings.hipMode = VirtualHipMode::Sitting;
		else
			VirtualHips::settings.hipMode = VirtualHipMode::Standing;
	}

	vr::HmdVector3d_t getAverageFootPosition()
	{
		// Get average of feet controller positions
		// NEED TO TAKE INTO ACCOUNT DRIVER-WORLD OFFSET!
		auto leftData = TrackingPoolManager::getDeviceData(TrackingPoolManager::leftFootDevicePosGID);
		auto rightData = TrackingPoolManager::getDeviceData(TrackingPoolManager::rightFootDevicePosGID);

		vr::HmdVector3d_t leftPos = getWorldPositionFromDriverPose(leftData.pose);
		vr::HmdVector3d_t rightPos = getWorldPositionFromDriverPose(rightData.pose);

		return vr::HmdVector3d_t{
			(leftPos.v[0] + rightPos.v[0]) * 0.5,
			(leftPos.v[1] + rightPos.v[1]) * 0.5,
			(leftPos.v[2] + rightPos.v[2]) * 0.5
		};
	}

	void calculateStandingPosition(vr::HmdVector3d_t& hipPosition)
	{
		// Initially use head position, project downwards
		hipPosition = KinectSettings::hmdPosition;
		hipPosition.v[1] -= VirtualHips::settings.heightFromHMD;

		if (VirtualHips::settings.positionAccountsForFootTrackers &&
			footTrackersAvailable())
		{
			vr::HmdVector3d_t averageFeetPos = getAverageFootPosition();

			// Use their X and Z
			hipPosition.v[0] = averageFeetPos.v[0];
			hipPosition.v[2] = averageFeetPos.v[2];
		}
	}

	void calculateSittingPosition(vr::HmdVector3d_t& hipPosition)
	{
		// Initially use head position, project downwards
		hipPosition = KinectSettings::hmdPosition;
		hipPosition.v[1] -= VirtualHips::settings.heightFromHMD;

		// Prevents sinking when head gets closer to ground
		if (hipPosition.v[1] <= VirtualHips::settings.hipThickness)
		{
			hipPosition.v[1] = VirtualHips::settings.hipThickness + 0.20;
		}

		// Use the x,z average of all tracked points - for now, turned off as it didn't have the desired result
		/*
		if (VirtualHips::settings.positionAccountsForFootTrackers &&
		    footTrackersAvailable()) {
		    const vr::HmdVector3d_t & headPos = KinectSettings::hmdPosition;

		    vr::HmdVector3d_t averageFeetPos = getAverageFootPosition();
		    
		    vr::HmdVector3d_t averageHipPos{
		        (averageFeetPos.v[0] + headPos.v[0]) * 0.5,
		        (averageFeetPos.v[1] + headPos.v[1]) * 0.5,
		        (averageFeetPos.v[2] + headPos.v[2]) * 0.5
		    };

		    // Use their X and Z
		    hipPosition.v[0] = averageFeetPos.v[0];
		    hipPosition.v[2] = averageFeetPos.v[2];
		}
		else {
		    // Stays only under the HMD
		}
		*/
	}

	void calculateLyingPosition(vr::HmdVector3d_t& hipPosition)
	{
		// Initially use head position, project downwards
		hipPosition = KinectSettings::hmdPosition;
		hipPosition.v[1] -= VirtualHips::settings.heightFromHMD;

		// Prevents sinking when head gets closer to ground
		if (hipPosition.v[1] <= VirtualHips::settings.hipThickness)
		{
			hipPosition.v[1] = VirtualHips::settings.hipThickness;
		}

		// Move the tracker horizontally to simulate hip position when lying down
		// https://math.stackexchange.com/questions/83404/finding-a-point-along-a-line-in-three-dimensions-given-two-points

		if (footTrackersAvailable())
		{
			// Get the point 'd' units along the line from point 'A' to 'B'
			vr::HmdVector3d_t A = KinectSettings::hmdPosition;
			vr::HmdVector3d_t B = getAverageFootPosition();
			double ratio = 0.5; // Ratio for how far from the head to the feet the hips are
			double distanceAB = sqrt(
				(B.v[0] - A.v[0]) * (B.v[0] - A.v[0]) +
				(B.v[1] - A.v[1]) * (B.v[1] - A.v[1]) +
				(B.v[2] - A.v[2]) * (B.v[2] - A.v[2])
			);
			//double d = VirtualHips::settings.heightFromHMD
			double d = distanceAB * ratio;

			vr::HmdVector3d_t BA = B - A;

			// Normalise
			double length = sqrt(
				BA.v[0] * BA.v[0] +
				BA.v[1] * BA.v[1] +
				BA.v[2] * BA.v[2]);
			vr::HmdVector3d_t unitVector = (BA) / length;

			// Scale towards B
			vr::HmdVector3d_t desiredHipPosition = A + (unitVector * d);
			hipPosition.v[0] = desiredHipPosition.v[0];
			hipPosition.v[2] = desiredHipPosition.v[2];
		}
	}

	void updateVirtualHips()
	{
		// Has access to head point directly, (and controllers if necessary)
		// Needs feet points to be supplied in order to properly predict the hips

		calculateHipMode();

		// Calculate Position
		vr::HmdVector3d_t hipPosition{0};

		switch (VirtualHips::settings.hipMode)
		{
		case VirtualHipMode::Standing:
			{
				calculateStandingPosition(hipPosition);
				//LOG(INFO) << "Standing";
				break;
			}
		case VirtualHipMode::Sitting:
			{
				calculateSittingPosition(hipPosition);
				//LOG(INFO) << "Sitting";
				break;
			}
		case VirtualHipMode::Lying:
			{
				calculateLyingPosition(hipPosition);
				//LOG(INFO) << "Lying";
				break;
			}
		default:
			{
				LOG(ERROR) << "Virtual Hip mode invalid!!!";
			}
		}

		// Calculate Rotation
		vr::HmdQuaternion_t rotation{1, 0, 0, 0};
		double yaw = 0;
		double pitch = 0;
		double roll = 0;
		toEulerAngle(KinectSettings::hmdRotation, pitch, yaw, roll);

		vr::HmdQuaternion_t yawRotation = {1, 0, 0, 0};
		vr::HmdQuaternion_t pitchRotation = {1, 0, 0, 0};
		vr::HmdQuaternion_t rollRotation = {1, 0, 0, 0};

		switch (VirtualHips::settings.hipMode)
		{
		case VirtualHipMode::Standing:
			{
				if (VirtualHips::settings.followHmdYawRotation)
					yawRotation = vrmath::quaternionFromRotationY(yaw);
				if (VirtualHips::settings.followHmdPitchRotation)
					pitchRotation = vrmath::quaternionFromRotationX(pitch);
				if (VirtualHips::settings.followHmdRollRotation)
					rollRotation = vrmath::quaternionFromRotationZ(roll);
				break;
			}
		case VirtualHipMode::Sitting:
			{
				// As the hip tracker sinks further into the ground, rotate the hip upwards
				// Apply yaw first, then pitch

				if (VirtualHips::settings.followHmdYawRotation)
				{
					yawRotation = vrmath::quaternionFromRotationY(yaw);
				}

				// Adjust up/down

				const double maxRotation = M_PI / 4.0; // 45 degrees up

				double rotationRatio = (
						KinectSettings::hmdPosition.v[1] - VirtualHips::settings.heightFromHMD)
					/ (VirtualHips::settings.sittingMaxHeightThreshold - VirtualHips::settings.heightFromHMD);
				double radiansToRotatePitch = -(maxRotation * rotationRatio);
				pitchRotation = vrmath::quaternionFromRotationX(radiansToRotatePitch);
				break;
			}
		case VirtualHipMode::Lying:
			{
				// Follows HMD roll now, and yaw is based on lookat from HMD to feet
				//rollRotation = vrmath::quaternionFromRotationZ(roll);
				pitchRotation = vrmath::quaternionFromRotationX(M_PI_2);
				yawRotation = {1, 0, 0, 0};

				if (footTrackersAvailable())
				{
					// rotation between
					auto rawQ = vrmath::get_rotation_between(KinectSettings::hmdPosition, getAverageFootPosition());
					double lookAtYaw = 0;
					double lookAtPitch = 0;
					double lookAtRoll = 0;

					toEulerAngle(rawQ, lookAtPitch, lookAtYaw, lookAtRoll);
					LOG(INFO) << lookAtYaw;
					yawRotation = vrmath::quaternionFromRotationY(lookAtYaw);
					pitchRotation = vrmath::quaternionFromRotationX(lookAtPitch);
					rollRotation = vrmath::quaternionFromRotationZ(lookAtRoll);
					//yawRotation = vrmath::quaternionFromRotationY(lookAtYaw) * vrmath::quaternionFromRotationY(yaw);
					//pitchRotation = vrmath::quaternionFromRotationX(lookAtPitch) * vrmath::quaternionFromRotationX(pitch);
					//rollRotation = vrmath::quaternionFromRotationZ(lookAtRoll) * vrmath::quaternionFromRotationZ(roll);
				}
				break;
			}
		default:
			{
				LOG(ERROR) << "Virtual Hip mode invalid!!!";
			}
		}

		rotation = yawRotation * pitchRotation * rollRotation; // Right side applied first

		KVR::TrackedDeviceInputData data = defaultDeviceData(virtualHipsLocalId);
		data.deviceId = virtualHipsIds.globalID;
		data.position = hipPosition;
		data.rotation = rotation;

		data.pose = defaultReadyDriverPose();
		data.pose.vecPosition[0] = hipPosition.v[0];
		data.pose.vecPosition[1] = hipPosition.v[1];
		data.pose.vecPosition[2] = hipPosition.v[2];
		data.pose.qRotation = rotation;

		TrackingPoolManager::updatePoolWithDevice(data, virtualHipsIds.globalID);
	}

	KVR::TrackedDeviceInputData defaultDeviceData(uint32_t localID)
	{
		// Local ID is the same as SteamVR ID's considering it iterates over the tracked device array
		if (localID == virtualHipsLocalId)
		{
			KVR::TrackedDeviceInputData data;

			data.positionTrackingOption = KVR::JointPositionTrackingOption::IMU;
			data.rotationTrackingOption = KVR::JointRotationTrackingOption::IMU;
			data.parentHandler = dynamic_cast<DeviceHandler*>(this);

			data.deviceName = "Virtual Hips";
			data.serial = "SERIAL_HIPS";
			data.deviceId = virtualHipsLocalId;
			data.customModelName = "vr_controller_vive_1_5";

			return data;
		}

		KVR::TrackedDeviceInputData data;
		data.positionTrackingOption = KVR::JointPositionTrackingOption::IMU;
		data.rotationTrackingOption = KVR::JointRotationTrackingOption::IMU;
		data.parentHandler = dynamic_cast<DeviceHandler*>(this);

		// Courtesy of https://steamcommunity.com/app/358720/discussions/0/1353742967802223832/
		std::string modelName = "";
		std::string serial = "";
		getVRStringProperty(localID, vr::Prop_ModelNumber_String, modelName);
		getVRStringProperty(localID, vr::Prop_SerialNumber_String, serial);

		data.deviceName = "SteamVR ID: " + std::to_string(localID) + " " + modelName;
		data.serial = serial;
		data.deviceId = localID;
		data.customModelName = "vr_controller_vive_1_5";
		return data;
	}

	void getVRStringProperty(const uint32_t& openvrID, vr::ETrackedDeviceProperty strProperty, std::string& string)
	{
		vr::ETrackedPropertyError peError;
		uint32_t unRequiredBufferLen = m_VRSystem->GetStringTrackedDeviceProperty(
			openvrID, strProperty, nullptr, 0, &peError);
		if (unRequiredBufferLen == 0)
		{
			string = "";
		}
		else
		{
			char* pchBuffer = new char[unRequiredBufferLen];
			unRequiredBufferLen = m_VRSystem->GetStringTrackedDeviceProperty(
				openvrID, strProperty, pchBuffer, unRequiredBufferLen, &peError);
			string = pchBuffer;
			delete[] pchBuffer;
		}
	}
};
