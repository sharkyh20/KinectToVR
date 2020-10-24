#pragma once
#include "stdafx.h"
#include <iostream>
#include <vector>
#include <string>

#include "KinectSettings.h"
#include "KinectJoint.h"
#include "IETracker.h"
#include "VRHelper.h"
#include <vrinputemulator.h>
#include <SFML/System/Vector3.hpp>
#include <openvr_math.h>
#include "VectorMath.h"

namespace KVR
{
	enum class KinectDeviceRole
	{
		Unassigned = 0,
		LeftFoot,
		RightFoot,
		Hip,
		KinectSensor,
		LeftHand,
		RightHand,
		Count
	};

	const static std::string KinectDeviceRoleName[] = {
		"Unassigned",
		"Left Foot",
		"Right Foot",
		"Hip",
		"Kinect Sensor",
		"Left Hand",
		"Right Hand",
		"Count"
	};

	enum class JointRotationFilterOption
	{
		Filtered,
		Unfiltered,
		HeadLook
	};

	enum class JointPositionFilterOption
	{
		Filtered,
		Unfiltered
	};

	enum class JointPositionTrackingOption
	{
		Skeleton,
		IMU,
		Color
	};

	enum class JointRotationTrackingOption
	{
		Skeleton,
		IMU,
		Headlook
	};


	class KinectTrackedDevice
	{
	public:
		KinectTrackedDevice(
			vrinputemulator::VRInputEmulator& inputEmulator,
			uint32_t posDevice_gId,
			uint32_t rotDevice_gId,
			KinectDeviceRole r)
			:
			inputEmulatorRef(inputEmulator),
			trackedPositionVROffset({0, 0, 0}),
			positionDevice_gId(posDevice_gId),
			rotationDevice_gId(rotDevice_gId),
			role(r)
		{
		}

		KinectTrackedDevice(
			vrinputemulator::VRInputEmulator& inputEmulator,
			KinectJointType j0,
			KinectJointType j1,
			KinectDeviceRole r)
			:
			inputEmulatorRef(inputEmulator),
			joint0(j0),
			joint1(j1),
			trackedPositionVROffset({0, 0, 0}),
			role(r)
		{
		}

		void init(vrinputemulator::VRInputEmulator& inputEmulator)
		{
			deviceId = initTracker(inputEmulator, true);
			setDeviceProperty(inputEmulator, deviceId, vr::Prop_RenderModelName_String, "string", customModelName);
		}

		void setRotationForNextUpdate(vr::HmdQuaternion_t rotation)
		{
			if (!nextUpdateRotationIsSet)
			{
				nextUpdateRotation = rotation;
				nextUpdateRotationIsSet = true;
			}
		}

		void setPositionForNextUpdate(vr::HmdVector3d_t position)
		{
			if (!nextUpdatePositionIsSet)
			{
				nextUpdatePosition = position;
				nextUpdatePositionIsSet = true;
			}
		}

		void setPoseForNextUpdate(vr::DriverPose_t pose, bool readyForVR = true)
		{
			if (!nextUpdatePoseIsSet)
			{
				nextUpdatePoseIsSet = readyForVR;
				nextUpdatePose = pose;
			}
		}

		void update()
		{
			if (sensorShouldSkipUpdate())
				return;
			// Send through the positions for the next update of the controller
			// - called for each controller at the end of the TrackingMethod iteration
			nextUpdatePositionIsSet = false;
			nextUpdateRotationIsSet = false;

			/* bool poseDataInvalid = (nextUpdatePosition == vr::HmdVector3d_t{ 0,0,0 }
			 * || nextUpdateRotation == vr::HmdQuaternion_t{ 1, 0, 0, 0 });
			 */
			if (!nextUpdatePose.poseIsValid)
			{
				nextUpdatePoseIsSet = false;
				update(lastValidPose);
				return;
			}
			if (nextUpdatePoseIsSet)
			{
				// If pose already handled entirely by tracking method
				// and ready to be sent directly to IE
				// e.g. PSMove's rotation and position and HMD alignment for the one tracker

				// World from driver should be set here - as it is used solely for translating to IE right now, and any devices should have their vecPosition be their properVR position
				// It should not be taken into account for any calculations - as it is SOLELY TO SEND INTO IE

				// Calibrate off the device's own offsets

				applyInputEmulatorOffsets(nextUpdatePose);
				update(nextUpdatePose);

				nextUpdatePoseIsSet = false;
				lastValidPose = nextUpdatePose;
				return;
			}

			vr::DriverPose_t pose{};
			pose.deviceIsConnected = true;

			pose.qRotation = nextUpdateRotation;

			pose.qWorldFromDriverRotation = {1, 0, 0, 0}; // need these else nothing rotates/moves visually
			applyInputEmulatorOffsets(nextUpdatePose);
			update(nextUpdatePose);

			pose.qDriverFromHeadRotation = {1, 0, 0, 0};
			pose.vecDriverFromHeadTranslation[0] = 0;
			pose.vecDriverFromHeadTranslation[1] = 0;
			pose.vecDriverFromHeadTranslation[2] = 0;

			//Final Position Adjustment
			updateDevicePosePosition(pose, nextUpdatePosition);

			pose.poseIsValid = true;

			pose.result = vr::TrackingResult_Running_OK;
			inputEmulatorRef.setVirtualDevicePose(deviceId, pose);

			lastValidPose = pose;
		}

		void applyInputEmulatorOffsets(vr::DriverPose_t& nextUpdatePose)
		{
			// Input Emulator has *very* strange offsets to apply to the trackers

			// This should be evident in the code, but I need to write this down as
			// it makes negative sense.

			// The first 'trackingOriginPosition' is calculated from the standingRawZeroPose
			// provided by SteamVR, and updated at the first run of the program.

			// The 'secondaryTrackingOriginOffset' is strange, and only seems to happen for 
			// some people, sometimes. It must be manually obtained by pressing the 'Calibrate
			// VR Offsets' button after spawning a tracker in the Adv tab - calculated by applying 
			// the offsets like normal, and then seeing if it's still off for some reason.

			nextUpdatePose.vecWorldFromDriverTranslation[0] -= KinectSettings::trackingOriginPosition.v[0];
			nextUpdatePose.vecWorldFromDriverTranslation[1] -= KinectSettings::trackingOriginPosition.v[1];
			nextUpdatePose.vecWorldFromDriverTranslation[2] -= KinectSettings::trackingOriginPosition.v[2];

			nextUpdatePose.vecPosition[0] -= KinectSettings::secondaryTrackingOriginOffset.v[0];
			nextUpdatePose.vecPosition[1] -= KinectSettings::secondaryTrackingOriginOffset.v[1];
			nextUpdatePose.vecPosition[2] -= KinectSettings::secondaryTrackingOriginOffset.v[2];

			nextUpdatePose.qWorldFromDriverRotation = nextUpdatePose.qWorldFromDriverRotation * vrmath::inverse(
				GetVRRotationFromMatrix(KinectSettings::trackingOrigin)); // CLEAN UP INTO SETTINGS
		}

		void update(vr::DriverPose_t pose)
		{
			// Pose already completely handled by Tracking Method
			inputEmulatorRef.setVirtualDevicePose(deviceId, pose);

			// DEBUG
			//LOG(INFO) << "PSMOVE: IE: " << pose.vecPosition[0] + pose.vecWorldFromDriverTranslation[0] << ", " << pose.vecPosition[1] + pose.vecWorldFromDriverTranslation[1] << ", " << pose.vecPosition[2] + pose.vecWorldFromDriverTranslation[2];
		}

		bool sensorShouldSkipUpdate()
		{
			// The sensor doesn't actually need to be updated more often than not,
			// and by only updating it after the config changes, there's a pretty large
			// perf/responsiveness benefit to the trackers
			if (isSensor())
			{
				using namespace KinectSettings;
				if (sensorConfigChanged)
				{
					// Sensor manually updated
					sensorConfigChanged = false;
					return false;
				}
				if (adjustingKinectRepresentationPos
					|| adjustingKinectRepresentationRot) // Still adjusting the sensor
					return false;

				return true;
			}
			return false;
		}

		void update(vr::HmdVector3d_t additionalOffset, vr::HmdVector3d_t rawJointPos,
		            vr::HmdQuaternion_t rawJointRotation)
		{
			// Old, and soon to be deprecated method of updating tracker
			// Done all at once, with little modularity
			if (sensorShouldSkipUpdate())
			{
				return;
			}

			//auto pose = inputEmulatorRef.getVirtualDevicePose(deviceId);
			vr::DriverPose_t pose{};
			vr::HmdQuaternion_t jointRotation = rawJointRotation;
			vr::HmdVector3d_t jointPosition = rawJointPos;


			// JUST FOR PSMOVE TESTING
			usingKinectCalibrationModel = true;
			// ---------------------

			if (usingKinectCalibrationModel)
			{
				applyKinectArrowCalibrationToTracker(jointRotation, jointPosition);
			}
			pose.deviceIsConnected = true;

			/*
			// PSEUDO 360 TRACKING
			// if in fzone
			// do nothing
			double x;
			double y;
			double z;
			toEulerAngle(KinectSettings::hmdRotation, x, y, z);

			// Example of the coordinates of OpenVR from top down
			// O is the center of rotation, and basically North is facing the front of the OpenVR area
			//     +ve           -ve
			//(0)           N
			//              |
			//              |
			//(pi/2) W------o------E
			//              |
			//              |
			//(pi)          S

			// if in backzone
			float halfAngleSizeDegrees = 60.0; // Size on each side of south
			float angleSizeDegrees = halfAngleSizeDegrees * 2.0;
			float halfAngleRadians = (halfAngleSizeDegrees / 180.0 ) * M_PI;

			float westBoundsRadians = M_PI - halfAngleRadians;
			float eastBoundsRadians = -M_PI + halfAngleRadians;

			bool inBackZoneBounds = y >= westBoundsRadians || y <= eastBoundsRadians;

			if (inBackZoneBounds) {
			    //std::cout << "IN RANGE: " << y << "west: " << westBoundsRadians << " east " << eastBoundsRadians <<  '\n';
			    // rotate by 180
			    vr::HmdQuaternion_t rotation180 = { 0,0,1,0 };
			    jointRotation = rotation180 * jointRotation;

			    // Swap joints with handed sides
			    // For now just the feet, but in the future this can be expanded into a lookup table which converts them all
			    
			    switch (role) {
			    case KinectDeviceRole::LeftFoot:
			        joint0.joint = KinectSettings::rightFootJointWithRotation;
			        joint1.joint = KinectSettings::rightFootJointWithoutRotation;
			        break;
			    case KinectDeviceRole::RightFoot:
			        joint0.joint = KinectSettings::leftFootJointWithRotation;
			        joint1.joint = KinectSettings::leftFootJointWithoutRotation;
			        break;
			    case KinectDeviceRole::Hip: // Not handed
			        break;
			    default:
			        LOG(ERROR) << "Pseduo 360 rotation joint swapping attempted on an un-set joint!";
			        break;
			    }
			    
			}
			else {
			    switch (role) {
			    case KinectDeviceRole::LeftFoot:
			        joint0.joint = KinectSettings::leftFootJointWithRotation;
			        joint1.joint = KinectSettings::leftFootJointWithoutRotation;
			        break;
			    case KinectDeviceRole::RightFoot:
			        joint0.joint = KinectSettings::rightFootJointWithRotation;
			        joint1.joint = KinectSettings::rightFootJointWithoutRotation;
			        break;
			    case KinectDeviceRole::Hip: // Not handed
			        break;
			    default:
			        LOG(ERROR) << "Pseduo 360 rotation joint un-swapping attempted on an un-set joint!";
			        break;
			    }
			}
			*/

			pose.qRotation = jointRotation;

			pose.qWorldFromDriverRotation = {1, 0, 0, 0}; // need these two or else nothing rotates visually
			pose.qDriverFromHeadRotation = {1, 0, 0, 0};

			//Final Position Adjustment
			updateDevicePosePosition(pose, jointPosition);

			pose.poseIsValid = true;

			pose.result = vr::TrackingResult_Running_OK;
			inputEmulatorRef.setVirtualDevicePose(deviceId, pose);
		}

		void applyKinectArrowCalibrationToTracker(vr::HmdQuaternion_t& rotation, vr::HmdVector3d_t& position)
		{
			// If using the kinect, then the position/rot of the tracker has to be adjusted
			// according to the arrow, to transform it into the proper OpenVR coords

			//Rotate the position around the kinect rep's rot
			if (isSensor())
			{
				//Rotate arrow by 180
				rotation = rotation * vrmath::quaternionFromRotationY(PI);
			}

			vr::HmdVector3d_t rotatedPos = vrmath::quaternionRotateVector(
				KinectSettings::kinectRepRotation, position, false);

			//Adjust this position by the Kinect's VR pos offset
			rotatedPos.v[0] += KinectSettings::kinectRepPosition.v[0];
			rotatedPos.v[1] += KinectSettings::kinectRepPosition.v[1];
			rotatedPos.v[2] += KinectSettings::kinectRepPosition.v[2];

			if (rotationFilterOption == JointRotationFilterOption::HeadLook)
			{
			}
			else
				rotation = KinectSettings::kinectRepRotation * rotation;
			position = rotatedPos;
		}

		bool isSensor()
		{
			return role == KinectDeviceRole::KinectSensor;
		}

		~KinectTrackedDevice()
		{
		}

		void destroy()
		{
			destroyTracker(inputEmulatorRef, deviceId);
		}

		vrinputemulator::VRInputEmulator& inputEmulatorRef;
		uint32_t deviceId;

		KinectJoint joint0 = KinectJointType::INVALID;
		KinectJoint joint1 = KinectJointType::INVALID;

		std::string defaultModelName{"vr_controller_vive_1_5"};
		std::string customModelName{defaultModelName}; // Set by the device handler in the input data

		bool invisible = false;

		bool usingKinectCalibrationModel = true;

		vr::HmdVector3_t trackedPositionVROffset;
		vr::HmdVector3_t lastRawPos{0, 0, 0};

		vr::DriverPose_t lastValidPose = {};
		vr::DriverPose_t nextUpdatePose = {};
		bool nextUpdatePoseIsSet = false;

		vr::HmdQuaternion_t nextUpdateRotation{1, 0, 0, 0};
		bool nextUpdateRotationIsSet = false;
		vr::HmdVector3d_t nextUpdatePosition{0, 0, 0};
		bool nextUpdatePositionIsSet = false;

		JointRotationFilterOption rotationFilterOption = JointRotationFilterOption::Filtered;
		JointPositionFilterOption positionFilterOption = JointPositionFilterOption::Filtered;
		JointPositionTrackingOption positionTrackingOption = JointPositionTrackingOption::Skeleton;
		JointRotationTrackingOption rotationTrackingOption = JointRotationTrackingOption::Skeleton;

		uint32_t positionDevice_gId = 404;
		uint32_t rotationDevice_gId = 404;


		KinectDeviceRole role;
	private:
		void updateDevicePosePosition(vr::DriverPose_t& pose, vr::HmdVector3d_t rotatedPos)
		{
			pose.vecPosition[0] = rotatedPos.v[0] + trackedPositionVROffset.v[0];
			pose.vecPosition[1] = rotatedPos.v[1] + trackedPositionVROffset.v[1];
			pose.vecPosition[2] = rotatedPos.v[2] + trackedPositionVROffset.v[2];
			if (role == KinectDeviceRole::Hip)
			{
				pose.vecPosition[1] += KinectSettings::hipRoleHeightAdjust;
			}
		}
	};
}
