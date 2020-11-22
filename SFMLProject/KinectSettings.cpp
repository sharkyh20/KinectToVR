#include "stdafx.h"
#include "KinectSettings.h"
#include <cereal/archives/json.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/memory.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <LowPassFilter.h>
#include <EKF_Filter.h>
#include <MathEigen.h>
#include <iostream>
#include <fstream>
#include "wtypes.h"

#include <Windows.h>
#include <codecvt>
#include <openvr_math.h>

namespace KinectSettings
{
	std::wstring const CFG_NAME(L"KinectToVR.cfg");
	std::string KVRversion = "0.8.0 EX";
	int kinectVersion = -1;
	bool psmbuttons[5][10];
	bool isKinectDrawn = false;
	bool isSkeletonDrawn = false;
	bool isDriverPresent = false;
	float svrhmdyaw = 0, calibration_kinect_pitch = 0;
	int psmh, psmm;
	int K2Drivercode = -1; //unknown
	std::vector<int> psmindexidpsm[2];
	int flashnow[2];
	bool conActivated = false;
	bool ignoreInferredPositions = false;
	bool ignoreRotationSmoothing = false;
	float ardroffset = 0.f;
	int positional_tracking_option = 1, headtrackingoption = 1;
	// The joints which actually have rotation change based on the kinect
	// Each kinect type should set these in their process beginning
	// These would be the defaults for the V1
	KVR::KinectJointType leftFootJointWithRotation = KVR::KinectJointType::FootLeft;
	KVR::KinectJointType rightFootJointWithRotation = KVR::KinectJointType::FootRight;
	KVR::KinectJointType leftFootJointWithoutRotation = KVR::KinectJointType::AnkleLeft;
	KVR::KinectJointType rightFootJointWithoutRotation = KVR::KinectJointType::AnkleRight;
	bool isCalibrating = false;

	PSMPSMove right_move_controller, left_move_controller, left_foot_psmove, right_foot_psmove, waist_psmove, atamamove;
	bool isGripPressed[2] = {false, false}, isTriggerPressed[2] = {false, false}; //0L, 1R
	bool initialised = false, isKinectPSMS = false;
	bool userChangingZero = false;
	bool legacy = false;
	float g_TrackedBoneThickness = 6.0f;
	float g_InferredBoneThickness = 1.5f;
	float g_JointThickness = 4.0f;
	std::string opt = "";
	const int kinectHeight = 640;
	const int kinectWidth = 480;
	bool expcalib = true;
	bool frame1 = true;
	int feet_rotation_option, hips_rotation_option, posOption = 2, conOption;

	float map(float value, float start1, float stop1, float start2, float stop2)
	{
		return (start2 + (stop2 - start2) * ((value - start1) / (stop1 - start1)));
	}

	glm::vec3 joybk[2];

	template <class T>
	const T& constrain(const T& x, const T& a, const T& b)
	{
		if (x < a)
		{
			return a;
		}
		if (b < x)
		{
			return b;
		}
		return x;
	}

	glm::vec3 head_position, left_hand_pose, mHandPose, left_foot_raw_pose, right_foot_raw_pose, waist_raw_pose, hElPose, mElPose,
	          lastPose[3][2];
	glm::quat left_foot_raw_ori, right_foot_raw_ori, waist_raw_ori;
	glm::quat trackerSoftRot[2];
	vr::HmdQuaternion_t hmdRot;

	const int kinectV2Height = 1920;
	const int kinectV2Width = 1080;
	bool rtconcalib = false;
	double kinectToVRScale = 1;
	double hipRoleHeightAdjust = 0.0; // in metres up - applied post-scale
	//Need to delete later (Merge should sort it)
	int leftHandPlayspaceMovementButton = 0;
	int rightHandPlayspaceMovementButton = 0;
	int leftFootPlayspaceMovementButton = 0;
	int rightFootPlayspaceMovementButton = 0;
	int psmmigi, psmhidari, psmyobu, psmatama;
	float hmdYaw = 0;
	float conID[2] = {0, 1};

	vr::HmdVector3d_t hmdPosition = {0, 0, 0};
	vr::HmdQuaternion_t hmdRotation = {0, 0, 0, 0};
	vr::HmdMatrix34_t hmdAbsoluteTracking = {};
	extern vr::HmdMatrix34_t trackingOrigin = {};
	extern vr::HmdVector3d_t trackingOriginPosition = {0, 0, 0};
	vr::HmdVector3d_t secondaryTrackingOriginOffset = {0};
	vr::HmdQuaternion_t kinectRepRotation{0, 0, 0, 0}; //TEMP
	vr::HmdVector3d_t kinectRadRotation{0, 0, 0};
	vr::HmdVector3d_t kinectRepPosition{0, 0, 0};
	vr::HmdVector3d_t manual_offsets[2][3] = {{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}};
	vr::HmdVector3d_t hoffsets{0, 0, 0};
	vr::HmdVector3d_t huoffsets{0, 0, 0};
	vr::HmdVector3d_t kinect_tracker_offsets{0, 0, 0};
	float hroffset = 0;
	float troffset = 0;
	Eigen::Vector3f calibration_origin;
	vr::TrackedDevicePose_t controllersPose[2];
	vr::HmdVector3d_t hauoffset{0, 0, 0}, mauoffset{0, 0, 0};
	Eigen::Matrix<float, 3, 3> calibration_rotation;
	Eigen::Matrix<float, 3, 1> calibration_translation;
	bool ismatrixcalibrated = false;
	bool matrixes_calibrated = false;

	float calibration_trackers_yaw = 0.0;
	bool jcalib;
	int cpoints = 3;

	vr::HmdQuaternion_t hmdquat{1, 0, 0, 0};

	vr::HmdVector3d_t kinect_m_positions[3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

	bool headtracked = false;
	bool sensorConfigChanged = true; // First time used, it's config has changed internally

	bool adjustingKinectRepresentationRot = false;
	bool adjustingKinectRepresentationPos = false;

	void updateKinectQuaternion()
	{
		kinectRepRotation = vrmath::quaternionFromYawPitchRoll(kinectRadRotation.v[1], kinectRadRotation.v[0],
		                                                       kinectRadRotation.v[2]);
	}

	//first is cutoff in hz (multiplied per 2PI) and second is our framerate about 100 fps
	bool flip;
	PSMQuatf offset[2];
	Eigen::Quaternionf quatf[2];
	glm::quat move_ori_offset[3];
	glm::vec3 joy[2] = {glm::vec3(0, 0, 0), glm::vec3(0, 0, 0)};
	glm::quat left_tracker_rot, right_tracker_rot, waist_tracker_rot;

	void sendipc()
	{
		LowPassFilter lowPassFilter[3][3] = {
			{LowPassFilter(7.1, 0.005), LowPassFilter(7.1, 0.005), LowPassFilter(7.1, 0.005)},
			{LowPassFilter(7.1, 0.005), LowPassFilter(7.1, 0.005), LowPassFilter(7.1, 0.005)},
			{LowPassFilter(7.1, 0.005), LowPassFilter(7.1, 0.005), LowPassFilter(7.1, 0.005)}
		};

		int n = 3, m = 1; // Number of measurements & statements
		double dt = 1.0 / 30, t[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
		Eigen::MatrixXd A(n, n), C(m, n), Q(n, n), R(m, m), P(n, n);

		A << 1, dt, 0, 0, 1, dt, 0, 0, 1;
		C << 1, 0, 0;

		Q << .05, .05, .0, .05, .05, .0, .0, .0, .0;
		R << 5;
		P << .1, .1, .1, .1, 10000, 10, .1, 10, 100;

		KalmanFilter kalmanFilter[3][3] = {
			{KalmanFilter(dt, A, C, Q, R, P), KalmanFilter(dt, A, C, Q, R, P), KalmanFilter(dt, A, C, Q, R, P)},
			{KalmanFilter(dt, A, C, Q, R, P), KalmanFilter(dt, A, C, Q, R, P), KalmanFilter(dt, A, C, Q, R, P)},
			{KalmanFilter(dt, A, C, Q, R, P), KalmanFilter(dt, A, C, Q, R, P), KalmanFilter(dt, A, C, Q, R, P)}
		};

		Eigen::VectorXd x0(n);
		x0 << 0.f, 0.f, 0.f;

		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				kalmanFilter[i][j].init(0.f, x0);
			}
		}

		Eigen::VectorXd y[3][3] = {
			{Eigen::VectorXd(m), Eigen::VectorXd(m), Eigen::VectorXd(m)},
			{Eigen::VectorXd(m), Eigen::VectorXd(m), Eigen::VectorXd(m)},
			{Eigen::VectorXd(m), Eigen::VectorXd(m), Eigen::VectorXd(m)}
		};

		while (true)
		{
			auto loop_start_time = std::chrono::high_resolution_clock::now();

			if (positional_tracking_option == k_PSMoveFullTracking)
			{
				left_foot_raw_pose = .01f * glm::vec3(left_foot_psmove.Pose.Position.x, left_foot_psmove.Pose.Position.y,
				                             left_foot_psmove.Pose.Position.z);
				right_foot_raw_pose = .01f * glm::vec3(right_foot_psmove.Pose.Position.x, right_foot_psmove.Pose.Position.y,
				                             right_foot_psmove.Pose.Position.z);
				waist_raw_pose = .01f * glm::vec3(waist_psmove.Pose.Position.x, waist_psmove.Pose.Position.y,
				                            waist_psmove.Pose.Position.z);

				left_foot_raw_ori = glm::quat(left_foot_psmove.Pose.Orientation.w, left_foot_psmove.Pose.Orientation.x,
				                     left_foot_psmove.Pose.Orientation.y, left_foot_psmove.Pose.Orientation.z);
				right_foot_raw_ori = glm::quat(right_foot_psmove.Pose.Orientation.w, right_foot_psmove.Pose.Orientation.x,
				                     right_foot_psmove.Pose.Orientation.y, right_foot_psmove.Pose.Orientation.z);
				waist_raw_ori = glm::quat(waist_psmove.Pose.Orientation.w, waist_psmove.Pose.Orientation.x,
				                    waist_psmove.Pose.Orientation.y, waist_psmove.Pose.Orientation.z);
			}

			kinect_m_positions[2].v[0] = waist_raw_pose.x;
			kinect_m_positions[2].v[1] = waist_raw_pose.y;
			kinect_m_positions[2].v[2] = waist_raw_pose.z;
			kinect_m_positions[1].v[0] = left_hand_pose.x;
			kinect_m_positions[1].v[1] = left_hand_pose.y;
			kinect_m_positions[1].v[2] = left_hand_pose.z;
			kinect_m_positions[0].v[0] = head_position.x;
			kinect_m_positions[0].v[1] = head_position.y;
			kinect_m_positions[0].v[2] = head_position.z;

			const glm::vec3 posePrev[3] = {left_foot_raw_pose, right_foot_raw_pose, waist_raw_pose};
			const glm::vec3 poseLast[3] = {lastPose[0][0], lastPose[1][0], lastPose[2][0]};
			const glm::vec3 poseLerp[3] = {
				mix(posePrev[0], poseLast[0], 0.3f),
				mix(posePrev[1], poseLast[1], 0.3f),
				mix(posePrev[2], poseLast[2], 0.3f)
			};
			glm::vec3 poseFiltered[3] = {glm::vec3(0, 0, 0), glm::vec3(0, 0, 0), glm::vec3(0, 0, 0)};

			if (posOption == k_DisablePositionFilter)
			{
				poseFiltered[0] = left_foot_raw_pose;
				poseFiltered[1] = right_foot_raw_pose;
				poseFiltered[2] = waist_raw_pose;
			}
			else if (posOption == k_EnablePositionFilter_LERP)
			{
				poseFiltered[0] = poseLerp[0];
				poseFiltered[1] = poseLerp[1];
				poseFiltered[2] = poseLerp[2];
			}
			else if (posOption == k_EnablePositionFilter_LowPass)
			{
				poseFiltered[0] = glm::vec3(lowPassFilter[0][0].update(left_foot_raw_pose.x),
				                            lowPassFilter[0][1].update(left_foot_raw_pose.y),
				                            lowPassFilter[0][2].update(left_foot_raw_pose.z));
				poseFiltered[1] = glm::vec3(lowPassFilter[1][0].update(right_foot_raw_pose.x),
				                            lowPassFilter[1][1].update(right_foot_raw_pose.y),
				                            lowPassFilter[1][2].update(right_foot_raw_pose.z));
				poseFiltered[2] = glm::vec3(lowPassFilter[2][0].update(waist_raw_pose.x),
				                            lowPassFilter[2][1].update(waist_raw_pose.y),
				                            lowPassFilter[2][2].update(waist_raw_pose.z));
			}
			else if (posOption == k_EnablePositionFilter_Kalman)
			{
				for (int i = 0; i < 3; i++)
				{
					t[0][i] += dt;
					switch (i)
					{
					case 0:
						y[0][i] << left_foot_raw_pose.x;
						kalmanFilter[0][i].update(y[0][i]);
						poseFiltered[0].x = kalmanFilter[0][i].state().x();
						break;
					case 1:
						y[0][i] << left_foot_raw_pose.y;
						kalmanFilter[0][i].update(y[0][i]);
						poseFiltered[0].y = kalmanFilter[0][i].state().x();
						break;
					case 2:
						y[0][i] << left_foot_raw_pose.z;
						kalmanFilter[0][i].update(y[0][i]);
						poseFiltered[0].z = kalmanFilter[0][i].state().x();
						break;
					}
				}

				for (int i = 0; i < 3; i++)
				{
					t[1][i] += dt;
					switch (i)
					{
					case 0:
						y[1][i] << right_foot_raw_pose.x;
						kalmanFilter[1][i].update(y[1][i]);
						poseFiltered[1].x = kalmanFilter[1][i].state().x();
						break;
					case 1:
						y[1][i] << right_foot_raw_pose.y;
						kalmanFilter[1][i].update(y[1][i]);
						poseFiltered[1].y = kalmanFilter[1][i].state().x();
						break;
					case 2:
						y[1][i] << right_foot_raw_pose.z;
						kalmanFilter[1][i].update(y[1][i]);
						poseFiltered[1].z = kalmanFilter[1][i].state().x();
						break;
					}
				}

				for (int i = 0; i < 3; i++)
				{
					t[2][i] += dt;
					switch (i)
					{
					case 0:
						y[2][i] << waist_raw_pose.x;
						kalmanFilter[2][i].update(y[2][i]);
						poseFiltered[2].x = kalmanFilter[2][i].state().x();
						break;
					case 1:
						y[2][i] << waist_raw_pose.y;
						kalmanFilter[2][i].update(y[2][i]);
						poseFiltered[2].y = kalmanFilter[2][i].state().x();
						break;
					case 2:
						y[2][i] << waist_raw_pose.z;
						kalmanFilter[2][i].update(y[2][i]);
						poseFiltered[2].z = kalmanFilter[2][i].state().x();
						break;
					}
				}
			}

			const PSMPSMove left_psmove = left_move_controller, right_psmove = right_move_controller;

			//if (KVR_PSMoves.size() >= 1) {
			//    hidariKontorora = KVR_PSMoves.at(psmh).PSMoveData;
			//}
			//if (KVR_PSMoves.size() >= 2) {
			//    migiKontorora = KVR_PSMoves.at(psmm).PSMoveData;
			//}

			if (right_psmove.SelectButton == PSMButtonState_DOWN) //we are recentering right psmove with select button
				offset[0] = right_psmove.Pose.Orientation; //quaterion for further offset maths

			if (left_psmove.SelectButton == PSMButtonState_DOWN)
				//we are recentering right psmove with select button
				offset[1] = left_psmove.Pose.Orientation; //quaterion for further offset maths


			if (left_foot_psmove.SelectButton == PSMButtonState_DOWN) //recenter left foot move with select button
				move_ori_offset[0] = glm::quat(left_foot_psmove.Pose.Orientation.w,
				                              left_foot_psmove.Pose.Orientation.x, left_foot_psmove.Pose.Orientation.y,
				                              left_foot_psmove.Pose.Orientation.z);

			if (right_foot_psmove.SelectButton == PSMButtonState_DOWN) //recenter right foot move with select button
				move_ori_offset[1] = glm::quat(right_foot_psmove.Pose.Orientation.w,
				                              right_foot_psmove.Pose.Orientation.x, right_foot_psmove.Pose.Orientation.y,
				                              right_foot_psmove.Pose.Orientation.z);

			if (waist_psmove.SelectButton == PSMButtonState_DOWN) //recenter waist move with select button
				move_ori_offset[2] = glm::quat(waist_psmove.Pose.Orientation.w,
				                              waist_psmove.Pose.Orientation.x, waist_psmove.Pose.Orientation.y,
				                              waist_psmove.Pose.Orientation.z);

			using PointSet = Eigen::Matrix<float, 3, Eigen::Dynamic>; //create pointset for korejan's transform algo
			const float yaw = hmdYaw * 180 / M_PI; //get current headset yaw (RAD->DEG)
			const float facing = yaw - calibration_trackers_yaw; //get facing to kinect; 

			// we're subtracting looking at the kinect degree from actual yaw to get offset angle:
			//       
			//             FRONT                 Front is at 0deg
			//              / \     KINECT       Kinect is at 30deg
			//               |       /           
			//               |      /            Assuming we're looking at front, we have facing -30
			//               |     /             because: front:0deg, kinect:30deg -> 0-30 = -30deg        
			//               |    /                             
			//               |   /               flip activates itself if facing is between -155 and -205deg
			//               |  /                and deactivates if facing is between 25 and -25deg
			//              ---                  AND we're not using psms for tracking
			//             CENTER                          
			//              ---                         
			//               |                          
			//               |

			// NOTE! I'm using:
			// Pitch for rotation around +x
			// Yaw for rotation around +y (yes, +)
			// Roll for rotation around +z
			// Just get used to it

			if (positional_tracking_option == k_PSMoveFullTracking)
				flip = false;
			else
			{
				if ((facing <= 25 && facing >= -25) || //if we use -180+180
					(facing <= 25 && facing >= 0 || facing >= 345 && facing <= 360)) //if we use 0+360
					flip = false;
				if ((facing <= -155 && facing >= -205) || //if we use -180+180
					(facing >= 155 && facing <= 205)) //if we use 0+360
					flip = true;
			}

			std::string tracker_data_string = [&]()-> std::string
			{
				std::stringstream S;

				if (hips_rotation_option == k_EnableHipsOrientationFilter)
				{
					if (positional_tracking_option == k_KinectFullTracking)
						waist_tracker_rot = waist_raw_ori;
					else
						waist_tracker_rot = glm::quat(waist_psmove.Pose.Orientation.w, waist_psmove.Pose.Orientation.x,
						                        waist_psmove.Pose.Orientation.y, waist_psmove.Pose.Orientation.z);
				}
				else if (hips_rotation_option == k_DisableHipsOrientationFilter)
					waist_tracker_rot = glm::quat(0, 0, 0, 0);

				// We may be using special orientation filter, apply it
				/*******************************************************/
				if (feet_rotation_option == k_EnableOrientationFilter_Software || kinectVersion == 2)
				{
					glm::quat q = glm::quat(glm::vec3(0.f, M_PI, 0.f));
					if (!flip)
					{
						left_tracker_rot = trackerSoftRot[0] * q;
						right_tracker_rot = trackerSoftRot[1] * q;

						glm::vec3 euler_orientation[2] = {
							eulerAngles(left_tracker_rot),
							eulerAngles(right_tracker_rot)
						};

						// Mirror pitch and roll
						euler_orientation[0].x *= -1;
						euler_orientation[1].x *= -1;
						
						euler_orientation[0].z *= -1;
						euler_orientation[1].z *= -1;

						// Apply to parent
						left_tracker_rot = euler_orientation[0];
						right_tracker_rot = euler_orientation[1];
					}
					else {
						left_tracker_rot = inverse(trackerSoftRot[1]) * q;
						right_tracker_rot = inverse(trackerSoftRot[0]) * q;
					}
				}
				/*******************************************************/
				
				if (feet_rotation_option == k_EnableOrientationFilter)
				{
					if (positional_tracking_option == k_KinectFullTracking)
					{
						// Don't run on v2
						if (kinectVersion == 1) {
							if (!flip)
							{
								left_tracker_rot = left_foot_raw_ori;
								right_tracker_rot = right_foot_raw_ori;
							}
							else
							{
								right_tracker_rot = inverse(left_foot_raw_ori);
								left_tracker_rot = inverse(right_foot_raw_ori);
							}
						}
					}
					else
					{
						left_tracker_rot = glm::quat(left_foot_psmove.Pose.Orientation.w, left_foot_psmove.Pose.Orientation.x,
						                        left_foot_psmove.Pose.Orientation.y, left_foot_psmove.Pose.Orientation.z);
						right_tracker_rot = glm::quat(right_foot_psmove.Pose.Orientation.w, right_foot_psmove.Pose.Orientation.x,
						                        right_foot_psmove.Pose.Orientation.y, right_foot_psmove.Pose.Orientation.z);
					}
				}
				else if (feet_rotation_option == k_EnableOrientationFilter_WithoutYaw)
				{
					if (positional_tracking_option == k_KinectFullTracking)
					{
						if (!flip)
						{
							glm::vec3 left_ori_with_yaw = eulerAngles(left_foot_raw_ori);
							left_tracker_rot = glm::quat(glm::vec3(left_ori_with_yaw.x, 0.f, left_ori_with_yaw.z));
							glm::vec3 right_ori_with_yaw = eulerAngles(right_foot_raw_ori);
							right_tracker_rot = glm::quat(glm::vec3(right_ori_with_yaw.x, 0.f, right_ori_with_yaw.z));
						}
						else
						{
							glm::vec3 left_ori_with_yaw = eulerAngles(left_foot_raw_ori);
							left_tracker_rot = normalize(inverse(glm::quat(glm::vec3(left_ori_with_yaw.x, 0.f, left_ori_with_yaw.z))));
							glm::vec3 right_ori_with_yaw = eulerAngles(right_foot_raw_ori);
							right_tracker_rot = normalize(inverse(glm::quat(glm::vec3(right_ori_with_yaw.x, 0.f, right_ori_with_yaw.z))));
						}
					}
					else
					{
						glm::vec3 left_ori_with_yaw = eulerAngles(glm::quat(left_foot_psmove.Pose.Orientation.w,
						                                           left_foot_psmove.Pose.Orientation.x,
						                                           left_foot_psmove.Pose.Orientation.y,
						                                           left_foot_psmove.Pose.Orientation.z));
						left_tracker_rot = glm::quat(glm::vec3(left_ori_with_yaw.x, 0.f, left_ori_with_yaw.z));
						glm::vec3 right_ori_with_yaw = eulerAngles(glm::quat(right_foot_psmove.Pose.Orientation.w,
						                                           right_foot_psmove.Pose.Orientation.x,
						                                           right_foot_psmove.Pose.Orientation.y,
						                                           right_foot_psmove.Pose.Orientation.z));
						right_tracker_rot = glm::quat(glm::vec3(right_ori_with_yaw.x, 0.f, right_ori_with_yaw.z));
					}
				}
				else if (feet_rotation_option == k_DisableOrientationFilter)
				{
					left_tracker_rot = glm::quat(0, 0, 0, 0);
					right_tracker_rot = glm::quat(0, 0, 0, 0);
				}

				/*******************************************************/
				glm::quat r = glm::vec3(0.f, hmdYaw, 0.f);
				if (feet_rotation_option == k_EnableOrientationFilter_HeadOrientation)
				{
					left_tracker_rot = r;
					right_tracker_rot = r;
				}
				if (hips_rotation_option == k_EnableHipsOrientationFilter_HeadOrientation)
				{
					waist_tracker_rot = r;
				}
				/*******************************************************/
				
				/* Apply offsets to orientations */
				glm::vec3 offset_orientation[3] = {
					eulerAngles(left_tracker_rot),
					eulerAngles(right_tracker_rot),
					eulerAngles(waist_tracker_rot)
				};
				offset_orientation[0] += glm::vec3(manual_offsets[1][1].v[0] * M_PI / 180.f, manual_offsets[1][1].v[1] * M_PI / 180.f,
				                      manual_offsets[1][1].v[2] * M_PI / 180.f);
				offset_orientation[1] += glm::vec3(manual_offsets[1][0].v[0] * M_PI / 180.f, manual_offsets[1][0].v[1] * M_PI / 180.f,
				                      manual_offsets[1][0].v[2] * M_PI / 180.f);
				offset_orientation[2] += glm::vec3(manual_offsets[1][2].v[0] * M_PI / 180.f, manual_offsets[1][2].v[1] * M_PI / 180.f,
				                      manual_offsets[1][2].v[2] * M_PI / 180.f);
				left_tracker_rot = offset_orientation[0];
				right_tracker_rot = offset_orientation[1];
				waist_tracker_rot = offset_orientation[2];

				if (positional_tracking_option == k_KinectFullTracking)
				{
					glm::vec3 kinect_trackers_orientation[3] = { eulerAngles(left_tracker_rot), eulerAngles(right_tracker_rot), eulerAngles(waist_tracker_rot) };
					if (feet_rotation_option != k_EnableOrientationFilter_HeadOrientation) {
						kinect_trackers_orientation[0] += glm::vec3(0.f, calibration_trackers_yaw * M_PI / 180, 0.f);
						kinect_trackers_orientation[1] += glm::vec3(0.f, calibration_trackers_yaw * M_PI / 180, 0.f);
					}
					if (hips_rotation_option != k_EnableHipsOrientationFilter_HeadOrientation)
						kinect_trackers_orientation[2] += glm::vec3(0.f, calibration_trackers_yaw * M_PI / 180, 0.f);

					if (flip)
					{
						bool pitchOn = false;

						if (feet_rotation_option != k_EnableOrientationFilter_HeadOrientation) {
							kinect_trackers_orientation[0] += glm::vec3(0.f, 0.f, M_PI);
							kinect_trackers_orientation[1] += glm::vec3(0.f, 0.f, M_PI);

							left_tracker_rot = glm::vec3(pitchOn * kinect_trackers_orientation[0].x, kinect_trackers_orientation[0].y, kinect_trackers_orientation[0].z);
							right_tracker_rot = glm::vec3(pitchOn * kinect_trackers_orientation[1].x, kinect_trackers_orientation[1].y, kinect_trackers_orientation[1].z);
						}
						if (hips_rotation_option != k_EnableHipsOrientationFilter_HeadOrientation) {
							kinect_trackers_orientation[2] += glm::vec3(0.f, 0.f, M_PI);

							waist_tracker_rot = glm::vec3(pitchOn * kinect_trackers_orientation[2].x, kinect_trackers_orientation[2].y, kinect_trackers_orientation[2].z);
						}
					}
					else
					{
						left_tracker_rot = glm::vec3(kinect_trackers_orientation[0].x, kinect_trackers_orientation[0].y, kinect_trackers_orientation[0].z);
						right_tracker_rot = glm::vec3(kinect_trackers_orientation[1].x, kinect_trackers_orientation[1].y, kinect_trackers_orientation[1].z);
						waist_tracker_rot = glm::vec3(kinect_trackers_orientation[2].x, kinect_trackers_orientation[2].y, kinect_trackers_orientation[2].z);
					}
				}
				else
				{
					left_tracker_rot *= normalize(inverse(move_ori_offset[0]));
					right_tracker_rot *= normalize(inverse(move_ori_offset[1]));
					waist_tracker_rot *= normalize(inverse(move_ori_offset[2]));
				}

				/*******************************************************/
				if (matrixes_calibrated && positional_tracking_option == k_KinectFullTracking && flip)
				{
					glm::quat tune_quat(glm::vec3(-calibration_kinect_pitch / 4, 2 * M_PI, 0.f)),
						tune_quat_w(glm::vec3(0.f, 2 * M_PI, 0.f));
					if (feet_rotation_option == k_EnableOrientationFilter ||
						feet_rotation_option == k_EnableOrientationFilter_WithoutYaw) {

						// Don't run on v2
						if (kinectVersion == 1) {
							left_tracker_rot *= tune_quat;
							right_tracker_rot *= tune_quat;
						}
					}
					if (hips_rotation_option == k_EnableHipsOrientationFilter)
						waist_tracker_rot *= tune_quat_w;
				}
				/*******************************************************/

				if (matrixes_calibrated)
				{
					Eigen::Vector3f left_foot_pose, right_foot_pose, waist_pose;
					if (!flip)
					{
						left_foot_pose(0) = poseFiltered[0].x;
						left_foot_pose(1) = poseFiltered[0].y;
						left_foot_pose(2) = poseFiltered[0].z;

						right_foot_pose(0) = poseFiltered[1].x;
						right_foot_pose(1) = poseFiltered[1].y;
						right_foot_pose(2) = poseFiltered[1].z;
					}
					else
					{
						right_foot_pose(0) = poseFiltered[0].x;
						right_foot_pose(1) = poseFiltered[0].y;
						right_foot_pose(2) = poseFiltered[0].z;

						left_foot_pose(0) = poseFiltered[1].x;
						left_foot_pose(1) = poseFiltered[1].y;
						left_foot_pose(2) = poseFiltered[1].z;
					}

					waist_pose(0) = poseFiltered[2].x;
					waist_pose(1) = poseFiltered[2].y;
					waist_pose(2) = poseFiltered[2].z;

					PointSet left_pose_end = (calibration_rotation * (left_foot_pose - calibration_origin)).colwise() + calibration_translation + calibration_origin;
					PointSet right_pose_end = (calibration_rotation * (right_foot_pose - calibration_origin)).colwise() + calibration_translation + calibration_origin;
					PointSet waist_pose_end = (calibration_rotation * (waist_pose - calibration_origin)).colwise() + calibration_translation + calibration_origin;

					S << "HX" << 10000 * (left_pose_end(0) + manual_offsets[0][1].v[0] + kinect_tracker_offsets.v[0]) <<
						"/HY" << 10000 * (left_pose_end(1) + manual_offsets[0][1].v[1] + kinect_tracker_offsets.v[1]) <<
						"/HZ" << 10000 * (left_pose_end(2) + manual_offsets[0][1].v[2] + kinect_tracker_offsets.v[2]) <<
						"/MX" << 10000 * (right_pose_end(0) + manual_offsets[0][0].v[0] + kinect_tracker_offsets.v[0]) <<
						"/MY" << 10000 * (right_pose_end(1) + manual_offsets[0][0].v[1] + kinect_tracker_offsets.v[1]) <<
						"/MZ" << 10000 * (right_pose_end(2) + manual_offsets[0][0].v[2] + kinect_tracker_offsets.v[2]) <<
						"/PX" << 10000 * (waist_pose_end(0) + manual_offsets[0][2].v[0] + kinect_tracker_offsets.v[0]) <<
						"/PY" << 10000 * (waist_pose_end(1) + manual_offsets[0][2].v[1] + kinect_tracker_offsets.v[1]) <<
						"/PZ" << 10000 * (waist_pose_end(2) + manual_offsets[0][2].v[2] + kinect_tracker_offsets.v[2]) <<
						"/HRW" << 10000 * left_tracker_rot.w <<
						"/HRX" << 10000 * left_tracker_rot.x <<
						"/HRY" << 10000 * left_tracker_rot.y <<
						"/HRZ" << 10000 * left_tracker_rot.z <<
						"/MRW" << 10000 * right_tracker_rot.w <<
						"/MRX" << 10000 * right_tracker_rot.x <<
						"/MRY" << 10000 * right_tracker_rot.y <<
						"/MRZ" << 10000 * right_tracker_rot.z <<
						"/PRW" << 10000 * waist_tracker_rot.w <<
						"/PRX" << 10000 * waist_tracker_rot.x <<
						"/PRY" << 10000 * waist_tracker_rot.y <<
						"/PRZ" << 10000 * waist_tracker_rot.z <<
						"/WRW" << 0 << //DEPRECATED: GLM_ROTATE SCREWED UP WITH > 99
						"/ENABLED" << initialised << "/";
				}
				else
				{
					S << "HX" << 10000 * (poseFiltered[0].x + manual_offsets[0][1].v[0] + kinect_tracker_offsets.v[0]) <<
						"/HY" << 10000 * (poseFiltered[0].y + manual_offsets[0][1].v[1] + kinect_tracker_offsets.v[1]) <<
						"/HZ" << 10000 * (poseFiltered[0].z + manual_offsets[0][1].v[2] + kinect_tracker_offsets.v[2]) <<
						"/MX" << 10000 * (poseFiltered[1].x + manual_offsets[0][0].v[0] + kinect_tracker_offsets.v[0]) <<
						"/MY" << 10000 * (poseFiltered[1].y + manual_offsets[0][0].v[1] + kinect_tracker_offsets.v[1]) <<
						"/MZ" << 10000 * (poseFiltered[1].z + manual_offsets[0][0].v[2] + kinect_tracker_offsets.v[2]) <<
						"/PX" << 10000 * (poseFiltered[2].x + manual_offsets[0][2].v[0] + kinect_tracker_offsets.v[0]) <<
						"/PY" << 10000 * (poseFiltered[2].y + manual_offsets[0][2].v[1] + kinect_tracker_offsets.v[1]) <<
						"/PZ" << 10000 * (poseFiltered[2].z + manual_offsets[0][2].v[2] + kinect_tracker_offsets.v[2]) <<
						"/HRW" << 10000 * left_tracker_rot.w <<
						"/HRX" << 10000 * left_tracker_rot.x <<
						"/HRY" << 10000 * left_tracker_rot.y <<
						"/HRZ" << 10000 * left_tracker_rot.z <<
						"/MRW" << 10000 * right_tracker_rot.w <<
						"/MRX" << 10000 * right_tracker_rot.x <<
						"/MRY" << 10000 * right_tracker_rot.y <<
						"/MRZ" << 10000 * right_tracker_rot.z <<
						"/PRW" << 10000 * waist_tracker_rot.w <<
						"/PRX" << 10000 * waist_tracker_rot.x <<
						"/PRY" << 10000 * waist_tracker_rot.y <<
						"/PRZ" << 10000 * waist_tracker_rot.z <<
						"/WRW" << 0 << //DEPRECATED: GLM_ROTATE SCREWED UP WITH > 99
						"/ENABLED" << initialised << "/";
				}

				return S.str();
			}();
			
			char tracker_data_char[1024];
			strcpy_s(tracker_data_char, tracker_data_string.c_str());

			const HANDLE server_pipe_handle = CreateFile(
				TEXT("\\\\.\\pipe\\LogPipeTracker"), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0,
				nullptr);
			
			DWORD written;
			WriteFile(server_pipe_handle, tracker_data_char, sizeof(tracker_data_char), &written, nullptr);
			CloseHandle(server_pipe_handle);

			// Wait until certain time has passed
			auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
				std::chrono::high_resolution_clock::now() - loop_start_time).count();
			if (duration <= 9000000.f)
			{
				std::this_thread::sleep_for(std::chrono::nanoseconds(9000000 - duration));
			}
		}
	}

	void serializeKinectSettings()
	{
		std::ifstream is(KVR::fileToDirPath(CFG_NAME));
		LOG(INFO) << "Attempted CFG load: " << KVR::fileToDirPath(CFG_NAME) << '\n';
		//CHECK IF VALID
		if (is.fail())
		{
			//FAIL!!!!
			LOG(ERROR) << "ERROR: COULD NOT OPEN CONFIG FILE, GENERATING NEW ONE...";
			writeKinectSettings();
		}
		else
		{
			LOG(INFO) << "CFG Loaded Attempted!";

			using namespace KinectSettings;
			using namespace SFMLsettings;
			float rot[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
			float pos[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
			double hipHeight = 0;
			float fontScale = 12.f;

			try
			{
				cereal::JSONInputArchive archive(is);
				archive(rot);
				archive(pos);
				archive(hipHeight);
				archive(globalFontSize);
				archive(secondaryTrackingOriginOffset);
			}
			catch (cereal::RapidJSONException& e)
			{
				LOG(ERROR) << "CONFIG FILE LOAD JSON ERROR: " << e.what();
			}

			manual_offsets[0][0] = {pos[0][0], pos[0][1], pos[0][2]};
			manual_offsets[0][1] = {pos[1][0], pos[1][1], pos[1][2]};
			manual_offsets[0][2] = {pos[2][0], pos[2][1], pos[2][2]};

			manual_offsets[1][0] = {rot[0][0], rot[0][1], rot[0][2]};
			manual_offsets[1][1] = {rot[1][0], rot[1][1], rot[1][2]};
			manual_offsets[1][2] = {rot[2][0], rot[2][1], rot[2][2]};

			hipRoleHeightAdjust = hipHeight;
			sensorConfigChanged = true;
		}
	}

	void writeKinectSettings()
	{
		std::ofstream os(KVR::fileToDirPath(CFG_NAME));
		if (os.fail())
		{
			//FAIL!!!
			LOG(ERROR) << "ERROR: COULD NOT WRITE TO CONFIG FILE\n";
		}
		else
		{
			using namespace KinectSettings;
			using namespace SFMLsettings;
			float kRotation[3][3] = {
				{manual_offsets[1][0].v[0], manual_offsets[1][0].v[1], manual_offsets[1][0].v[2]},
				{manual_offsets[1][1].v[0], manual_offsets[1][1].v[1], manual_offsets[1][1].v[2]},
				{manual_offsets[1][2].v[0], manual_offsets[1][2].v[1], manual_offsets[1][2].v[2]},
			};

			float kPosition[3][3] = {
				{manual_offsets[0][0].v[0], manual_offsets[0][0].v[1], manual_offsets[0][0].v[2]},
				{manual_offsets[0][1].v[0], manual_offsets[0][1].v[1], manual_offsets[0][1].v[2]},
				{manual_offsets[0][2].v[0], manual_offsets[0][2].v[1], manual_offsets[0][2].v[2]},
			};

			cereal::JSONOutputArchive archive(os);
			LOG(INFO) << "Attempted to save config settings to file";
			try
			{
				archive(
					CEREAL_NVP(kRotation),
					CEREAL_NVP(kPosition),
					CEREAL_NVP(hipRoleHeightAdjust),
					CEREAL_NVP(globalFontSize),
					CEREAL_NVP(secondaryTrackingOriginOffset)
				);
			}
			catch (cereal::RapidJSONException& e)
			{
				LOG(ERROR) << "CONFIG FILE SAVE JSON ERROR: " << e.what();
			}
		}
	}
}

namespace SFMLsettings
{
	int m_window_width = 800;
	int m_window_height = 600;
	float windowScale = .4f;
	bool keepRunning = true;

	float globalFontSize = 12.f;

	std::wstring fileDirectoryPath;

	bool usingGamepad = false;
	std::stringstream debugDisplayTextStream;
}

namespace vr
{
	bool operator==(const HmdVector3d_t& lhs, const HmdVector3d_t& rhs)
	{
		return lhs.v[0] == rhs.v[0]
			&& lhs.v[1] == rhs.v[1]
			&& lhs.v[2] == rhs.v[2];
	}

	bool operator==(const HmdQuaternion_t& lhs, const HmdQuaternion_t& rhs)
	{
		return lhs.w == rhs.w
			&& lhs.x == rhs.x
			&& lhs.y == rhs.y
			&& lhs.z == rhs.z;
	}
}

namespace KVR
{
	std::wstring trackerConfig = L"lastTrackers.cfg";

	std::wstring fileToDirPath(const std::wstring& relativeFilePath)
	{
		CreateDirectory(std::wstring(std::wstring(_wgetenv(L"APPDATA")) + std::wstring(L"\\KinectToVR\\")).c_str(),
		                nullptr);
		return std::wstring(_wgetenv(L"APPDATA")) + L"\\KinectToVR\\" + relativeFilePath;
	}

	std::wstring ToUTF16(const std::string& data)
	{
		return std::wstring_convert<std::codecvt_utf8<wchar_t>>().from_bytes(data);
	}

	std::string ToUTF8(const std::wstring& data)
	{
		return std::wstring_convert<std::codecvt_utf8<wchar_t>>().to_bytes(data);
	}

	std::string inputDirForOpenVR(const std::string& file)
	{
		std::string pathStr = ToUTF8(SFMLsettings::fileDirectoryPath) + "Input\\" + file;
		std::cout << file << " PATH: " << pathStr << '\n';
		return pathStr;
	}

	TrackingSystemCalibration retrieveSystemCalibration(const std::string& systemName)
	{
		std::wstring trackingSystemConfig = ToUTF16(systemName) + L".tracking";
		std::ifstream is(fileToDirPath(trackingSystemConfig));
		LOG(INFO) << "Attempted tracking system load: " << fileToDirPath(trackingSystemConfig) << '\n';

		TrackingSystemCalibration calibration;

		//CHECK IF VALID
		if (is.fail())
		{
			//FAIL!!!!
			LOG(ERROR) << "ERROR: COULD NOT OPEN " << systemName << " TRACKING FILE, GENERATING NEW ONE...";
			saveSystemCalibration(systemName, calibration);
		}
		else
		{
			LOG(INFO) << systemName << "Tracking Load Attempted!";

			vr::HmdQuaternion_t driverFromWorldRotation = {1, 0, 0, 0};
			vr::HmdVector3d_t driverFromWorldPosition = {0, 0, 0};

			try
			{
				cereal::JSONInputArchive archive(is);
				archive(CEREAL_NVP(driverFromWorldRotation));
				archive(CEREAL_NVP(driverFromWorldPosition));
			}
			catch (cereal::Exception& e)
			{
				LOG(ERROR) << systemName << "TRACKING FILE LOAD JSON ERROR: " << e.what();
			}

			calibration.systemName = systemName;
			calibration.driverFromWorldRotation = driverFromWorldRotation;
			calibration.driverFromWorldPosition = driverFromWorldPosition;
		}
		return calibration;
	}

	void saveSystemCalibration(const std::string& systemName, TrackingSystemCalibration calibration)
	{
		std::wstring trackingSystemConfig = ToUTF16(systemName) + L".tracking";
		std::ofstream os(fileToDirPath(trackingSystemConfig));
		if (os.fail())
		{
			//FAIL!!!
			LOG(ERROR) << "ERROR: COULD NOT WRITE TO TRACKING SYSTEM FILE\n";
		}
		else
		{
			cereal::JSONOutputArchive archive(os);
			LOG(INFO) << "Attempted to save " << systemName << " tracking system to file";

			vr::HmdQuaternion_t driverFromWorldRotation = calibration.driverFromWorldRotation;
			vr::HmdVector3d_t driverFromWorldPosition = calibration.driverFromWorldPosition;

			try
			{
				archive(CEREAL_NVP(driverFromWorldRotation));
				archive(CEREAL_NVP(driverFromWorldPosition));
			}
			catch (cereal::RapidJSONException& e)
			{
				LOG(ERROR) << systemName << "TRACKING FILE SAVE JSON ERROR: " << e.what();
			}
		}
	}
}

# define M_PI           3.14159265358979323846

namespace VRInput
{
	bool legacyInputModeEnabled;

	// Action Handles
	vr::VRActionHandle_t moveHorizontallyHandle;
	vr::VRActionHandle_t moveVerticallyHandle;
	vr::VRActionHandle_t confirmCalibrationHandle;

	// Calibration Sets
	vr::VRActionSetHandle_t calibrationSetHandle;

	// Action Sets
	vr::VRActiveActionSet_t activeActionSet;

	// Digital Action Data
	vr::InputDigitalActionData_t confirmCalibrationData{};

	// Analog Action Data
	vr::InputAnalogActionData_t moveHorizontallyData{};
	vr::InputAnalogActionData_t moveVerticallyData{};

	vr::InputAnalogActionData_t trackpadpose[2]{};
	vr::InputDigitalActionData_t confirmdatapose{};
}

bool VRInput::initialiseVRInput()
{
	std::string path = KVR::inputDirForOpenVR("action-manifest.json");
	const char* c_path = path.c_str();
	vr::EVRInputError iError = vr::VRInput()->SetActionManifestPath(c_path);
	if (iError == vr::EVRInputError::VRInputError_None)
	{
		LOG(INFO) << "Action manifest path set correctly!";
	}
	else
	{
		LOG(ERROR) << "Action manifest path Error, EVRInputError Code: " << static_cast<int>(iError);
		return false;
	}
	// Obtain handles
	iError = vr::VRInput()->GetActionHandle("/actions/calibration/in/MoveHorizontally", &moveHorizontallyHandle);
	iError = vr::VRInput()->GetActionHandle("/actions/calibration/in/MoveVertically", &moveVerticallyHandle);
	iError = vr::VRInput()->GetActionHandle("/actions/calibration/in/ConfirmCalibration", &confirmCalibrationHandle);

	iError = vr::VRInput()->GetActionSetHandle("/actions/calibration", &calibrationSetHandle);

	// Set Actionset Settings
	activeActionSet.ulActionSet = calibrationSetHandle;
	activeActionSet.ulRestrictedToDevice = vr::k_ulInvalidInputValueHandle;
	activeActionSet.unPadding; // Ignored
	activeActionSet.nPriority = 0;

	if (iError == vr::EVRInputError::VRInputError_None)
	{
		LOG(INFO) << "Input Handles set correctly!";
	}
	else
	{
		LOG(ERROR) << "Input Handle Error, EVRInputError Code: " << static_cast<int>(iError);
		return false;
	}
	return true;
}

void VRInput::updateVRInput()
{
	vr::EVRInputError iError = vr::VRInput()->UpdateActionState(&activeActionSet, sizeof(activeActionSet), 1);
	if (iError != vr::EVRInputError::VRInputError_None)
	{
		LOG(ERROR) << "Error when updating input action state, EVRInputError Code: " << static_cast<int>(iError);
		return;
	}
	vr::InputAnalogActionData_t moveHorizontallyData{};
	iError = vr::VRInput()->GetAnalogActionData(
		moveHorizontallyHandle,
		&moveHorizontallyData,
		sizeof(moveHorizontallyData),
		vr::k_ulInvalidInputValueHandle);

	vr::InputAnalogActionData_t moveVerticallyData{};
	iError = vr::VRInput()->GetAnalogActionData(
		moveVerticallyHandle,
		&moveVerticallyData,
		sizeof(moveVerticallyData),
		vr::k_ulInvalidInputValueHandle);

	vr::InputDigitalActionData_t confirmPosData{};
	iError = vr::VRInput()->GetDigitalActionData(
		confirmCalibrationHandle,
		&confirmPosData,
		sizeof(confirmPosData),
		vr::k_ulInvalidInputValueHandle);

	// Ugly Hack until Valve fixes this behaviour ---------
	if (iError == vr::EVRInputError::VRInputError_InvalidHandle)
	{
		// SteamVR's latest wonderful bug/feature:
		// Switches to Legacy mode on any application it doesn't recognize
		// Meaning that the new system isn't used at all...
		// Why god. Why do you taunt me so?
		legacyInputModeEnabled = true;
	}
	else
	{
		legacyInputModeEnabled = false;
	}
	// -----------------------------------------------------
}
