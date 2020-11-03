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
	bool psmbuttons[5][10];
	bool isKinectDrawn = false;
	bool isSkeletonDrawn = false;
	bool isDriverPresent = false;
	float svrhmdyaw = 0, kinpitch = 0;
	int psmh, psmm;
	int K2Drivercode = -1; //unknown
	std::vector<int> psmindexidpsm[2];
	int flashnow[2];
	bool conActivated = false;
	bool ignoreInferredPositions = false;
	bool ignoreRotationSmoothing = false;
	float ardroffset = 0.f;
	int bodytrackingoption = 1, headtrackingoption = 1;
	// The joints which actually have rotation change based on the kinect
	// Each kinect type should set these in their process beginning
	// These would be the defaults for the V1
	KVR::KinectJointType leftFootJointWithRotation = KVR::KinectJointType::FootLeft;
	KVR::KinectJointType rightFootJointWithRotation = KVR::KinectJointType::FootRight;
	KVR::KinectJointType leftFootJointWithoutRotation = KVR::KinectJointType::AnkleLeft;
	KVR::KinectJointType rightFootJointWithoutRotation = KVR::KinectJointType::AnkleRight;
	bool isCalibrating = false;

	PSMPSMove migiMove, hidariMove, hidariashimove, migiashimove, yobumove, atamamove;
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
	bool expcalib = false;
	bool frame1 = true;
	int footOption, hipsOption, posOption = 2, conOption;

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

	glm::vec3 hmdPose, hHandPose, mHandPose, hFootPose, mFootPose, hipsPose, hElPose, mElPose,
	          lastPose[3][2];
	glm::quat hFootRot, mFootRot, hipsRot;
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
	vr::HmdVector3d_t moffsets[2][3] = {{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}};
	vr::HmdVector3d_t hoffsets{0, 0, 0};
	vr::HmdVector3d_t huoffsets{0, 0, 0};
	vr::HmdVector3d_t troffsets{0, 0, 0};
	float hroffset = 0;
	float troffset = 0;
	Eigen::Vector3f calorigin;
	vr::TrackedDevicePose_t controllersPose[2];
	vr::HmdVector3d_t hauoffset{0, 0, 0}, mauoffset{0, 0, 0};
	Eigen::Matrix<float, 3, 3> R_matT;
	Eigen::Matrix<float, 3, 1> T_matT;
	bool ismatrixcalibrated = false;
	bool rtcalibrated = false;

	float tryaw = 0.0;
	bool jcalib;
	int cpoints = 3;

	vr::HmdQuaternion_t hmdquat{1, 0, 0, 0};

	vr::HmdVector3d_t mposes[3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

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
	glm::quat btrackeroffset[3];
	glm::vec3 joy[2] = {glm::vec3(0, 0, 0), glm::vec3(0, 0, 0)};
	glm::quat trackerRoth, trackerRotm, trackerRoty;

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

		KalmanFilter posef[3][3] = {
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
				posef[i][j].init(0.f, x0);
			}
		}

		Eigen::VectorXd y[3][3] = {
			{Eigen::VectorXd(m), Eigen::VectorXd(m), Eigen::VectorXd(m)},
			{Eigen::VectorXd(m), Eigen::VectorXd(m), Eigen::VectorXd(m)},
			{Eigen::VectorXd(m), Eigen::VectorXd(m), Eigen::VectorXd(m)}
		};

		while (true)
		{
			auto t1 = std::chrono::high_resolution_clock::now();

			if (bodytrackingoption == k_PSMoveFullTracking)
			{
				hFootPose = .01f * glm::vec3(hidariashimove.Pose.Position.x, hidariashimove.Pose.Position.y,
				                             hidariashimove.Pose.Position.z);
				mFootPose = .01f * glm::vec3(migiashimove.Pose.Position.x, migiashimove.Pose.Position.y,
				                             migiashimove.Pose.Position.z);
				hipsPose = .01f * glm::vec3(yobumove.Pose.Position.x, yobumove.Pose.Position.y,
				                            yobumove.Pose.Position.z);

				hFootRot = glm::quat(hidariashimove.Pose.Orientation.w, hidariashimove.Pose.Orientation.x,
				                     hidariashimove.Pose.Orientation.y, hidariashimove.Pose.Orientation.z);
				mFootRot = glm::quat(migiashimove.Pose.Orientation.w, migiashimove.Pose.Orientation.x,
				                     migiashimove.Pose.Orientation.y, migiashimove.Pose.Orientation.z);
				hipsRot = glm::quat(yobumove.Pose.Orientation.w, yobumove.Pose.Orientation.x,
				                    yobumove.Pose.Orientation.y, yobumove.Pose.Orientation.z);
			}

			mposes[2].v[0] = hipsPose.x;
			mposes[2].v[1] = hipsPose.y;
			mposes[2].v[2] = hipsPose.z;
			mposes[1].v[0] = hHandPose.x;
			mposes[1].v[1] = hHandPose.y;
			mposes[1].v[2] = hHandPose.z;
			mposes[0].v[0] = hmdPose.x;
			mposes[0].v[1] = hmdPose.y;
			mposes[0].v[2] = hmdPose.z;

			const glm::vec3 posePrev[3] = {hFootPose, mFootPose, hipsPose};
			const glm::vec3 poseLast[3] = {lastPose[0][0], lastPose[1][0], lastPose[2][0]};
			const glm::vec3 poseLerp[3] = {
				mix(posePrev[0], poseLast[0], 0.3f),
				mix(posePrev[1], poseLast[1], 0.3f),
				mix(posePrev[2], poseLast[2], 0.3f)
			};
			glm::vec3 poseFiltered[3] = {glm::vec3(0, 0, 0), glm::vec3(0, 0, 0), glm::vec3(0, 0, 0)};

			if (posOption == k_DisablePositionFilter)
			{
				poseFiltered[0] = hFootPose;
				poseFiltered[1] = mFootPose;
				poseFiltered[2] = hipsPose;
			}
			else if (posOption == k_EnablePositionFilter_LERP)
			{
				poseFiltered[0] = poseLerp[0];
				poseFiltered[1] = poseLerp[1];
				poseFiltered[2] = poseLerp[2];
			}
			else if (posOption == k_EnablePositionFilter_LowPass)
			{
				poseFiltered[0] = glm::vec3(lowPassFilter[0][0].update(hFootPose.x),
				                            lowPassFilter[0][1].update(hFootPose.y),
				                            lowPassFilter[0][2].update(hFootPose.z));
				poseFiltered[1] = glm::vec3(lowPassFilter[1][0].update(mFootPose.x),
				                            lowPassFilter[1][1].update(mFootPose.y),
				                            lowPassFilter[1][2].update(mFootPose.z));
				poseFiltered[2] = glm::vec3(lowPassFilter[2][0].update(hipsPose.x),
				                            lowPassFilter[2][1].update(hipsPose.y),
				                            lowPassFilter[2][2].update(hipsPose.z));
			}
			else if (posOption == k_EnablePositionFilter_Kalman)
			{
				for (int i = 0; i < 3; i++)
				{
					t[0][i] += dt;
					switch (i)
					{
					case 0:
						y[0][i] << hFootPose.x;
						posef[0][i].update(y[0][i]);
						poseFiltered[0].x = posef[0][i].state().x();
						break;
					case 1:
						y[0][i] << hFootPose.y;
						posef[0][i].update(y[0][i]);
						poseFiltered[0].y = posef[0][i].state().x();
						break;
					case 2:
						y[0][i] << hFootPose.z;
						posef[0][i].update(y[0][i]);
						poseFiltered[0].z = posef[0][i].state().x();
						break;
					}
				}

				for (int i = 0; i < 3; i++)
				{
					t[1][i] += dt;
					switch (i)
					{
					case 0:
						y[1][i] << mFootPose.x;
						posef[1][i].update(y[1][i]);
						poseFiltered[1].x = posef[1][i].state().x();
						break;
					case 1:
						y[1][i] << mFootPose.y;
						posef[1][i].update(y[1][i]);
						poseFiltered[1].y = posef[1][i].state().x();
						break;
					case 2:
						y[1][i] << mFootPose.z;
						posef[1][i].update(y[1][i]);
						poseFiltered[1].z = posef[1][i].state().x();
						break;
					}
				}

				for (int i = 0; i < 3; i++)
				{
					t[2][i] += dt;
					switch (i)
					{
					case 0:
						y[2][i] << hipsPose.x;
						posef[2][i].update(y[2][i]);
						poseFiltered[2].x = posef[2][i].state().x();
						break;
					case 1:
						y[2][i] << hipsPose.y;
						posef[2][i].update(y[2][i]);
						poseFiltered[2].y = posef[2][i].state().x();
						break;
					case 2:
						y[2][i] << hipsPose.z;
						posef[2][i].update(y[2][i]);
						poseFiltered[2].z = posef[2][i].state().x();
						break;
					}
				}
			}

			PSMPSMove hidariKontorora = hidariMove, migiKontorora = migiMove;

			//if (KVR_PSMoves.size() >= 1) {
			//    hidariKontorora = KVR_PSMoves.at(psmh).PSMoveData;
			//}
			//if (KVR_PSMoves.size() >= 2) {
			//    migiKontorora = KVR_PSMoves.at(psmm).PSMoveData;
			//}

			if (migiKontorora.SelectButton == PSMButtonState_DOWN) //we are recentering right psmove with select button
				offset[0] = migiKontorora.Pose.Orientation; //quaterion for further offset maths

			if (hidariKontorora.SelectButton == PSMButtonState_DOWN)
				//we are recentering right psmove with select button
				offset[1] = hidariKontorora.Pose.Orientation; //quaterion for further offset maths


			if (hidariashimove.SelectButton == PSMButtonState_DOWN) //recenter left foot move with select button
				btrackeroffset[0] = glm::quat(hidariashimove.Pose.Orientation.w,
				                              hidariashimove.Pose.Orientation.x, hidariashimove.Pose.Orientation.y,
				                              hidariashimove.Pose.Orientation.z);

			if (migiashimove.SelectButton == PSMButtonState_DOWN) //recenter right foot move with select button
				btrackeroffset[1] = glm::quat(migiashimove.Pose.Orientation.w,
				                              migiashimove.Pose.Orientation.x, migiashimove.Pose.Orientation.y,
				                              migiashimove.Pose.Orientation.z);

			if (yobumove.SelectButton == PSMButtonState_DOWN) //recenter waist move with select button
				btrackeroffset[2] = glm::quat(yobumove.Pose.Orientation.w,
				                              yobumove.Pose.Orientation.x, yobumove.Pose.Orientation.y,
				                              yobumove.Pose.Orientation.z);

			using PointSet = Eigen::Matrix<float, 3, Eigen::Dynamic>; //create pointset for korejan's transform algo
			float yaw = hmdYaw * 180 / M_PI; //get current headset yaw (RAD->DEG)
			float facing = yaw - tryaw; //get facing to kinect; 

			//we're subtracting looking at the kinect degree from actual yaw to get offset angle:
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

			if (bodytrackingoption == k_PSMoveFullTracking)
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

			std::string TrackerS = [&]()-> std::string
			{
				std::stringstream S;

				if (hipsOption == k_EnableHipsOrientationFilter)
				{
					if (bodytrackingoption == k_KinectFullTracking)
						trackerRoty = hipsRot;
					else
						trackerRoty = glm::quat(yobumove.Pose.Orientation.w, yobumove.Pose.Orientation.x,
						                        yobumove.Pose.Orientation.y, yobumove.Pose.Orientation.z);
				}
				else if (hipsOption == k_EnableHipsOrientationFilter_HeadOrientation)
					trackerRoty = glm::quat(hmdRot.w, hmdRot.x, hmdRot.y, hmdRot.z);
				else if (hipsOption == k_DisableHipsOrientationFilter)
					trackerRoty = glm::quat(0, 0, 0, 0);

				glm::quat footrot[2] = {hFootRot, mFootRot};
				if (footOption == k_EnableOrientationFilter)
				{
					if (bodytrackingoption == k_KinectFullTracking)
					{
						if (!flip)
						{
							trackerRoth = hFootRot;
							trackerRotm = mFootRot;
						}
						else
						{
							trackerRotm = inverse(hFootRot);
							trackerRoth = inverse(mFootRot);
						}
					}
					else
					{
						trackerRoth = glm::quat(hidariashimove.Pose.Orientation.w, hidariashimove.Pose.Orientation.x,
						                        hidariashimove.Pose.Orientation.y, hidariashimove.Pose.Orientation.z);
						trackerRotm = glm::quat(migiashimove.Pose.Orientation.w, migiashimove.Pose.Orientation.x,
						                        migiashimove.Pose.Orientation.y, migiashimove.Pose.Orientation.z);
					}
				}
				else if (footOption == k_EnableOrientationFilter_WithoutYaw)
				{
					if (bodytrackingoption == k_KinectFullTracking)
					{
						if (!flip)
						{
							glm::vec3 hwithyaw = eulerAngles(hFootRot);
							trackerRoth = glm::quat(glm::vec3(hwithyaw.x, 0.f, hwithyaw.z));
							glm::vec3 mwithyaw = eulerAngles(mFootRot);
							trackerRotm = glm::quat(glm::vec3(mwithyaw.x, 0.f, mwithyaw.z));
						}
						else
						{
							glm::vec3 hwithyaw = eulerAngles(hFootRot);
							trackerRoth = normalize(inverse(glm::quat(glm::vec3(hwithyaw.x, 0.f, hwithyaw.z))));
							glm::vec3 mwithyaw = eulerAngles(mFootRot);
							trackerRotm = normalize(inverse(glm::quat(glm::vec3(mwithyaw.x, 0.f, mwithyaw.z))));
						}
					}
					else
					{
						glm::vec3 hwithyaw = eulerAngles(glm::quat(hidariashimove.Pose.Orientation.w,
						                                           hidariashimove.Pose.Orientation.x,
						                                           hidariashimove.Pose.Orientation.y,
						                                           hidariashimove.Pose.Orientation.z));
						trackerRoth = glm::quat(glm::vec3(hwithyaw.x, 0.f, hwithyaw.z));
						glm::vec3 mwithyaw = eulerAngles(glm::quat(migiashimove.Pose.Orientation.w,
						                                           migiashimove.Pose.Orientation.x,
						                                           migiashimove.Pose.Orientation.y,
						                                           migiashimove.Pose.Orientation.z));
						trackerRotm = glm::quat(glm::vec3(mwithyaw.x, 0.f, mwithyaw.z));
					}
				}
				else if (footOption == k_DisableOrientationFilter)
				{
					trackerRoth = glm::quat(0, 0, 0, 0);
					trackerRotm = glm::quat(0, 0, 0, 0);
				}
				else if (footOption == k_EnableOrientationFilter_HeadOrientation)
				{
					glm::vec3 hmdR = eulerAngles(glm::quat(hmdRot.w, hmdRot.x, hmdRot.y, hmdRot.z));
					trackerRoth = glm::vec3(0.f, hmdR.y, 0.f);
					trackerRotm = glm::vec3(0.f, hmdR.y, 0.f);
				}

				glm::vec3 unofu[3] = {eulerAngles(trackerRoth), eulerAngles(trackerRotm), eulerAngles(trackerRoty)};
				unofu[0] += glm::vec3(moffsets[1][1].v[0] * M_PI / 180.f, moffsets[1][1].v[1] * M_PI / 180.f,
				                      moffsets[1][1].v[2] * M_PI / 180.f);
				unofu[1] += glm::vec3(moffsets[1][0].v[0] * M_PI / 180.f, moffsets[1][0].v[1] * M_PI / 180.f,
				                      moffsets[1][0].v[2] * M_PI / 180.f);
				unofu[2] += glm::vec3(moffsets[1][2].v[0] * M_PI / 180.f, moffsets[1][2].v[1] * M_PI / 180.f,
				                      moffsets[1][2].v[2] * M_PI / 180.f);
				trackerRoth = unofu[0];
				trackerRotm = unofu[1];
				trackerRoty = unofu[2];

				if (bodytrackingoption == k_KinectFullTracking &&
					footOption != k_EnableOrientationFilter_HeadOrientation)
				{
					glm::vec3 unofu[3] = {eulerAngles(trackerRoth), eulerAngles(trackerRotm), eulerAngles(trackerRoty)};
					unofu[0] += glm::vec3(0.f, tryaw * M_PI / 180, 0.f);
					unofu[1] += glm::vec3(0.f, tryaw * M_PI / 180, 0.f);
					unofu[2] += glm::vec3(0.f, tryaw * M_PI / 180, 0.f);
					if (flip)
					{
						//unofu[0] += glm::vec3(0.f, M_PI, M_PI);
						//unofu[1] += glm::vec3(0.f, M_PI, M_PI);
						//unofu[2] += glm::vec3(0.f, M_PI, M_PI);

						unofu[0] += glm::vec3(0.f, 0.f, M_PI);
						unofu[1] += glm::vec3(0.f, 0.f, M_PI);
						unofu[2] += glm::vec3(0.f, 0.f, M_PI);

						trackerRoth = glm::vec3(/*unofu[0].x*/ 0.f, unofu[0].y, unofu[0].z);
						trackerRotm = glm::vec3(/*unofu[1].x*/ 0.f, unofu[1].y, unofu[1].z);
						trackerRoty = glm::vec3(/*unofu[2].x*/ 0.f, unofu[2].y, unofu[2].z);
					}
					else
					{
						trackerRoth = glm::vec3(unofu[0].x, unofu[0].y, unofu[0].z);
						trackerRotm = glm::vec3(unofu[1].x, unofu[1].y, unofu[1].z);
						trackerRoty = glm::vec3(unofu[2].x, unofu[2].y, unofu[2].z);
					}
				}
				else
				{
					trackerRoth *= normalize(inverse(btrackeroffset[0]));
					trackerRotm *= normalize(inverse(btrackeroffset[1]));
					trackerRoty *= normalize(inverse(btrackeroffset[2]));
				}

				if (rtcalibrated)
				{
					Eigen::Vector3f Hf, Mf, Hp;
					if (!flip)
					{
						Hf(0) = poseFiltered[0].x;
						Hf(1) = poseFiltered[0].y;
						Hf(2) = poseFiltered[0].z;

						Mf(0) = poseFiltered[1].x;
						Mf(1) = poseFiltered[1].y;
						Mf(2) = poseFiltered[1].z;
					}
					else
					{
						Mf(0) = poseFiltered[0].x;
						Mf(1) = poseFiltered[0].y;
						Mf(2) = poseFiltered[0].z;

						Hf(0) = poseFiltered[1].x;
						Hf(1) = poseFiltered[1].y;
						Hf(2) = poseFiltered[1].z;
					}

					Hp(0) = poseFiltered[2].x;
					Hp(1) = poseFiltered[2].y;
					Hp(2) = poseFiltered[2].z;

					PointSet Hf2 = (R_matT * (Hf - calorigin)).colwise() + T_matT + calorigin;
					PointSet Mf2 = (R_matT * (Mf - calorigin)).colwise() + T_matT + calorigin;
					PointSet Hp2 = (R_matT * (Hp - calorigin)).colwise() + T_matT + calorigin;

					/*******************************************************/
					if (bodytrackingoption == k_KinectFullTracking &&
						footOption != k_EnableOrientationFilter_HeadOrientation &&
						flip)
					{
						glm::quat qy_quat(glm::vec3(-glm::radians(kinpitch) / 2, 2 * M_PI, 0.f)),
							qy_quat_hips(glm::vec3(0.f, 2 * M_PI, 0.f));
						trackerRoth *= qy_quat;
						trackerRotm *= qy_quat;
						trackerRoty *= qy_quat_hips;
					}
					/*******************************************************/

					S << "HX" << 10000 * (Hf2(0) + moffsets[0][1].v[0] + troffsets.v[0]) <<
						"/HY" << 10000 * (Hf2(1) + moffsets[0][1].v[1] + troffsets.v[1]) <<
						"/HZ" << 10000 * (Hf2(2) + moffsets[0][1].v[2] + troffsets.v[2]) <<
						"/MX" << 10000 * (Mf2(0) + moffsets[0][0].v[0] + troffsets.v[0]) <<
						"/MY" << 10000 * (Mf2(1) + moffsets[0][0].v[1] + troffsets.v[1]) <<
						"/MZ" << 10000 * (Mf2(2) + moffsets[0][0].v[2] + troffsets.v[2]) <<
						"/PX" << 10000 * (Hp2(0) + moffsets[0][2].v[0] + troffsets.v[0]) <<
						"/PY" << 10000 * (Hp2(1) + moffsets[0][2].v[1] + troffsets.v[1]) <<
						"/PZ" << 10000 * (Hp2(2) + moffsets[0][2].v[2] + troffsets.v[2]) <<
						"/HRW" << 10000 * (trackerRoth.w) <<
						"/HRX" << 10000 * (trackerRoth.x) <<
						"/HRY" << 10000 * (trackerRoth.y) <<
						"/HRZ" << 10000 * (trackerRoth.z) <<
						"/MRW" << 10000 * (trackerRotm.w) <<
						"/MRX" << 10000 * (trackerRotm.x) <<
						"/MRY" << 10000 * (trackerRotm.y) <<
						"/MRZ" << 10000 * (trackerRotm.z) <<
						"/PRW" << 10000 * (trackerRoty.w) <<
						"/PRX" << 10000 * (trackerRoty.x) <<
						"/PRY" << 10000 * (trackerRoty.y) <<
						"/PRZ" << 10000 * (trackerRoty.z) <<
						"/WRW" << 10000 * (0) << //DEPRECATED: GLM_ROTATE SCREWED UP WITH > 99
						"/ENABLED" << initialised << "/";
				}
				else
				{
					S << "HX" << 10000 * (poseFiltered[0].x + moffsets[0][1].v[0] + troffsets.v[0]) <<
						"/HY" << 10000 * (poseFiltered[0].y + moffsets[0][1].v[1] + troffsets.v[1]) <<
						"/HZ" << 10000 * (poseFiltered[0].z + moffsets[0][1].v[2] + troffsets.v[2]) <<
						"/MX" << 10000 * (poseFiltered[1].x + moffsets[0][0].v[0] + troffsets.v[0]) <<
						"/MY" << 10000 * (poseFiltered[1].y + moffsets[0][0].v[1] + troffsets.v[1]) <<
						"/MZ" << 10000 * (poseFiltered[1].z + moffsets[0][0].v[2] + troffsets.v[2]) <<
						"/PX" << 10000 * (poseFiltered[2].x + moffsets[0][2].v[0] + troffsets.v[0]) <<
						"/PY" << 10000 * (poseFiltered[2].y + moffsets[0][2].v[1] + troffsets.v[1]) <<
						"/PZ" << 10000 * (poseFiltered[2].z + moffsets[0][2].v[2] + troffsets.v[2]) <<
						"/HRW" << 10000 * (trackerRoth.w) <<
						"/HRX" << 10000 * (trackerRoth.x) <<
						"/HRY" << 10000 * (trackerRoth.y) <<
						"/HRZ" << 10000 * (trackerRoth.z) <<
						"/MRW" << 10000 * (trackerRotm.w) <<
						"/MRX" << 10000 * (trackerRotm.x) <<
						"/MRY" << 10000 * (trackerRotm.y) <<
						"/MRZ" << 10000 * (trackerRotm.z) <<
						"/PRW" << 10000 * (trackerRoty.w) <<
						"/PRX" << 10000 * (trackerRoty.x) <<
						"/PRY" << 10000 * (trackerRoty.y) <<
						"/PRZ" << 10000 * (trackerRoty.z) <<
						"/WRW" << 10000 * (0) << //DEPRECATED: GLM_ROTATE SCREWED UP WITH > 99
						"/ENABLED" << initialised << "/";
				}

				return S.str();
			}();
			
			char TrackerD[1024];
			strcpy_s(TrackerD, TrackerS.c_str());

			HANDLE pipeTracker = CreateFile(
				TEXT("\\\\.\\pipe\\LogPipeTracker"), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0,
				nullptr);
			
			DWORD numWritten;
			WriteFile(pipeTracker, TrackerD, sizeof(TrackerD), &numWritten, nullptr);
			CloseHandle(pipeTracker);


			auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
				std::chrono::high_resolution_clock::now() - t1).count();
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

			moffsets[0][0] = {pos[0][0], pos[0][1], pos[0][2]};
			moffsets[0][1] = {pos[1][0], pos[1][1], pos[1][2]};
			moffsets[0][2] = {pos[2][0], pos[2][1], pos[2][2]};

			moffsets[1][0] = {rot[0][0], rot[0][1], rot[0][2]};
			moffsets[1][1] = {rot[1][0], rot[1][1], rot[1][2]};
			moffsets[1][2] = {rot[2][0], rot[2][1], rot[2][2]};

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
				{moffsets[1][0].v[0], moffsets[1][0].v[1], moffsets[1][0].v[2]},
				{moffsets[1][1].v[0], moffsets[1][1].v[1], moffsets[1][1].v[2]},
				{moffsets[1][2].v[0], moffsets[1][2].v[1], moffsets[1][2].v[2]},
			};

			float kPosition[3][3] = {
				{moffsets[0][0].v[0], moffsets[0][0].v[1], moffsets[0][0].v[2]},
				{moffsets[0][1].v[0], moffsets[0][1].v[1], moffsets[0][1].v[2]},
				{moffsets[0][2].v[0], moffsets[0][2].v[1], moffsets[0][2].v[2]},
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
