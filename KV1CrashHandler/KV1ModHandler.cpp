#include "KV1ModHandler.h"

#include <string>

HRESULT KinectV1Handler::getStatusResult()
{
	if (kinectSensor)
		return kinectSensor->NuiStatus();
	return E_NUI_NOTCONNECTED;
}

void KinectV1Handler::initOpenGL()
{
}

std::string KinectV1Handler::statusResultString(HRESULT stat)
{
	switch (stat)
	{
	case S_OK: return "S_OK";
	case S_NUI_INITIALIZING: return "S_NUI_INITIALIZING The device is connected, but still initializing.";
	case E_NUI_NOTCONNECTED: return "E_NUI_NOTCONNECTED The device is not connected.";
	case E_NUI_NOTGENUINE: return "E_NUI_NOTGENUINE The device is not a valid Kinect.";
	case E_NUI_NOTSUPPORTED: return "E_NUI_NOTSUPPORTED The device is an unsupported model.";
	case E_NUI_INSUFFICIENTBANDWIDTH: return
			"E_NUI_INSUFFICIENTBANDWIDTH The device is connected to a hub without the necessary bandwidth requirements.";
	case E_NUI_NOTPOWERED: return "E_NUI_NOTPOWERED The device is connected, but unpowered.";
	case E_NUI_NOTREADY: return "E_NUI_NOTREADY There was some other unspecified error.";
	default: return "Uh Oh undefined kinect error! " + std::to_string(stat);
	}
}

void KinectV1Handler::initialise()
{
	try
	{
		initialised = initKinect();
		if (!initialised) throw FailedKinectInitialisation;
	}
	catch (std::exception& e)
	{
	}
}

void KinectV1Handler::shutdown()
{
	try
	{
		kinectSensor->NuiShutdown();
	}
	catch (std::exception& e)
	{
	}
}

void KinectV1Handler::update()
{
	if (isInitialised())
	{
		HRESULT kinectStatus = kinectSensor->NuiStatus();
		if (kinectStatus == S_OK)
		{
			updateSkeletalData();
		}
	}
}

bool KinectV1Handler::initKinect()
{
	//Get a working Kinect Sensor
	int numSensors = 0;
	if (NuiGetSensorCount(&numSensors) < 0 || numSensors < 1)
	{
		return false;
	}
	if (NuiCreateSensorByIndex(0, &kinectSensor) < 0)
	{
		return false;
	}
	//Initialise Sensor
	HRESULT hr = kinectSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_SKELETON);
	kinectSensor->NuiSkeletonTrackingEnable(nullptr, 0); //NUI_SKELETON_TRACKING_FLAG_ENABLE_IN_NEAR_RANGE

	return kinectSensor;
}

bool KinectV1Handler::acquireKinectFrame(NUI_IMAGE_FRAME& imageFrame, HANDLE& rgbStream, INuiSensor*& sensor)
{
	return (sensor->NuiImageStreamGetNextFrame(rgbStream, 1, &imageFrame) < 0);
}

void KinectV1Handler::releaseKinectFrame(NUI_IMAGE_FRAME& imageFrame, HANDLE& rgbStream, INuiSensor*& sensor)
{
	sensor->NuiImageStreamReleaseFrame(rgbStream, &imageFrame);
}

void KinectV1Handler::updateSkeletalData()
{
	if (kinectSensor->NuiSkeletonGetNextFrame(0, &skeletonFrame) >= 0)
	{
		NUI_TRANSFORM_SMOOTH_PARAMETERS params;
		/*
		params.fCorrection = .25f;
		params.fJitterRadius = .4f;
		params.fMaxDeviationRadius = .25f;
		params.fPrediction = .25f;
		params.fSmoothing = .25f;
		//*/
		///*
		params.fSmoothing = .15f;
		params.fCorrection = .25f;
		params.fMaxDeviationRadius = .17f;
		params.fJitterRadius = .11f;
		params.fPrediction = .17f;
		//*/

		kinectSensor->NuiTransformSmooth(&skeletonFrame, &params); //Smooths jittery tracking

		for (int i = 0; i < NUI_SKELETON_COUNT; ++i)
		{
			NUI_SKELETON_TRACKING_STATE trackingState = skeletonFrame.SkeletonData[i].eTrackingState;

			if (NUI_SKELETON_TRACKED == trackingState)
			{
				for (int j = 0; j < NUI_SKELETON_POSITION_COUNT; ++j)
				{
					jointPositions[j] = skeletonFrame.SkeletonData[i].SkeletonPositions[j];
					jointStates[j] = skeletonFrame.SkeletonData[i].eSkeletonPositionTrackingState[j];
				}
				NuiSkeletonCalculateBoneOrientations(&skeletonFrame.SkeletonData[i], boneOrientations);
				break;
			}
		}
	}
};

Vector4 KinectV1Handler::zeroKinectPosition(int trackedSkeletonIndex)
{
	return jointPositions[NUI_SKELETON_POSITION_HEAD];
}
