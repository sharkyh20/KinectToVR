#pragma once
#include "IKinectHandler.h"
#include "KinectHandlerBase.h"
#include <Kinect.h>
#include <Windows.h>

class KinectV2Handler : public KinectHandlerBase
{
public:
	KinectV2Handler()
	{
		KinectV2Handler::initialise();
		KinectV2Handler::initOpenGL();
	}

	virtual ~KinectV2Handler()
	{
	}

	IKinectSensor* kinectSensor = nullptr;
	IBodyFrameReader* bodyFrameReader = nullptr;
	IColorFrameReader* colorFrameReader = nullptr;
	IDepthFrameReader* depthFrameReader = nullptr;
	IMultiSourceFrame* multiFrame = nullptr;
	ICoordinateMapper* coordMapper = nullptr;
	BOOLEAN isTracking = false;

	Joint joints[JointType_Count];
	JointOrientation jointOrientations[JointType_Count];
	IBody* kinectBodies[BODY_COUNT];

	HRESULT getStatusResult() override;
	std::string statusResultString(HRESULT stat) override;

	void initialise() override;

	void initialiseSkeleton() override;
	void terminateSkeleton() override;
	void initOpenGL() override;
	void update() override;
	void shutdown() override;


	bool convertColorToDepthResolution = false;
	void onBodyFrameArrived(IBodyFrameReader& sender, IBodyFrameArrivedEventArgs& eventArgs);
	virtual void updateSkeletalData();

private:
	bool initKinect();
	void updateSkeletalFilters();

	WAITABLE_HANDLE h_bodyFrameEvent;
	bool newBodyFrameArrived = false;
};
