#pragma once
#include "IKinectHandler.h"
#include <opencv2/opencv.hpp>
#include "KinectTrackedDevice.h"

class KinectHandlerBase : public IKinectHandler
{
public:
	KinectHandlerBase()
	{
	}

	~KinectHandlerBase()
	{
	}

	bool convertColorToDepthResolution = false;
	// Color Buffer
	std::vector<BYTE> colorBuffer;
	int colorWidth;
	int colorHeight;
	unsigned int colorBytesPerPixel;
	cv::Mat colorMat;

	// Depth Buffer
	std::vector<UINT16> depthBuffer;
	int depthWidth;
	int depthHeight;
	unsigned int depthBytesPerPixel;
	cv::Mat depthMat;

	void initOpenGL() override
	{
	};

	void initialise() override
	{
	};

	virtual void initialiseSkeleton()
	{
	};

	virtual void initialiseColor()
	{
	};

	virtual void initialiseDepth()
	{
	};

	virtual void terminateSkeleton()
	{
	};

	virtual void terminateColor()
	{
	};

	virtual void terminateDepth()
	{
	};

	HRESULT getStatusResult() override { return E_NOTIMPL; }
	std::string statusResultString(HRESULT stat) override { return "statusResultString behaviour not defined"; };

	virtual bool getFilteredJoint(KVR::KinectTrackedDevice device, vr::HmdVector3d_t& position,
	                              vr::HmdQuaternion_t& rotation) { return false; };

	void update() override
	{
	};

	bool putRGBDataIntoMatrix(cv::Mat& image) override { return false; }

	void drawKinectData(sf::RenderWindow& win) override
	{
	}; // Houses the below draw functions with a check
	void drawKinectImageData(sf::RenderWindow& win) override
	{
	};

	void drawTrackedSkeletons(sf::RenderWindow& win) override
	{
	};

	void zeroAllTracking(vr::IVRSystem* & m_sys) override
	{
	};

	void updateTrackersWithSkeletonPosition(
		std::vector<KVR::KinectTrackedDevice>& trackers
	) override
	{
	};

	virtual void updateTrackersWithColorPosition(
		std::vector<KVR::KinectTrackedDevice> trackers, sf::Vector2i pos)
	{
	}
};
