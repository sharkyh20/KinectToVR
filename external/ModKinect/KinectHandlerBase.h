#pragma once
#include "IKinectHandler.h"
#include <iostream>

typedef int JointTrackingState;
class KinectHandlerBase : public IKinectHandler {
public:
	KinectHandlerBase() {

	}
	~KinectHandlerBase() {

	}

	virtual void initialise() {};
	virtual void shutdown() {};
	virtual void initOpenGL() {};

	virtual void initialiseSkeleton() {};
	virtual void initialiseColor() {};
	virtual void initialiseDepth() {};

	virtual void terminateSkeleton() {};
	virtual void terminateColor() {};
	virtual void terminateDepth() {};

	virtual HRESULT getStatusResult() { return E_NOTIMPL; }
	virtual std::string statusResultString(HRESULT stat) { return "statusResultString behaviour not defined"; };

    virtual void update() {};
	
	JointTrackingState trackingStates[25] = { 0 }; //All not tracked
        bool isSkeletonTracked = false;

};
