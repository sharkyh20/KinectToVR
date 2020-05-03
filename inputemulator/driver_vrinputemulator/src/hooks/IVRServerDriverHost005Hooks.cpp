#include "IVRServerDriverHost005Hooks.h"

#include "../driver/ServerDriver.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

using namespace glm;

namespace vrinputemulator {
namespace driver {


HookData<IVRServerDriverHost005Hooks::trackedDeviceAdded_t> IVRServerDriverHost005Hooks::trackedDeviceAddedHook;
HookData<IVRServerDriverHost005Hooks::trackedDevicePoseUpdated_t> IVRServerDriverHost005Hooks::trackedDevicePoseUpdatedHook;
HookData<IVRServerDriverHost005Hooks::pollNextEvent_t> IVRServerDriverHost005Hooks::pollNextEventHook;


IVRServerDriverHost005Hooks::IVRServerDriverHost005Hooks(void* iptr) {
	if (!_isHooked) {
		CREATE_MH_HOOK(trackedDeviceAddedHook, _trackedDeviceAdded, "IVRServerDriverHost005::TrackedDeviceAdded", iptr, 0);
		CREATE_MH_HOOK(trackedDevicePoseUpdatedHook, _trackedDevicePoseUpdated, "IVRServerDriverHost005::TrackedDevicePoseUpdated", iptr, 1);
		CREATE_MH_HOOK(pollNextEventHook, _pollNextEvent, "IVRServerDriverHost005::PollNextEvent", iptr, 5);
		_isHooked = true;
	}
}


IVRServerDriverHost005Hooks::~IVRServerDriverHost005Hooks() {
	if (_isHooked) {
		REMOVE_MH_HOOK(trackedDeviceAddedHook);
		REMOVE_MH_HOOK(trackedDevicePoseUpdatedHook);
		REMOVE_MH_HOOK(pollNextEventHook);
		_isHooked = false;
	}
}


std::shared_ptr<InterfaceHooks> IVRServerDriverHost005Hooks::createHooks(void * iptr) {
	std::shared_ptr<InterfaceHooks> retval = std::shared_ptr<InterfaceHooks>(new IVRServerDriverHost005Hooks(iptr));
	return retval;
}

void IVRServerDriverHost005Hooks::trackedDevicePoseUpdatedOrig(void * _this, uint32_t unWhichDevice, const vr::DriverPose_t & newPose, uint32_t unPoseStructSize) {
	if (ServerDriver::started == true) {
		if (unWhichDevice == 0 || unWhichDevice == 4294967295) {
			auto poseCopy = newPose;
			poseCopy.vecPosition[0] /= 2056;
			poseCopy.vecPosition[1] /= 2056;
			poseCopy.vecPosition[2] /= 2056;

			quat nr, xr;
			vec3 nre, xre;
			nr.x = poseCopy.qRotation.x;
			nr.y = poseCopy.qRotation.y;
			nr.z = poseCopy.qRotation.z;
			nr.w = poseCopy.qRotation.w;
			nre = eulerAngles(nr);

			xre.x = ServerDriver::nrotT.x + nre.x;
			xre.y = ServerDriver::nrotT.y + nre.y;
			xre.z = ServerDriver::nrotT.z + nre.z;

			xr = quat(xre);

			poseCopy.qRotation.w = xr.w;
			poseCopy.qRotation.x = xr.x;
			poseCopy.qRotation.y = xr.y;
			poseCopy.qRotation.z = xr.z;


			trackedDevicePoseUpdatedHook.origFunc(_this, unWhichDevice, poseCopy, unPoseStructSize);
		}
		else {
			auto poseCopy = newPose;

			poseCopy.vecPosition[0] -= ServerDriver::nposT.v[0];
			poseCopy.vecPosition[1] -= ServerDriver::nposT.v[1];
			poseCopy.vecPosition[2] -= ServerDriver::nposT.v[2];

			trackedDevicePoseUpdatedHook.origFunc(_this, unWhichDevice, poseCopy, unPoseStructSize);
		}
	}
	else {
		auto poseCopy = newPose;

		poseCopy.vecPosition[0] -= ServerDriver::nposT.v[0];
		poseCopy.vecPosition[1] -= ServerDriver::nposT.v[1];
		poseCopy.vecPosition[2] -= ServerDriver::nposT.v[2];
		
		trackedDevicePoseUpdatedHook.origFunc(_this, unWhichDevice, poseCopy, unPoseStructSize);
	}
}


bool IVRServerDriverHost005Hooks::_trackedDeviceAdded(void* _this, const char *pchDeviceSerialNumber, vr::ETrackedDeviceClass eDeviceClass, void *pDriver) {
	LOG(TRACE) << "IVRServerDriverHost005Hooks::_trackedDeviceAdded(" << _this << ", " << pchDeviceSerialNumber << ", " << eDeviceClass << ", " << pDriver << ")";
	serverDriver->hooksTrackedDeviceAdded(_this, 5, pchDeviceSerialNumber, eDeviceClass, pDriver);
	auto retval = trackedDeviceAddedHook.origFunc(_this, pchDeviceSerialNumber, eDeviceClass, pDriver);
	return retval;
}

void IVRServerDriverHost005Hooks::_trackedDevicePoseUpdated(void* _this, uint32_t unWhichDevice, const vr::DriverPose_t& newPose, uint32_t unPoseStructSize) {
	// Call rates:
	//
	// Vive HMD: 1120 calls/s
	// Vive Controller: 369 calls/s each
	//
	// Time is key. If we assume 1 HMD and 13 controllers, we have a total of  ~6000 calls/s. That's about 166 microseconds per call at 100% load.
	auto poseCopy = newPose;
	if (ServerDriver::started == true) {
		if (unWhichDevice == 0 || unWhichDevice == 4294967295) {
			poseCopy.vecPosition[0] /= 2056;
			poseCopy.vecPosition[1] /= 2056;
			poseCopy.vecPosition[2] /= 2056;

			quat nr, xr;
			vec3 nre, xre;
			nr.x = poseCopy.qRotation.x;
			nr.y = poseCopy.qRotation.y;
			nr.z = poseCopy.qRotation.z;
			nr.w = poseCopy.qRotation.w;
			nre = eulerAngles(nr);

			xre.x = ServerDriver::nrotT.x + nre.x;
			xre.y = ServerDriver::nrotT.y + nre.y;
			xre.z = ServerDriver::nrotT.z + nre.z;

			xr = quat(xre);

			poseCopy.qRotation.w = xr.w;
			poseCopy.qRotation.x = xr.x;
			poseCopy.qRotation.y = xr.y;
			poseCopy.qRotation.z = xr.z;

			if (serverDriver->hooksTrackedDevicePoseUpdated(_this, 5, unWhichDevice, poseCopy, unPoseStructSize)) {
				trackedDevicePoseUpdatedHook.origFunc(_this, unWhichDevice, poseCopy, unPoseStructSize);
			}
		}
		else {
			poseCopy.vecPosition[0] -= ServerDriver::nposT.v[0];
			poseCopy.vecPosition[1] -= ServerDriver::nposT.v[1];
			poseCopy.vecPosition[2] -= ServerDriver::nposT.v[2];

			if (serverDriver->hooksTrackedDevicePoseUpdated(_this, 5, unWhichDevice, poseCopy, unPoseStructSize)) {
				trackedDevicePoseUpdatedHook.origFunc(_this, unWhichDevice, poseCopy, unPoseStructSize);
			}
		}
	}
	else {
		poseCopy.vecPosition[0] -= ServerDriver::nposT.v[0];
		poseCopy.vecPosition[1] -= ServerDriver::nposT.v[1];
		poseCopy.vecPosition[2] -= ServerDriver::nposT.v[2];

		if (serverDriver->hooksTrackedDevicePoseUpdated(_this, 5, unWhichDevice, poseCopy, unPoseStructSize)) {
			trackedDevicePoseUpdatedHook.origFunc(_this, unWhichDevice, poseCopy, unPoseStructSize);
		}
	}
}

bool IVRServerDriverHost005Hooks::_pollNextEvent(void* _this, void* pEvent, uint32_t uncbVREvent) {
	auto injectedEvent = serverDriver->getDriverEventForInjection(_this);
	if (injectedEvent.first) {
		if (injectedEvent.second == uncbVREvent) {
			memcpy(pEvent, injectedEvent.first.get(), uncbVREvent);
			auto event = (vr::VREvent_t*)pEvent;
			LOG(DEBUG) << "IVRServerDriverHost005Hooks::_pollNextEvent: Injecting event: " << event->eventType << ", " << event->trackedDeviceIndex;
			return true;
		} else {
			auto event = (vr::VREvent_t*)injectedEvent.first.get();
			LOG(ERROR) << "IVRServerDriverHost005Hooks::_pollNextEvent: Could not inject event (" << event->eventType << ", " << event->trackedDeviceIndex 
				<< ") because size does not match, expected " << uncbVREvent << " but got " << injectedEvent.second;
		}
	}
	bool retval, hretval;
	do {
		retval = pollNextEventHook.origFunc(_this, pEvent, uncbVREvent);
		if (retval) {
			hretval = serverDriver->hooksPollNextEvent(_this, 5, pEvent, uncbVREvent);
		}
	} while (retval && !hretval);
	return retval;
}


}
}
