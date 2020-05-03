#pragma once

#include <memory>
#include <array>
#include <iostream>
#include <WinSock2.h>

#include <openvr_driver.h>

#include "FakeTracker.hpp"

class ServerDriver : public vr::IServerTrackedDeviceProvider {
public:
	static ServerDriver* get();
	
	ServerDriver(const ServerDriver&) = delete;
	ServerDriver& operator=(const ServerDriver&) = delete;


	virtual vr::EVRInitError Init(vr::IVRDriverContext * pDriverContext) override;
	virtual void Cleanup() override;
	virtual const char * const * GetInterfaceVersions() override;
	virtual void RunFrame() override;
	virtual bool ShouldBlockStandbyMode() override;
	virtual void EnterStandby() override;
	virtual void LeaveStandby() override;


	// Reason for using shared_ptr is that the pointer will always be the same once initialized, 
	// whereas if we use just a plain vector, the pointer to the element could change if the 
	// vector is reallocated...

	std::vector<std::shared_ptr<FakeTracker>> _trackers;

private:
	ServerDriver();

	static ServerDriver* _instance;
	
};