#include "stdafx.h"

#include "TrackingPoolManager.h"

std::vector<KVR::TrackedDeviceInputData> TrackingPoolManager::devicePool;

uint32_t TrackingPoolManager::leftFootDevicePosGID = k_invalidTrackerID;
uint32_t TrackingPoolManager::rightFootDevicePosGID = k_invalidTrackerID;
uint32_t TrackingPoolManager::leftFootDeviceRotGID = k_invalidTrackerID;
uint32_t TrackingPoolManager::rightFootDeviceRotGID = k_invalidTrackerID;