#include "stdafx.h"

#include "TrackingPoolManager.h"

std::vector<KVR::TrackedDeviceInputData> TrackingPoolManager::devicePool;

uint32_t TrackingPoolManager::leftFootDevicePosGID = k_invalidTrackerID;
uint32_t TrackingPoolManager::rightFootDevicePosGID = k_invalidTrackerID;
uint32_t TrackingPoolManager::leftFootDeviceRotGID = k_invalidTrackerID;
uint32_t TrackingPoolManager::rightFootDeviceRotGID = k_invalidTrackerID;

uint32_t TrackingPoolManager::kinectFirstId = k_invalidTrackerID;
uint32_t TrackingPoolManager::kinectLastId = k_invalidTrackerID;
uint32_t TrackingPoolManager::kinectJointToGlobalIDTable[KVR::KinectJointCount];

uint32_t TrackingPoolManager::kinectSensorGID = k_invalidTrackerID;
