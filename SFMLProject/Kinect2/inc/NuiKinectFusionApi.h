///------------------------------------------------------------------------------
/// <copyright file="NuiKinectFusionApi.h" company="Microsoft">
///    Copyright (c) Microsoft Corporation. All rights reserved.
/// </copyright>
///------------------------------------------------------------------------------
/// This module defines the APIs for the Natural User Interface(NUI) Kinect Fusion Implementation

#pragma once

#ifndef FUSIONAPI
    #define FUSIONAPI __declspec( dllimport ) WINAPI
#endif

#ifdef __cplusplus
extern "C" {
#endif

//
// Define NUI Kinect Fusion specific error codes
//

#ifndef _NUI_FUSION_HRESULTS
#define _NUI_FUSION_HRESULTS

#define NUI_FUSION_API_HRESULT_START_INDEX          3050
#define E_NUI_FUSION_TRACKING_ERROR                 MAKE_HRESULT(SEVERITY_ERROR, FACILITY_NUI, NUI_FUSION_API_HRESULT_START_INDEX)
#define E_NUI_GPU_FAIL                              MAKE_HRESULT(SEVERITY_ERROR, FACILITY_NUI, NUI_FUSION_API_HRESULT_START_INDEX+1)
#define E_NUI_GPU_OUTOFMEMORY                       MAKE_HRESULT(SEVERITY_ERROR, FACILITY_NUI, NUI_FUSION_API_HRESULT_START_INDEX+2)
#define E_NUI_NO_MATCH                              MAKE_HRESULT(SEVERITY_ERROR, FACILITY_NUI, NUI_FUSION_API_HRESULT_START_INDEX+3)

#endif

#ifdef __cplusplus
} // end extern "C"
#endif

#include <NuiKinectFusionVolume.h>
#include <NuiKinectFusionColorVolume.h>
#include <NuiKinectFusionDepthProcessor.h>
#include <NuiKinectFusionCameraPoseFinder.h>

