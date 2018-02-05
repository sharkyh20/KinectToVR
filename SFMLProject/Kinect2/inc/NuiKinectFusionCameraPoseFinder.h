

/* this ALWAYS GENERATED file contains the definitions for the interfaces */


 /* File created by MIDL compiler version 8.00.0595 */
/* at Sun Oct 19 12:58:15 2014
 */
/* Compiler settings for ..\inc_public\KinectFusionCameraPoseFinder.idl:
    Oicf, W1, Zp8, env=Win64 (32b run), target_arch=AMD64 8.00.0595 
    protocol : dce , ms_ext, c_ext, robust
    error checks: allocation ref bounds_check enum stub_data 
    VC __declspec() decoration level: 
         __declspec(uuid()), __declspec(selectany), __declspec(novtable)
         DECLSPEC_UUID(), MIDL_INTERFACE()
*/
/* @@MIDL_FILE_HEADING(  ) */

#pragma warning( disable: 4049 )  /* more than 64k source lines */


/* verify that the <rpcndr.h> version is high enough to compile this file*/
#ifndef __REQUIRED_RPCNDR_H_VERSION__
#define __REQUIRED_RPCNDR_H_VERSION__ 475
#endif

/* verify that the <rpcsal.h> version is high enough to compile this file*/
#ifndef __REQUIRED_RPCSAL_H_VERSION__
#define __REQUIRED_RPCSAL_H_VERSION__ 100
#endif

#include "rpc.h"
#include "rpcndr.h"

#ifndef __RPCNDR_H_VERSION__
#error this stub requires an updated version of <rpcndr.h>
#endif // __RPCNDR_H_VERSION__


#ifndef __NuiKinectFusionCameraPoseFinder_h__
#define __NuiKinectFusionCameraPoseFinder_h__

#if defined(_MSC_VER) && (_MSC_VER >= 1020)
#pragma once
#endif

/* Forward Declarations */ 

#ifndef __INuiFusionMatchCandidates_FWD_DEFINED__
#define __INuiFusionMatchCandidates_FWD_DEFINED__
typedef interface INuiFusionMatchCandidates INuiFusionMatchCandidates;

#endif 	/* __INuiFusionMatchCandidates_FWD_DEFINED__ */


#ifndef __INuiFusionCameraPoseFinder_FWD_DEFINED__
#define __INuiFusionCameraPoseFinder_FWD_DEFINED__
typedef interface INuiFusionCameraPoseFinder INuiFusionCameraPoseFinder;

#endif 	/* __INuiFusionCameraPoseFinder_FWD_DEFINED__ */


/* header files for imported files */
#include "oaidl.h"
#include "ocidl.h"

#ifdef __cplusplus
extern "C"{
#endif 


/* interface __MIDL_itf_KinectFusionCameraPoseFinder_0000_0000 */
/* [local] */ 

///----------------------------------------------------------------------------
/// <copyright file="NuiKinectFusionCameraPoseFinder.h" company="Microsoft">
/// Copyright (c) Microsoft Corporation. All rights reserved.>
/// </copyright>
///----------------------------------------------------------------------------

#include "NuiKinectFusionApi.h"

#ifndef DEFINE_UUID
#define DEFINE_UUID(name, l, w1, w2, b1, b2, b3, b4, b5, b6, b7, b8) extern "C"  const GUID __declspec(selectany) name = { l, w1, w2, { b1, b2,  b3,  b4,  b5,  b6,  b7,  b8 } }
#endif

// 4C98136C-F201-4C00-9911-4FCB9918FFD5
DEFINE_UUID(IID_INuiFusionCameraPoseFinder,0x4C98136C, 0xF201, 0x4C00, 0x99, 0x11, 0x4F, 0xCB, 0x99, 0x18, 0xFF, 0xD5);

#include <Kinect.h>

#include <pshpack8.h>

#define NUI_FUSION_CAMERA_POSE_FINDER_DEFAULT_FEATURE_LOCATIONS_PER_FRAME_COUNT (500)
#define NUI_FUSION_CAMERA_POSE_FINDER_DEFAULT_POSE_HISTORY_COUNT (10000)
#define NUI_FUSION_CAMERA_POSE_FINDER_DEFAULT_MAX_DEPTH_THRESHOLD (4.0f)

#ifndef _NUI_FUSION_CAMERA_POSE_FINDER_PARAMETERS_
#define _NUI_FUSION_CAMERA_POSE_FINDER_PARAMETERS_
typedef struct _NUI_FUSION_CAMERA_POSE_FINDER_PARAMETERS
    {
    UINT featureSampleLocationsPerFrameCount;
    UINT maxPoseHistoryCount;
    FLOAT maxDepthThreshold;
    } 	NUI_FUSION_CAMERA_POSE_FINDER_PARAMETERS;

#endif _NUI_FUSION_CAMERA_POSE_FINDER_PARAMETERS_


extern RPC_IF_HANDLE __MIDL_itf_KinectFusionCameraPoseFinder_0000_0000_v0_0_c_ifspec;
extern RPC_IF_HANDLE __MIDL_itf_KinectFusionCameraPoseFinder_0000_0000_v0_0_s_ifspec;






/* [helpstring][version][uuid] */ 




#ifndef __INuiFusionMatchCandidates_INTERFACE_DEFINED__
#define __INuiFusionMatchCandidates_INTERFACE_DEFINED__

/* interface INuiFusionMatchCandidates */
/* [object][local][uuid] */ 


EXTERN_C const IID IID_INuiFusionMatchCandidates;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("42C1992C-043A-492D-A486-E9275197E7E6")
    INuiFusionMatchCandidates : public IUnknown
    {
    public:
        virtual UINT STDMETHODCALLTYPE MatchPoseCount( void) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE GetMatchPoses( 
            /* [out] */ const Matrix4 **pPoses) = 0;
        
        virtual UINT STDMETHODCALLTYPE MatchSimilarityCount( void) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE GetMatchSimilarities( 
            /* [out] */ const FLOAT **pSimilarityMeasurements) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE CalculateMinimumDistance( 
            /* [out] */ FLOAT *pMinimumDistance) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct INuiFusionMatchCandidatesVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            INuiFusionMatchCandidates * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            INuiFusionMatchCandidates * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            INuiFusionMatchCandidates * This);
        
        UINT ( STDMETHODCALLTYPE *MatchPoseCount )( 
            INuiFusionMatchCandidates * This);
        
        HRESULT ( STDMETHODCALLTYPE *GetMatchPoses )( 
            INuiFusionMatchCandidates * This,
            /* [out] */ const Matrix4 **pPoses);
        
        UINT ( STDMETHODCALLTYPE *MatchSimilarityCount )( 
            INuiFusionMatchCandidates * This);
        
        HRESULT ( STDMETHODCALLTYPE *GetMatchSimilarities )( 
            INuiFusionMatchCandidates * This,
            /* [out] */ const FLOAT **pSimilarityMeasurements);
        
        HRESULT ( STDMETHODCALLTYPE *CalculateMinimumDistance )( 
            INuiFusionMatchCandidates * This,
            /* [out] */ FLOAT *pMinimumDistance);
        
        END_INTERFACE
    } INuiFusionMatchCandidatesVtbl;

    interface INuiFusionMatchCandidates
    {
        CONST_VTBL struct INuiFusionMatchCandidatesVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define INuiFusionMatchCandidates_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define INuiFusionMatchCandidates_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define INuiFusionMatchCandidates_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define INuiFusionMatchCandidates_MatchPoseCount(This)	\
    ( (This)->lpVtbl -> MatchPoseCount(This) ) 

#define INuiFusionMatchCandidates_GetMatchPoses(This,pPoses)	\
    ( (This)->lpVtbl -> GetMatchPoses(This,pPoses) ) 

#define INuiFusionMatchCandidates_MatchSimilarityCount(This)	\
    ( (This)->lpVtbl -> MatchSimilarityCount(This) ) 

#define INuiFusionMatchCandidates_GetMatchSimilarities(This,pSimilarityMeasurements)	\
    ( (This)->lpVtbl -> GetMatchSimilarities(This,pSimilarityMeasurements) ) 

#define INuiFusionMatchCandidates_CalculateMinimumDistance(This,pMinimumDistance)	\
    ( (This)->lpVtbl -> CalculateMinimumDistance(This,pMinimumDistance) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __INuiFusionMatchCandidates_INTERFACE_DEFINED__ */


#ifndef __INuiFusionCameraPoseFinder_INTERFACE_DEFINED__
#define __INuiFusionCameraPoseFinder_INTERFACE_DEFINED__

/* interface INuiFusionCameraPoseFinder */
/* [object][local][uuid] */ 


EXTERN_C const IID IID_INuiFusionCameraPoseFinder;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("4C98136C-F201-4C00-9911-4FCB9918FFD5")
    INuiFusionCameraPoseFinder : public IUnknown
    {
    public:
        virtual HRESULT STDMETHODCALLTYPE ResetCameraPoseFinder( void) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE ProcessFrame( 
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pDepthFloatFrame,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pColorFrame,
            /* [in] */ const Matrix4 *pWorldToCameraTransform,
            /* [in] */ FLOAT minimumDistanceThreshold,
            /* [out] */ BOOL *pAddedPose,
            /* [out] */ BOOL *pHistoryTrimmed) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE FindCameraPose( 
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pDepthFloatFrame,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pColorFrame,
            /* [out] */ INuiFusionMatchCandidates **ppMatchCandidates) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE LoadCameraPoseFinderDatabase( 
            /* [in] */ LPCWSTR filename) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE SaveCameraPoseFinderDatabase( 
            /* [in] */ LPCWSTR filename) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE GetCameraPoseFinderParameters( 
            /* [out] */ NUI_FUSION_CAMERA_POSE_FINDER_PARAMETERS *pCameraPoseFinderParameters) = 0;
        
        virtual UINT STDMETHODCALLTYPE GetStoredPoseCount( void) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct INuiFusionCameraPoseFinderVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            INuiFusionCameraPoseFinder * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            INuiFusionCameraPoseFinder * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            INuiFusionCameraPoseFinder * This);
        
        HRESULT ( STDMETHODCALLTYPE *ResetCameraPoseFinder )( 
            INuiFusionCameraPoseFinder * This);
        
        HRESULT ( STDMETHODCALLTYPE *ProcessFrame )( 
            INuiFusionCameraPoseFinder * This,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pDepthFloatFrame,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pColorFrame,
            /* [in] */ const Matrix4 *pWorldToCameraTransform,
            /* [in] */ FLOAT minimumDistanceThreshold,
            /* [out] */ BOOL *pAddedPose,
            /* [out] */ BOOL *pHistoryTrimmed);
        
        HRESULT ( STDMETHODCALLTYPE *FindCameraPose )( 
            INuiFusionCameraPoseFinder * This,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pDepthFloatFrame,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pColorFrame,
            /* [out] */ INuiFusionMatchCandidates **ppMatchCandidates);
        
        HRESULT ( STDMETHODCALLTYPE *LoadCameraPoseFinderDatabase )( 
            INuiFusionCameraPoseFinder * This,
            /* [in] */ LPCWSTR filename);
        
        HRESULT ( STDMETHODCALLTYPE *SaveCameraPoseFinderDatabase )( 
            INuiFusionCameraPoseFinder * This,
            /* [in] */ LPCWSTR filename);
        
        HRESULT ( STDMETHODCALLTYPE *GetCameraPoseFinderParameters )( 
            INuiFusionCameraPoseFinder * This,
            /* [out] */ NUI_FUSION_CAMERA_POSE_FINDER_PARAMETERS *pCameraPoseFinderParameters);
        
        UINT ( STDMETHODCALLTYPE *GetStoredPoseCount )( 
            INuiFusionCameraPoseFinder * This);
        
        END_INTERFACE
    } INuiFusionCameraPoseFinderVtbl;

    interface INuiFusionCameraPoseFinder
    {
        CONST_VTBL struct INuiFusionCameraPoseFinderVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define INuiFusionCameraPoseFinder_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define INuiFusionCameraPoseFinder_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define INuiFusionCameraPoseFinder_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define INuiFusionCameraPoseFinder_ResetCameraPoseFinder(This)	\
    ( (This)->lpVtbl -> ResetCameraPoseFinder(This) ) 

#define INuiFusionCameraPoseFinder_ProcessFrame(This,pDepthFloatFrame,pColorFrame,pWorldToCameraTransform,minimumDistanceThreshold,pAddedPose,pHistoryTrimmed)	\
    ( (This)->lpVtbl -> ProcessFrame(This,pDepthFloatFrame,pColorFrame,pWorldToCameraTransform,minimumDistanceThreshold,pAddedPose,pHistoryTrimmed) ) 

#define INuiFusionCameraPoseFinder_FindCameraPose(This,pDepthFloatFrame,pColorFrame,ppMatchCandidates)	\
    ( (This)->lpVtbl -> FindCameraPose(This,pDepthFloatFrame,pColorFrame,ppMatchCandidates) ) 

#define INuiFusionCameraPoseFinder_LoadCameraPoseFinderDatabase(This,filename)	\
    ( (This)->lpVtbl -> LoadCameraPoseFinderDatabase(This,filename) ) 

#define INuiFusionCameraPoseFinder_SaveCameraPoseFinderDatabase(This,filename)	\
    ( (This)->lpVtbl -> SaveCameraPoseFinderDatabase(This,filename) ) 

#define INuiFusionCameraPoseFinder_GetCameraPoseFinderParameters(This,pCameraPoseFinderParameters)	\
    ( (This)->lpVtbl -> GetCameraPoseFinderParameters(This,pCameraPoseFinderParameters) ) 

#define INuiFusionCameraPoseFinder_GetStoredPoseCount(This)	\
    ( (This)->lpVtbl -> GetStoredPoseCount(This) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __INuiFusionCameraPoseFinder_INTERFACE_DEFINED__ */



/* interface __MIDL_itf_KinectFusionCameraPoseFinder_0000_0001 */
/* [local] */ 


#include <poppack.h>

// <summary>
/// Initialize a Kinect Fusion Camera Pose Finder to be used with camera tracking.
/// Note: Users must call Release on the interface when finished with the camera pose finder.
/// A Kinect camera is also required to be connected.
/// </summary>
/// <param name="pCameraPoseFinderParameters">
/// A pointer to an instance of NUI_FUSION_CAMERA_POSE_FINDER_PARAMETERS, defining the camera pose finder
/// settings. The <paramref name="pCameraPoseFinderParameters"/> parameter's members must both be
/// greater than 0, and <c>maxPoseHistoryCount</c> should be a maximum of 10,000,000 and the 
/// <c>featureSampleLocationsPerFrameCount</c> member should be a maximum of 1000. The 
/// <c>maxDepthThreshold</c> should be greater than 0.4f and a maximum of the closest working distance
/// you expect in your scenario (i.e. if all your reconstruction is at short range 0-2m, set 2.0f here).
/// Note that with the <c>maxDepthThreshold</c> there is a trade-off, as setting large distances may
/// make the system less discriminative, hence more features may be required to maintain matching
/// performance.
/// </param>
/// <param name ="pRandomFeatureLocationAndThresholdSeed">
/// Optionally, a pointer to an unsigned int value which seeds the random number generator used for
/// calculating the feature locations and the feature thresholds at each sample location. Pass NULL to
/// ignore and use the default initializer (system time).
/// </param>
/// <param name ="ppNuiFusionCameraPoseFinder">
/// A pointer that receives a reference to the created INuiFusionCameraPoseFinder interface.
/// </param>
/// <returns>
/// <para>Returns S_OK if successful; otherwise, returns one of the following failure codes:
/// <list type="table">
///    <listheader>
///       <term>Error code</term>
///       <description>Description</description>
///    </listheader>
///    <item>
///       <term>E_INVALIDARG</term>
///       <description>The <paramref name="pCameraPoseFinderParameters"/> parameter is NULL, or the
///       <paramref name="pCameraPoseFinderParameters"/> parameter's <c>featureSampleLocationsPerFrameCount</c>,
///       <c>maxPoseHistoryCount</c> member is 0, or the <c>maxPoseHistoryCount</c> is greater 
///       than 10,000,000, the <c>featureSampleLocationsPerFrameCount</c> member is greater than 1000,
///       or <c>maxDepthThreshold</c> is not greater than 0.4f.</description>
///    </item>
///    <item>
///       <term>E_POINTER</term>
///       <description>The <paramref name="ppNuiFusionCameraPoseFinder"/> parameter is NULL.</description>
///    </item>
///    <item>
///       <term>E_OUTOFMEMORY</term>
///       <description>A memory allocation failed, try reducing paramref name="pCameraPoseFinderParameters"/> 
///       parameter's <c>maxPoseHistoryCount</c> or <c>treeCount</c> member.</description>
///    </item>
///    <item>
///       <term>E_NUI_FEATURE_NOT_INITIALIZED</term>
///       <description>The Kinect camera runtime could not be accessed.</description>
///    </item>
///    <item>
///       <term>E_NUI_DEVICE_NOT_CONNECTED</term>
///       <description>No Kinect camera could be found.</description>
///    </item>
///    <item>
///       <term>E_FAIL</term>
///       <description>The call failed for an unknown reason.</description>
///    </item>
/// </list>
/// </returns>
_Check_return_ HRESULT FUSIONAPI NuiFusionCreateCameraPoseFinder (
    _In_ const NUI_FUSION_CAMERA_POSE_FINDER_PARAMETERS *pCameraPoseFinderParameters,
    _In_opt_ UINT *pRandomFeatureLocationAndThresholdSeed,
    _Deref_out_ INuiFusionCameraPoseFinder **ppNuiFusionCameraPoseFinder
    );



extern RPC_IF_HANDLE __MIDL_itf_KinectFusionCameraPoseFinder_0000_0001_v0_0_c_ifspec;
extern RPC_IF_HANDLE __MIDL_itf_KinectFusionCameraPoseFinder_0000_0001_v0_0_s_ifspec;

/* Additional Prototypes for ALL interfaces */

/* end of Additional Prototypes */

#ifdef __cplusplus
}
#endif

#endif


