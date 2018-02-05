

/* this ALWAYS GENERATED file contains the definitions for the interfaces */


 /* File created by MIDL compiler version 8.00.0595 */
/* at Sun Oct 19 12:58:11 2014
 */
/* Compiler settings for ..\inc_public\KinectFusion.idl:
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


#ifndef __NuiKinectFusionVolume_h__
#define __NuiKinectFusionVolume_h__

#if defined(_MSC_VER) && (_MSC_VER >= 1020)
#pragma once
#endif

/* Forward Declarations */ 

#ifndef __INuiFusionMesh_FWD_DEFINED__
#define __INuiFusionMesh_FWD_DEFINED__
typedef interface INuiFusionMesh INuiFusionMesh;

#endif 	/* __INuiFusionMesh_FWD_DEFINED__ */


#ifndef __INuiFusionReconstruction_FWD_DEFINED__
#define __INuiFusionReconstruction_FWD_DEFINED__
typedef interface INuiFusionReconstruction INuiFusionReconstruction;

#endif 	/* __INuiFusionReconstruction_FWD_DEFINED__ */


/* header files for imported files */
#include "oaidl.h"
#include "ocidl.h"

#ifdef __cplusplus
extern "C"{
#endif 


/* interface __MIDL_itf_KinectFusion_0000_0000 */
/* [local] */ 

///----------------------------------------------------------------------------
/// <copyright file="NuiKinectFusionVolume.h" company="Microsoft">
/// Copyright (c) Microsoft Corporation. All rights reserved.>
/// </copyright>
///----------------------------------------------------------------------------

#ifndef FUSIONAPI
#error "You must include NuiKinectFusionApi.h rather than including NuiKinectFusionVolume.h directly"
#endif

#ifndef DEFINE_UUID
#define DEFINE_UUID(name, l, w1, w2, b1, b2, b3, b4, b5, b6, b7, b8) extern "C"  const GUID __declspec(selectany) name = { l, w1, w2, { b1, b2,  b3,  b4,  b5,  b6,  b7,  b8 } }
#endif


DEFINE_UUID(LIBID_KinectFusion, 0x7b3762f5, 0xd97f, 0x405f, 0xa9, 0x66, 0x79, 0x1, 0x8e, 0x9f, 0x64, 0xf6);

// AC19AD5F-2218-4C08-A00F-C981C50A09DF
DEFINE_UUID(IID_INuiFusionReconstruction, 0xac19ad5f, 0x2218, 0x4c08, 0xa0, 0xf, 0xc9, 0x81, 0xc5, 0xa, 0x9, 0xdf);

// 43102C25-82EE-4958-AA24-AC5456BF83C0
DEFINE_UUID(IID_INuiFusionMesh, 0x43102c25, 0x82ee, 0x4958, 0xaa, 0x24, 0xac, 0x54, 0x56, 0xbf, 0x83, 0xc0);

#include <Kinect.h>
#include "NuiKinectFusionBase.h"

#include <pshpack8.h>

#ifndef _Vector3_
#define _Vector3_
typedef struct _Vector3
    {
    FLOAT x;
    FLOAT y;
    FLOAT z;
    } 	Vector3;

#endif _Vector3_
#ifndef _NUI_FUSION_CAMERA_PARAMETERS_
#define _NUI_FUSION_CAMERA_PARAMETERS_
typedef struct _NUI_FUSION_CAMERA_PARAMETERS
    {
    FLOAT focalLengthX;
    FLOAT focalLengthY;
    FLOAT principalPointX;
    FLOAT principalPointY;
    } 	NUI_FUSION_CAMERA_PARAMETERS;

#endif // NUI_FUSION_CAMERA_PARAMETERS_
#ifndef _NUI_FUSION_IMAGE_TYPE_
#define _NUI_FUSION_IMAGE_TYPE_
typedef 
enum _NUI_FUSION_IMAGE_TYPE
    {
        NUI_FUSION_IMAGE_TYPE_INVALID	= 0,
        NUI_FUSION_IMAGE_TYPE_COLOR	= 1,
        NUI_FUSION_IMAGE_TYPE_FLOAT	= 2,
        NUI_FUSION_IMAGE_TYPE_POINT_CLOUD	= 3
    } 	NUI_FUSION_IMAGE_TYPE;

#endif _NUI_FUSION_IMAGE_TYPE_
#ifndef _NUI_FUSION_RECONSTRUCTION_PARAMETERS_
#define _NUI_FUSION_RECONSTRUCTION_PARAMETERS_
typedef struct _NUI_FUSION_RECONSTRUCTION_PARAMETERS
    {
    FLOAT voxelsPerMeter;
    UINT voxelCountX;
    UINT voxelCountY;
    UINT voxelCountZ;
    } 	NUI_FUSION_RECONSTRUCTION_PARAMETERS;

#endif _NUI_FUSION_RECONSTRUCTION_PARAMETERS_
#ifndef _NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_
#define _NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_
typedef 
enum _NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE
    {
        NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_CPU	= 1,
        NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_AMP	= 2
    } 	NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE;

#endif _NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_


extern RPC_IF_HANDLE __MIDL_itf_KinectFusion_0000_0000_v0_0_c_ifspec;
extern RPC_IF_HANDLE __MIDL_itf_KinectFusion_0000_0000_v0_0_s_ifspec;






/* [helpstring][version][uuid] */ 

#ifndef _NUI_FUSION_IMAGE_FRAME_
#define _NUI_FUSION_IMAGE_FRAME_
typedef struct _NUI_FUSION_IMAGE_FRAME
    {
    UINT width;
    UINT height;
    NUI_FUSION_IMAGE_TYPE imageType;
    NUI_FUSION_CAMERA_PARAMETERS *pCameraParameters;
    NUI_FUSION_BUFFER *pFrameBuffer;
    } 	NUI_FUSION_IMAGE_FRAME;

#endif _NUI_FUSION_IMAGE_FRAME_






#ifndef __INuiFusionMesh_INTERFACE_DEFINED__
#define __INuiFusionMesh_INTERFACE_DEFINED__

/* interface INuiFusionMesh */
/* [object][local][uuid] */ 


EXTERN_C const IID IID_INuiFusionMesh;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("43102C25-82EE-4958-AA24-AC5456BF83C0")
    INuiFusionMesh : public IUnknown
    {
    public:
        virtual UINT STDMETHODCALLTYPE VertexCount( void) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE GetVertices( 
            /* [out] */ const Vector3 **pVertices) = 0;
        
        virtual UINT STDMETHODCALLTYPE NormalCount( void) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE GetNormals( 
            /* [out] */ const Vector3 **pNormals) = 0;
        
        virtual UINT STDMETHODCALLTYPE TriangleVertexIndexCount( void) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE GetTriangleIndices( 
            /* [out] */ const int **pTriangleVertexIndices) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct INuiFusionMeshVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            INuiFusionMesh * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            INuiFusionMesh * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            INuiFusionMesh * This);
        
        UINT ( STDMETHODCALLTYPE *VertexCount )( 
            INuiFusionMesh * This);
        
        HRESULT ( STDMETHODCALLTYPE *GetVertices )( 
            INuiFusionMesh * This,
            /* [out] */ const Vector3 **pVertices);
        
        UINT ( STDMETHODCALLTYPE *NormalCount )( 
            INuiFusionMesh * This);
        
        HRESULT ( STDMETHODCALLTYPE *GetNormals )( 
            INuiFusionMesh * This,
            /* [out] */ const Vector3 **pNormals);
        
        UINT ( STDMETHODCALLTYPE *TriangleVertexIndexCount )( 
            INuiFusionMesh * This);
        
        HRESULT ( STDMETHODCALLTYPE *GetTriangleIndices )( 
            INuiFusionMesh * This,
            /* [out] */ const int **pTriangleVertexIndices);
        
        END_INTERFACE
    } INuiFusionMeshVtbl;

    interface INuiFusionMesh
    {
        CONST_VTBL struct INuiFusionMeshVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define INuiFusionMesh_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define INuiFusionMesh_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define INuiFusionMesh_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define INuiFusionMesh_VertexCount(This)	\
    ( (This)->lpVtbl -> VertexCount(This) ) 

#define INuiFusionMesh_GetVertices(This,pVertices)	\
    ( (This)->lpVtbl -> GetVertices(This,pVertices) ) 

#define INuiFusionMesh_NormalCount(This)	\
    ( (This)->lpVtbl -> NormalCount(This) ) 

#define INuiFusionMesh_GetNormals(This,pNormals)	\
    ( (This)->lpVtbl -> GetNormals(This,pNormals) ) 

#define INuiFusionMesh_TriangleVertexIndexCount(This)	\
    ( (This)->lpVtbl -> TriangleVertexIndexCount(This) ) 

#define INuiFusionMesh_GetTriangleIndices(This,pTriangleVertexIndices)	\
    ( (This)->lpVtbl -> GetTriangleIndices(This,pTriangleVertexIndices) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __INuiFusionMesh_INTERFACE_DEFINED__ */


#ifndef __INuiFusionReconstruction_INTERFACE_DEFINED__
#define __INuiFusionReconstruction_INTERFACE_DEFINED__

/* interface INuiFusionReconstruction */
/* [object][local][uuid] */ 


EXTERN_C const IID IID_INuiFusionReconstruction;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("AC19AD5F-2218-4C08-A00F-C981C50A09DF")
    INuiFusionReconstruction : public IUnknown
    {
    public:
        virtual HRESULT STDMETHODCALLTYPE ResetReconstruction( 
            /* [in] */ const Matrix4 *pInitialWorldToCameraTransform,
            /* [in] */ const Matrix4 *pWorldToVolumeTransform) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE AlignDepthFloatToReconstruction( 
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pDepthFloatFrame,
            /* [in] */ USHORT maxAlignIterationCount,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pDeltaFromReferenceFrame,
            /* [out] */ FLOAT *pAlignmentEnergy,
            /* [in] */ const Matrix4 *pWorldToCameraTransform) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE GetCurrentWorldToCameraTransform( 
            /* [out] */ Matrix4 *pWorldToCameraTransform) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE GetCurrentWorldToVolumeTransform( 
            /* [out] */ Matrix4 *pWorldToVolumeTransform) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE IntegrateFrame( 
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pDepthFloatFrame,
            /* [in] */ USHORT maxIntegrationWeight,
            /* [in] */ const Matrix4 *pWorldToCameraTransform) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE ProcessFrame( 
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pDepthFloatFrame,
            /* [in] */ USHORT maxAlignIterationCount,
            /* [in] */ USHORT maxIntegrationWeight,
            /* [out] */ FLOAT *pAlignmentEnergy,
            /* [in] */ const Matrix4 *pWorldToCameraTransform) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE CalculatePointCloud( 
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pPointCloudFrame,
            /* [in] */ const Matrix4 *pWorldToCameraTransform) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE CalculateMesh( 
            /* [in] */ UINT voxelStep,
            /* [out] */ INuiFusionMesh **ppMesh) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE ExportVolumeBlock( 
            /* [in] */ UINT sourceOriginX,
            /* [in] */ UINT sourceOriginY,
            /* [in] */ UINT sourceOriginZ,
            /* [in] */ UINT destinationResolutionX,
            /* [in] */ UINT destinationResolutionY,
            /* [in] */ UINT destinationResolutionZ,
            /* [in] */ UINT voxelStep,
            /* [in] */ UINT cbVolumeBlock,
            /* [out] */ SHORT *pVolumeBlock) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE ImportVolumeBlock( 
            /* [in] */ UINT cbVolumeBlock,
            /* [in] */ const SHORT *pVolumeBlock) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE DepthToDepthFloatFrame( 
            /* [in] */ const UINT16 *pDepthImageData,
            /* [in] */ UINT countDepthImageDataBytes,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pDepthFloatFrame,
            /* [in] */ FLOAT minDepthClip,
            /* [in] */ FLOAT maxDepthClip,
            /* [in] */ BOOL mirrorDepth) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE SmoothDepthFloatFrame( 
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pDepthFloatFrame,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pSmoothDepthFloatFrame,
            /* [in] */ UINT kernelWidth,
            /* [in] */ FLOAT distanceThreshold) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE AlignPointClouds( 
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pReferencePointCloudFrame,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pObservedPointCloudFrame,
            /* [in] */ USHORT maxAlignIterationCount,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pDeltaFromReferenceFrame,
            /* [out] */ FLOAT *pAlignmentEnergy,
            /* [out][in] */ Matrix4 *pReferenceToObservedTransform) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE SetAlignDepthFloatToReconstructionReferenceFrame( 
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pReferenceDepthFloatFrame) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE CalculatePointCloudAndDepth( 
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pPointCloudFrame,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pDepthFloatFrame,
            /* [in] */ const Matrix4 *pWorldToCameraTransform) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct INuiFusionReconstructionVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            INuiFusionReconstruction * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            INuiFusionReconstruction * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            INuiFusionReconstruction * This);
        
        HRESULT ( STDMETHODCALLTYPE *ResetReconstruction )( 
            INuiFusionReconstruction * This,
            /* [in] */ const Matrix4 *pInitialWorldToCameraTransform,
            /* [in] */ const Matrix4 *pWorldToVolumeTransform);
        
        HRESULT ( STDMETHODCALLTYPE *AlignDepthFloatToReconstruction )( 
            INuiFusionReconstruction * This,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pDepthFloatFrame,
            /* [in] */ USHORT maxAlignIterationCount,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pDeltaFromReferenceFrame,
            /* [out] */ FLOAT *pAlignmentEnergy,
            /* [in] */ const Matrix4 *pWorldToCameraTransform);
        
        HRESULT ( STDMETHODCALLTYPE *GetCurrentWorldToCameraTransform )( 
            INuiFusionReconstruction * This,
            /* [out] */ Matrix4 *pWorldToCameraTransform);
        
        HRESULT ( STDMETHODCALLTYPE *GetCurrentWorldToVolumeTransform )( 
            INuiFusionReconstruction * This,
            /* [out] */ Matrix4 *pWorldToVolumeTransform);
        
        HRESULT ( STDMETHODCALLTYPE *IntegrateFrame )( 
            INuiFusionReconstruction * This,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pDepthFloatFrame,
            /* [in] */ USHORT maxIntegrationWeight,
            /* [in] */ const Matrix4 *pWorldToCameraTransform);
        
        HRESULT ( STDMETHODCALLTYPE *ProcessFrame )( 
            INuiFusionReconstruction * This,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pDepthFloatFrame,
            /* [in] */ USHORT maxAlignIterationCount,
            /* [in] */ USHORT maxIntegrationWeight,
            /* [out] */ FLOAT *pAlignmentEnergy,
            /* [in] */ const Matrix4 *pWorldToCameraTransform);
        
        HRESULT ( STDMETHODCALLTYPE *CalculatePointCloud )( 
            INuiFusionReconstruction * This,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pPointCloudFrame,
            /* [in] */ const Matrix4 *pWorldToCameraTransform);
        
        HRESULT ( STDMETHODCALLTYPE *CalculateMesh )( 
            INuiFusionReconstruction * This,
            /* [in] */ UINT voxelStep,
            /* [out] */ INuiFusionMesh **ppMesh);
        
        HRESULT ( STDMETHODCALLTYPE *ExportVolumeBlock )( 
            INuiFusionReconstruction * This,
            /* [in] */ UINT sourceOriginX,
            /* [in] */ UINT sourceOriginY,
            /* [in] */ UINT sourceOriginZ,
            /* [in] */ UINT destinationResolutionX,
            /* [in] */ UINT destinationResolutionY,
            /* [in] */ UINT destinationResolutionZ,
            /* [in] */ UINT voxelStep,
            /* [in] */ UINT cbVolumeBlock,
            /* [out] */ SHORT *pVolumeBlock);
        
        HRESULT ( STDMETHODCALLTYPE *ImportVolumeBlock )( 
            INuiFusionReconstruction * This,
            /* [in] */ UINT cbVolumeBlock,
            /* [in] */ const SHORT *pVolumeBlock);
        
        HRESULT ( STDMETHODCALLTYPE *DepthToDepthFloatFrame )( 
            INuiFusionReconstruction * This,
            /* [in] */ const UINT16 *pDepthImageData,
            /* [in] */ UINT countDepthImageDataBytes,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pDepthFloatFrame,
            /* [in] */ FLOAT minDepthClip,
            /* [in] */ FLOAT maxDepthClip,
            /* [in] */ BOOL mirrorDepth);
        
        HRESULT ( STDMETHODCALLTYPE *SmoothDepthFloatFrame )( 
            INuiFusionReconstruction * This,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pDepthFloatFrame,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pSmoothDepthFloatFrame,
            /* [in] */ UINT kernelWidth,
            /* [in] */ FLOAT distanceThreshold);
        
        HRESULT ( STDMETHODCALLTYPE *AlignPointClouds )( 
            INuiFusionReconstruction * This,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pReferencePointCloudFrame,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pObservedPointCloudFrame,
            /* [in] */ USHORT maxAlignIterationCount,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pDeltaFromReferenceFrame,
            /* [out] */ FLOAT *pAlignmentEnergy,
            /* [out][in] */ Matrix4 *pReferenceToObservedTransform);
        
        HRESULT ( STDMETHODCALLTYPE *SetAlignDepthFloatToReconstructionReferenceFrame )( 
            INuiFusionReconstruction * This,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pReferenceDepthFloatFrame);
        
        HRESULT ( STDMETHODCALLTYPE *CalculatePointCloudAndDepth )( 
            INuiFusionReconstruction * This,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pPointCloudFrame,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pDepthFloatFrame,
            /* [in] */ const Matrix4 *pWorldToCameraTransform);
        
        END_INTERFACE
    } INuiFusionReconstructionVtbl;

    interface INuiFusionReconstruction
    {
        CONST_VTBL struct INuiFusionReconstructionVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define INuiFusionReconstruction_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define INuiFusionReconstruction_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define INuiFusionReconstruction_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define INuiFusionReconstruction_ResetReconstruction(This,pInitialWorldToCameraTransform,pWorldToVolumeTransform)	\
    ( (This)->lpVtbl -> ResetReconstruction(This,pInitialWorldToCameraTransform,pWorldToVolumeTransform) ) 

#define INuiFusionReconstruction_AlignDepthFloatToReconstruction(This,pDepthFloatFrame,maxAlignIterationCount,pDeltaFromReferenceFrame,pAlignmentEnergy,pWorldToCameraTransform)	\
    ( (This)->lpVtbl -> AlignDepthFloatToReconstruction(This,pDepthFloatFrame,maxAlignIterationCount,pDeltaFromReferenceFrame,pAlignmentEnergy,pWorldToCameraTransform) ) 

#define INuiFusionReconstruction_GetCurrentWorldToCameraTransform(This,pWorldToCameraTransform)	\
    ( (This)->lpVtbl -> GetCurrentWorldToCameraTransform(This,pWorldToCameraTransform) ) 

#define INuiFusionReconstruction_GetCurrentWorldToVolumeTransform(This,pWorldToVolumeTransform)	\
    ( (This)->lpVtbl -> GetCurrentWorldToVolumeTransform(This,pWorldToVolumeTransform) ) 

#define INuiFusionReconstruction_IntegrateFrame(This,pDepthFloatFrame,maxIntegrationWeight,pWorldToCameraTransform)	\
    ( (This)->lpVtbl -> IntegrateFrame(This,pDepthFloatFrame,maxIntegrationWeight,pWorldToCameraTransform) ) 

#define INuiFusionReconstruction_ProcessFrame(This,pDepthFloatFrame,maxAlignIterationCount,maxIntegrationWeight,pAlignmentEnergy,pWorldToCameraTransform)	\
    ( (This)->lpVtbl -> ProcessFrame(This,pDepthFloatFrame,maxAlignIterationCount,maxIntegrationWeight,pAlignmentEnergy,pWorldToCameraTransform) ) 

#define INuiFusionReconstruction_CalculatePointCloud(This,pPointCloudFrame,pWorldToCameraTransform)	\
    ( (This)->lpVtbl -> CalculatePointCloud(This,pPointCloudFrame,pWorldToCameraTransform) ) 

#define INuiFusionReconstruction_CalculateMesh(This,voxelStep,ppMesh)	\
    ( (This)->lpVtbl -> CalculateMesh(This,voxelStep,ppMesh) ) 

#define INuiFusionReconstruction_ExportVolumeBlock(This,sourceOriginX,sourceOriginY,sourceOriginZ,destinationResolutionX,destinationResolutionY,destinationResolutionZ,voxelStep,cbVolumeBlock,pVolumeBlock)	\
    ( (This)->lpVtbl -> ExportVolumeBlock(This,sourceOriginX,sourceOriginY,sourceOriginZ,destinationResolutionX,destinationResolutionY,destinationResolutionZ,voxelStep,cbVolumeBlock,pVolumeBlock) ) 

#define INuiFusionReconstruction_ImportVolumeBlock(This,cbVolumeBlock,pVolumeBlock)	\
    ( (This)->lpVtbl -> ImportVolumeBlock(This,cbVolumeBlock,pVolumeBlock) ) 

#define INuiFusionReconstruction_DepthToDepthFloatFrame(This,pDepthImageData,countDepthImageDataBytes,pDepthFloatFrame,minDepthClip,maxDepthClip,mirrorDepth)	\
    ( (This)->lpVtbl -> DepthToDepthFloatFrame(This,pDepthImageData,countDepthImageDataBytes,pDepthFloatFrame,minDepthClip,maxDepthClip,mirrorDepth) ) 

#define INuiFusionReconstruction_SmoothDepthFloatFrame(This,pDepthFloatFrame,pSmoothDepthFloatFrame,kernelWidth,distanceThreshold)	\
    ( (This)->lpVtbl -> SmoothDepthFloatFrame(This,pDepthFloatFrame,pSmoothDepthFloatFrame,kernelWidth,distanceThreshold) ) 

#define INuiFusionReconstruction_AlignPointClouds(This,pReferencePointCloudFrame,pObservedPointCloudFrame,maxAlignIterationCount,pDeltaFromReferenceFrame,pAlignmentEnergy,pReferenceToObservedTransform)	\
    ( (This)->lpVtbl -> AlignPointClouds(This,pReferencePointCloudFrame,pObservedPointCloudFrame,maxAlignIterationCount,pDeltaFromReferenceFrame,pAlignmentEnergy,pReferenceToObservedTransform) ) 

#define INuiFusionReconstruction_SetAlignDepthFloatToReconstructionReferenceFrame(This,pReferenceDepthFloatFrame)	\
    ( (This)->lpVtbl -> SetAlignDepthFloatToReconstructionReferenceFrame(This,pReferenceDepthFloatFrame) ) 

#define INuiFusionReconstruction_CalculatePointCloudAndDepth(This,pPointCloudFrame,pDepthFloatFrame,pWorldToCameraTransform)	\
    ( (This)->lpVtbl -> CalculatePointCloudAndDepth(This,pPointCloudFrame,pDepthFloatFrame,pWorldToCameraTransform) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __INuiFusionReconstruction_INTERFACE_DEFINED__ */



/* interface __MIDL_itf_KinectFusion_0000_0001 */
/* [local] */ 


#include <poppack.h>

// <summary>
/// Enumerate the devices capable of running KinectFusion.
/// This enables a specific device to be chosen when calling NuiFusionCreateReconstruction if desired.
/// </summary>
/// <param name="type">The type of processor to enumerate.</param>
/// <param name="index">The zero-based index of the device for which the description is returned,
/// or -1 to get the default device for the given processor type.</param>
/// <param name="pDescription">An optional pointer to a buffer that receives a description
/// string for the device.</param>
/// <param name="descriptionSizeInChars">The size of the buffer pointed to by
/// <paramref name="pDescription"/>, in characters.</param>
/// <param name="pInstancePath">An optional pointer to a buffer that receives an instance path
/// string for the device.</param>
/// <param name="instancePathSizeInChars">The size of the buffer pointed to by
/// <paramref name="pInstancePath"/>, in characters.</param>
/// <param name="pMemoryKB">An optional pointer to an unsigned integer that receives the total
/// amount of memory on the device, in kilobytes.</param>
/// <returns>
/// <para>Returns S_OK if successful; otherwise, returns one of the following failure codes:
/// <list type="table">
///    <listheader>
///       <term>Error code</term>
///       <description>Description</description>
///    </listheader>
///    <item>
///       <term>E_NUI_BADINDEX</term>
///       <description>The index parameter is out of range for the specified processor type.</description>
///    </item>
///    <item>
///       <term>E_INVALIDARG</term>
///       <description>The <paramref name="type"/> parameter is out of range.</description>
///    </item>
///    <item>
///       <term>E_FAIL</term>
///       <description>The call failed for an unknown reason.</description>
///    </item>
/// </list>
/// </returns>
_Check_return_ HRESULT FUSIONAPI NuiFusionGetDeviceInfo(
    _In_ _NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE type,
    _In_ INT index,
    _Out_opt_z_cap_(descriptionSizeInChars) PWSTR pDescription,
    _In_ UINT descriptionSizeInChars,
    _Out_opt_z_cap_(instancePathSizeInChars) PWSTR pInstancePath,
    _In_ UINT instancePathSizeInChars,
    _Out_opt_ UINT* pMemoryKB
    );

// <summary>
/// Initialize a Kinect Fusion 3D Reconstruction Volume.
/// Voxel volume axis sizes must be greater than 0 and a multiple of 32, and the voxelsPerMeter
/// must be greater than 0. A Kinect camera is also required to be connected.
/// Note: Users must call Release on the interface when finished with the volume.
/// </summary>
/// <param name="pVolumeParameters">
/// A pointer to an instance of NUI_FUSION_RECONSTRUCTION_PARAMETERS, defining the size and shape
/// of the reconstruction volume.
/// </param>
/// <param name="reconstructionProcessorType">
/// On calling set to a NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE, defining the processor type
/// to be used for all calls to the reconstruction volume object returned from this function.
/// </param>
/// <param name="deviceIndex">Set this variable to an explicit zero-based device index to use
/// a specific device as enumerated by NuiFusionGetDeviceInfo, or set to -1 to automatically
/// select the default device for a given processor type.
/// </param>
/// <param name="pInitialWorldToCameraTransform">
/// Optionally, a pointer to a pre-allocated Matrix4 instance, containing the initial camera pose
/// with respect to the world origin. Pass NULL to set the default initial camera pose (identity).
/// </param>
/// <param name ="ppNuiFusionReconstruction">
/// A pointer that receives a reference to the created INuiFusionReconstruction interface.
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
///       <description>The <paramref name="pReconstructonParameters"/> parameter is NULL, or the
///       <paramref name="pReconstructonParameters"/> parameter's <c>voxelCountX<c>, <c>voxelCountY<c>,
///       or <c>voxelCountZ<c> member is not a greater than 0 and multiple of 32, or the
///       <c>voxelsPerMeter<c> member is not greater than 0, or the <paramref name="deviceIndex"/>
///       parameter is less than -1 or greater than the number of available devices for the respective
///       processor type.</description>
///    </item>
///    <item>
///       <term>E_POINTER</term>
///       <description>The <paramref name="ppNuiFusionReconstruction"/> parameter is NULL.</description>
///    </item>
///    <item>
///       <term>E_OUTOFMEMORY</term>
///       <description>A memory allocation failed.</description>
///    </item>
///    <item>
///       <term>E_NUI_GPU_OUTOFMEMORY</term>
///       <description>The <paramref name="pVolumeParameters"/> parameter specifies a volume
///       too big, device memory allocation failed.</description>
///    </item>
///    <item>
///       <term>E_NUI_GPU_FAIL</term>
///       <description>The <paramref name="deviceIndex"/> parameter is valid, but the requested
///       device is not capable of running Kinect Fusion, or there was a problem initializing the
///       device.</description>
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
/// <remarks>
/// Users can select which device the processing is performed on with the 
/// <paramref name="reconstructionProcessorType"/> parameter.
/// For those with multiple devices the <paramref name="pDeviceIndex"/> parameter also enables
/// users to explicitly configure on which device the reconstruction volume is created.
/// Note that this function creates a default world-volume transform. To set a non-default
/// transform call ResetReconstruction with an appropriate Matrix4. This default transformation
/// is a combination of translation in X,Y to locate the world origin at the center of the front
/// face of the reconstruction volume cube, and scaling by the voxelsPerMeter reconstruction
/// parameter to convert from the world coordinate system to volume voxel indices.
/// </remarks>
_Check_return_ HRESULT FUSIONAPI NuiFusionCreateReconstruction (
    _In_ const NUI_FUSION_RECONSTRUCTION_PARAMETERS *pReconstructionParameters,
    _In_ NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE reconstructionProcessorType,
    _In_ INT deviceIndex,
    _In_opt_ const Matrix4 *pInitialWorldToCameraTransform,
    _Deref_out_ INuiFusionReconstruction **ppNuiFusionReconstruction
    );



extern RPC_IF_HANDLE __MIDL_itf_KinectFusion_0000_0001_v0_0_c_ifspec;
extern RPC_IF_HANDLE __MIDL_itf_KinectFusion_0000_0001_v0_0_s_ifspec;

/* Additional Prototypes for ALL interfaces */

/* end of Additional Prototypes */

#ifdef __cplusplus
}
#endif

#endif


