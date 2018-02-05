

/* this ALWAYS GENERATED file contains the definitions for the interfaces */


 /* File created by MIDL compiler version 8.00.0595 */
/* at Sun Oct 19 12:58:14 2014
 */
/* Compiler settings for ..\inc_public\KinectFusionColor.idl:
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


#ifndef __NuiKinectFusionColorVolume_h__
#define __NuiKinectFusionColorVolume_h__

#if defined(_MSC_VER) && (_MSC_VER >= 1020)
#pragma once
#endif

/* Forward Declarations */ 

#ifndef __INuiFusionColorMesh_FWD_DEFINED__
#define __INuiFusionColorMesh_FWD_DEFINED__
typedef interface INuiFusionColorMesh INuiFusionColorMesh;

#endif 	/* __INuiFusionColorMesh_FWD_DEFINED__ */


#ifndef __INuiFusionColorReconstruction_FWD_DEFINED__
#define __INuiFusionColorReconstruction_FWD_DEFINED__
typedef interface INuiFusionColorReconstruction INuiFusionColorReconstruction;

#endif 	/* __INuiFusionColorReconstruction_FWD_DEFINED__ */


/* header files for imported files */
#include "oaidl.h"
#include "ocidl.h"

#ifdef __cplusplus
extern "C"{
#endif 


/* interface __MIDL_itf_KinectFusionColor_0000_0000 */
/* [local] */ 

///----------------------------------------------------------------------------
/// <copyright file="NuiKinectFusionColorVolume.h" company="Microsoft">
/// Copyright (c) Microsoft Corporation. All rights reserved.>
/// </copyright>
///----------------------------------------------------------------------------

#ifndef FUSIONAPI
#error "You must include NuiKinectFusionApi.h rather than including NuiKinectFusionColorVolume.h directly"
#endif

#ifndef DEFINE_UUID
#define DEFINE_UUID(name, l, w1, w2, b1, b2, b3, b4, b5, b6, b7, b8) extern "C"  const GUID __declspec(selectany) name = { l, w1, w2, { b1, b2,  b3,  b4,  b5,  b6,  b7,  b8 } }
#endif

// A833A9A1-3608-49FA-982A-E050BFFD8C96
DEFINE_UUID(IID_INuiFusionColorReconstruction, 0xa833a9a1, 0x3608, 0x49fa, 0x98, 0x2a, 0xe0, 0x50, 0xbf, 0xfd, 0x8c, 0x96);

// 461C71E1-2017-4322-AEA3-303BB8DC545C
DEFINE_UUID(IID_INuiFusionColorMesh, 0x461C71E1, 0x2017, 0x4322, 0xAE, 0xA3, 0x30, 0x3B, 0xB8, 0xDC, 0x54, 0x5C);

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


extern RPC_IF_HANDLE __MIDL_itf_KinectFusionColor_0000_0000_v0_0_c_ifspec;
extern RPC_IF_HANDLE __MIDL_itf_KinectFusionColor_0000_0000_v0_0_s_ifspec;






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






#ifndef __INuiFusionColorMesh_INTERFACE_DEFINED__
#define __INuiFusionColorMesh_INTERFACE_DEFINED__

/* interface INuiFusionColorMesh */
/* [object][local][uuid] */ 


EXTERN_C const IID IID_INuiFusionColorMesh;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("461C71E1-2017-4322-AEA3-303BB8DC545C")
    INuiFusionColorMesh : public IUnknown
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
        
        virtual UINT STDMETHODCALLTYPE ColorCount( void) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE GetColors( 
            /* [out] */ const int **pColors) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct INuiFusionColorMeshVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            INuiFusionColorMesh * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            INuiFusionColorMesh * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            INuiFusionColorMesh * This);
        
        UINT ( STDMETHODCALLTYPE *VertexCount )( 
            INuiFusionColorMesh * This);
        
        HRESULT ( STDMETHODCALLTYPE *GetVertices )( 
            INuiFusionColorMesh * This,
            /* [out] */ const Vector3 **pVertices);
        
        UINT ( STDMETHODCALLTYPE *NormalCount )( 
            INuiFusionColorMesh * This);
        
        HRESULT ( STDMETHODCALLTYPE *GetNormals )( 
            INuiFusionColorMesh * This,
            /* [out] */ const Vector3 **pNormals);
        
        UINT ( STDMETHODCALLTYPE *TriangleVertexIndexCount )( 
            INuiFusionColorMesh * This);
        
        HRESULT ( STDMETHODCALLTYPE *GetTriangleIndices )( 
            INuiFusionColorMesh * This,
            /* [out] */ const int **pTriangleVertexIndices);
        
        UINT ( STDMETHODCALLTYPE *ColorCount )( 
            INuiFusionColorMesh * This);
        
        HRESULT ( STDMETHODCALLTYPE *GetColors )( 
            INuiFusionColorMesh * This,
            /* [out] */ const int **pColors);
        
        END_INTERFACE
    } INuiFusionColorMeshVtbl;

    interface INuiFusionColorMesh
    {
        CONST_VTBL struct INuiFusionColorMeshVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define INuiFusionColorMesh_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define INuiFusionColorMesh_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define INuiFusionColorMesh_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define INuiFusionColorMesh_VertexCount(This)	\
    ( (This)->lpVtbl -> VertexCount(This) ) 

#define INuiFusionColorMesh_GetVertices(This,pVertices)	\
    ( (This)->lpVtbl -> GetVertices(This,pVertices) ) 

#define INuiFusionColorMesh_NormalCount(This)	\
    ( (This)->lpVtbl -> NormalCount(This) ) 

#define INuiFusionColorMesh_GetNormals(This,pNormals)	\
    ( (This)->lpVtbl -> GetNormals(This,pNormals) ) 

#define INuiFusionColorMesh_TriangleVertexIndexCount(This)	\
    ( (This)->lpVtbl -> TriangleVertexIndexCount(This) ) 

#define INuiFusionColorMesh_GetTriangleIndices(This,pTriangleVertexIndices)	\
    ( (This)->lpVtbl -> GetTriangleIndices(This,pTriangleVertexIndices) ) 

#define INuiFusionColorMesh_ColorCount(This)	\
    ( (This)->lpVtbl -> ColorCount(This) ) 

#define INuiFusionColorMesh_GetColors(This,pColors)	\
    ( (This)->lpVtbl -> GetColors(This,pColors) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __INuiFusionColorMesh_INTERFACE_DEFINED__ */


#ifndef __INuiFusionColorReconstruction_INTERFACE_DEFINED__
#define __INuiFusionColorReconstruction_INTERFACE_DEFINED__

/* interface INuiFusionColorReconstruction */
/* [object][local][uuid] */ 


EXTERN_C const IID IID_INuiFusionColorReconstruction;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("A833A9A1-3608-49FA-982A-E050BFFD8C96")
    INuiFusionColorReconstruction : public IUnknown
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
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pColorFrame,
            /* [in] */ USHORT maxIntegrationWeight,
            /* [in] */ FLOAT maxColorIntegrationAngle,
            /* [in] */ const Matrix4 *pWorldToCameraTransform) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE ProcessFrame( 
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pDepthFloatFrame,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pColorFrame,
            /* [in] */ USHORT maxAlignIterationCount,
            /* [in] */ USHORT maxIntegrationWeight,
            /* [in] */ FLOAT maxColorIntegrationAngle,
            /* [out] */ FLOAT *pAlignmentEnergy,
            /* [in] */ const Matrix4 *pWorldToCameraTransform) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE CalculatePointCloud( 
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pPointCloudFrame,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pColorFrame,
            /* [in] */ const Matrix4 *pWorldToCameraTransform) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE CalculateMesh( 
            /* [in] */ UINT voxelStep,
            /* [out] */ INuiFusionColorMesh **ppMesh) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE ExportVolumeBlock( 
            /* [in] */ UINT sourceOriginX,
            /* [in] */ UINT sourceOriginY,
            /* [in] */ UINT sourceOriginZ,
            /* [in] */ UINT destinationResolutionX,
            /* [in] */ UINT destinationResolutionY,
            /* [in] */ UINT destinationResolutionZ,
            /* [in] */ UINT voxelStep,
            /* [in] */ UINT cbVolumeBlock,
            /* [in] */ UINT cbColorVolumeBlock,
            /* [out] */ SHORT *pVolumeBlock,
            /* [out] */ INT *pColorVolumeBlock) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE ImportVolumeBlock( 
            /* [in] */ UINT cbVolumeBlock,
            /* [in] */ UINT cbColorVolumeBlock,
            /* [in] */ const SHORT *pVolumeBlock,
            /* [in] */ const INT *pColorVolumeBlock) = 0;
        
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
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pColorFrame,
            /* [in] */ const Matrix4 *pWorldToCameraTransform) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct INuiFusionColorReconstructionVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            INuiFusionColorReconstruction * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            INuiFusionColorReconstruction * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            INuiFusionColorReconstruction * This);
        
        HRESULT ( STDMETHODCALLTYPE *ResetReconstruction )( 
            INuiFusionColorReconstruction * This,
            /* [in] */ const Matrix4 *pInitialWorldToCameraTransform,
            /* [in] */ const Matrix4 *pWorldToVolumeTransform);
        
        HRESULT ( STDMETHODCALLTYPE *AlignDepthFloatToReconstruction )( 
            INuiFusionColorReconstruction * This,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pDepthFloatFrame,
            /* [in] */ USHORT maxAlignIterationCount,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pDeltaFromReferenceFrame,
            /* [out] */ FLOAT *pAlignmentEnergy,
            /* [in] */ const Matrix4 *pWorldToCameraTransform);
        
        HRESULT ( STDMETHODCALLTYPE *GetCurrentWorldToCameraTransform )( 
            INuiFusionColorReconstruction * This,
            /* [out] */ Matrix4 *pWorldToCameraTransform);
        
        HRESULT ( STDMETHODCALLTYPE *GetCurrentWorldToVolumeTransform )( 
            INuiFusionColorReconstruction * This,
            /* [out] */ Matrix4 *pWorldToVolumeTransform);
        
        HRESULT ( STDMETHODCALLTYPE *IntegrateFrame )( 
            INuiFusionColorReconstruction * This,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pDepthFloatFrame,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pColorFrame,
            /* [in] */ USHORT maxIntegrationWeight,
            /* [in] */ FLOAT maxColorIntegrationAngle,
            /* [in] */ const Matrix4 *pWorldToCameraTransform);
        
        HRESULT ( STDMETHODCALLTYPE *ProcessFrame )( 
            INuiFusionColorReconstruction * This,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pDepthFloatFrame,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pColorFrame,
            /* [in] */ USHORT maxAlignIterationCount,
            /* [in] */ USHORT maxIntegrationWeight,
            /* [in] */ FLOAT maxColorIntegrationAngle,
            /* [out] */ FLOAT *pAlignmentEnergy,
            /* [in] */ const Matrix4 *pWorldToCameraTransform);
        
        HRESULT ( STDMETHODCALLTYPE *CalculatePointCloud )( 
            INuiFusionColorReconstruction * This,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pPointCloudFrame,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pColorFrame,
            /* [in] */ const Matrix4 *pWorldToCameraTransform);
        
        HRESULT ( STDMETHODCALLTYPE *CalculateMesh )( 
            INuiFusionColorReconstruction * This,
            /* [in] */ UINT voxelStep,
            /* [out] */ INuiFusionColorMesh **ppMesh);
        
        HRESULT ( STDMETHODCALLTYPE *ExportVolumeBlock )( 
            INuiFusionColorReconstruction * This,
            /* [in] */ UINT sourceOriginX,
            /* [in] */ UINT sourceOriginY,
            /* [in] */ UINT sourceOriginZ,
            /* [in] */ UINT destinationResolutionX,
            /* [in] */ UINT destinationResolutionY,
            /* [in] */ UINT destinationResolutionZ,
            /* [in] */ UINT voxelStep,
            /* [in] */ UINT cbVolumeBlock,
            /* [in] */ UINT cbColorVolumeBlock,
            /* [out] */ SHORT *pVolumeBlock,
            /* [out] */ INT *pColorVolumeBlock);
        
        HRESULT ( STDMETHODCALLTYPE *ImportVolumeBlock )( 
            INuiFusionColorReconstruction * This,
            /* [in] */ UINT cbVolumeBlock,
            /* [in] */ UINT cbColorVolumeBlock,
            /* [in] */ const SHORT *pVolumeBlock,
            /* [in] */ const INT *pColorVolumeBlock);
        
        HRESULT ( STDMETHODCALLTYPE *DepthToDepthFloatFrame )( 
            INuiFusionColorReconstruction * This,
            /* [in] */ const UINT16 *pDepthImageData,
            /* [in] */ UINT countDepthImageDataBytes,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pDepthFloatFrame,
            /* [in] */ FLOAT minDepthClip,
            /* [in] */ FLOAT maxDepthClip,
            /* [in] */ BOOL mirrorDepth);
        
        HRESULT ( STDMETHODCALLTYPE *SmoothDepthFloatFrame )( 
            INuiFusionColorReconstruction * This,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pDepthFloatFrame,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pSmoothDepthFloatFrame,
            /* [in] */ UINT kernelWidth,
            /* [in] */ FLOAT distanceThreshold);
        
        HRESULT ( STDMETHODCALLTYPE *AlignPointClouds )( 
            INuiFusionColorReconstruction * This,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pReferencePointCloudFrame,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pObservedPointCloudFrame,
            /* [in] */ USHORT maxAlignIterationCount,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pDeltaFromReferenceFrame,
            /* [out] */ FLOAT *pAlignmentEnergy,
            /* [out][in] */ Matrix4 *pReferenceToObservedTransform);
        
        HRESULT ( STDMETHODCALLTYPE *SetAlignDepthFloatToReconstructionReferenceFrame )( 
            INuiFusionColorReconstruction * This,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pReferenceDepthFloatFrame);
        
        HRESULT ( STDMETHODCALLTYPE *CalculatePointCloudAndDepth )( 
            INuiFusionColorReconstruction * This,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pPointCloudFrame,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pDepthFloatFrame,
            /* [in] */ const NUI_FUSION_IMAGE_FRAME *pColorFrame,
            /* [in] */ const Matrix4 *pWorldToCameraTransform);
        
        END_INTERFACE
    } INuiFusionColorReconstructionVtbl;

    interface INuiFusionColorReconstruction
    {
        CONST_VTBL struct INuiFusionColorReconstructionVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define INuiFusionColorReconstruction_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define INuiFusionColorReconstruction_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define INuiFusionColorReconstruction_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define INuiFusionColorReconstruction_ResetReconstruction(This,pInitialWorldToCameraTransform,pWorldToVolumeTransform)	\
    ( (This)->lpVtbl -> ResetReconstruction(This,pInitialWorldToCameraTransform,pWorldToVolumeTransform) ) 

#define INuiFusionColorReconstruction_AlignDepthFloatToReconstruction(This,pDepthFloatFrame,maxAlignIterationCount,pDeltaFromReferenceFrame,pAlignmentEnergy,pWorldToCameraTransform)	\
    ( (This)->lpVtbl -> AlignDepthFloatToReconstruction(This,pDepthFloatFrame,maxAlignIterationCount,pDeltaFromReferenceFrame,pAlignmentEnergy,pWorldToCameraTransform) ) 

#define INuiFusionColorReconstruction_GetCurrentWorldToCameraTransform(This,pWorldToCameraTransform)	\
    ( (This)->lpVtbl -> GetCurrentWorldToCameraTransform(This,pWorldToCameraTransform) ) 

#define INuiFusionColorReconstruction_GetCurrentWorldToVolumeTransform(This,pWorldToVolumeTransform)	\
    ( (This)->lpVtbl -> GetCurrentWorldToVolumeTransform(This,pWorldToVolumeTransform) ) 

#define INuiFusionColorReconstruction_IntegrateFrame(This,pDepthFloatFrame,pColorFrame,maxIntegrationWeight,maxColorIntegrationAngle,pWorldToCameraTransform)	\
    ( (This)->lpVtbl -> IntegrateFrame(This,pDepthFloatFrame,pColorFrame,maxIntegrationWeight,maxColorIntegrationAngle,pWorldToCameraTransform) ) 

#define INuiFusionColorReconstruction_ProcessFrame(This,pDepthFloatFrame,pColorFrame,maxAlignIterationCount,maxIntegrationWeight,maxColorIntegrationAngle,pAlignmentEnergy,pWorldToCameraTransform)	\
    ( (This)->lpVtbl -> ProcessFrame(This,pDepthFloatFrame,pColorFrame,maxAlignIterationCount,maxIntegrationWeight,maxColorIntegrationAngle,pAlignmentEnergy,pWorldToCameraTransform) ) 

#define INuiFusionColorReconstruction_CalculatePointCloud(This,pPointCloudFrame,pColorFrame,pWorldToCameraTransform)	\
    ( (This)->lpVtbl -> CalculatePointCloud(This,pPointCloudFrame,pColorFrame,pWorldToCameraTransform) ) 

#define INuiFusionColorReconstruction_CalculateMesh(This,voxelStep,ppMesh)	\
    ( (This)->lpVtbl -> CalculateMesh(This,voxelStep,ppMesh) ) 

#define INuiFusionColorReconstruction_ExportVolumeBlock(This,sourceOriginX,sourceOriginY,sourceOriginZ,destinationResolutionX,destinationResolutionY,destinationResolutionZ,voxelStep,cbVolumeBlock,cbColorVolumeBlock,pVolumeBlock,pColorVolumeBlock)	\
    ( (This)->lpVtbl -> ExportVolumeBlock(This,sourceOriginX,sourceOriginY,sourceOriginZ,destinationResolutionX,destinationResolutionY,destinationResolutionZ,voxelStep,cbVolumeBlock,cbColorVolumeBlock,pVolumeBlock,pColorVolumeBlock) ) 

#define INuiFusionColorReconstruction_ImportVolumeBlock(This,cbVolumeBlock,cbColorVolumeBlock,pVolumeBlock,pColorVolumeBlock)	\
    ( (This)->lpVtbl -> ImportVolumeBlock(This,cbVolumeBlock,cbColorVolumeBlock,pVolumeBlock,pColorVolumeBlock) ) 

#define INuiFusionColorReconstruction_DepthToDepthFloatFrame(This,pDepthImageData,countDepthImageDataBytes,pDepthFloatFrame,minDepthClip,maxDepthClip,mirrorDepth)	\
    ( (This)->lpVtbl -> DepthToDepthFloatFrame(This,pDepthImageData,countDepthImageDataBytes,pDepthFloatFrame,minDepthClip,maxDepthClip,mirrorDepth) ) 

#define INuiFusionColorReconstruction_SmoothDepthFloatFrame(This,pDepthFloatFrame,pSmoothDepthFloatFrame,kernelWidth,distanceThreshold)	\
    ( (This)->lpVtbl -> SmoothDepthFloatFrame(This,pDepthFloatFrame,pSmoothDepthFloatFrame,kernelWidth,distanceThreshold) ) 

#define INuiFusionColorReconstruction_AlignPointClouds(This,pReferencePointCloudFrame,pObservedPointCloudFrame,maxAlignIterationCount,pDeltaFromReferenceFrame,pAlignmentEnergy,pReferenceToObservedTransform)	\
    ( (This)->lpVtbl -> AlignPointClouds(This,pReferencePointCloudFrame,pObservedPointCloudFrame,maxAlignIterationCount,pDeltaFromReferenceFrame,pAlignmentEnergy,pReferenceToObservedTransform) ) 

#define INuiFusionColorReconstruction_SetAlignDepthFloatToReconstructionReferenceFrame(This,pReferenceDepthFloatFrame)	\
    ( (This)->lpVtbl -> SetAlignDepthFloatToReconstructionReferenceFrame(This,pReferenceDepthFloatFrame) ) 

#define INuiFusionColorReconstruction_CalculatePointCloudAndDepth(This,pPointCloudFrame,pDepthFloatFrame,pColorFrame,pWorldToCameraTransform)	\
    ( (This)->lpVtbl -> CalculatePointCloudAndDepth(This,pPointCloudFrame,pDepthFloatFrame,pColorFrame,pWorldToCameraTransform) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __INuiFusionColorReconstruction_INTERFACE_DEFINED__ */



/* interface __MIDL_itf_KinectFusionColor_0000_0001 */
/* [local] */ 


#include <poppack.h>

// <summary>
/// Initialize a Kinect Fusion 3D Reconstruction Volume with color capability.
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
/// <param name ="ppNuiFusionColorReconstruction">
/// A pointer that receives a reference to the created INuiFusionColorReconstruction interface.
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
///       <description>The <paramref name="ppNuiFusionColorReconstruction"/> parameter is NULL.</description>
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
_Check_return_ HRESULT FUSIONAPI NuiFusionCreateColorReconstruction (
    _In_ const NUI_FUSION_RECONSTRUCTION_PARAMETERS *pReconstructionParameters,
    _In_ NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE reconstructionProcessorType,
    _In_ INT deviceIndex,
    _In_opt_ const Matrix4 *pInitialWorldToCameraTransform,
    _Deref_out_ INuiFusionColorReconstruction **ppNuiFusionColorReconstruction
    );



extern RPC_IF_HANDLE __MIDL_itf_KinectFusionColor_0000_0001_v0_0_c_ifspec;
extern RPC_IF_HANDLE __MIDL_itf_KinectFusionColor_0000_0001_v0_0_s_ifspec;

/* Additional Prototypes for ALL interfaces */

/* end of Additional Prototypes */

#ifdef __cplusplus
}
#endif

#endif


