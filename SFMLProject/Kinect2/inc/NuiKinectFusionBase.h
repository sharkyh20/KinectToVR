

/* this ALWAYS GENERATED file contains the definitions for the interfaces */


 /* File created by MIDL compiler version 8.00.0595 */
/* at Sun Oct 19 12:58:13 2014
 */
/* Compiler settings for ..\inc_public\KinectFusionBase.idl:
    Oicf, W3, Zp8, env=Win64 (32b run), target_arch=AMD64 8.00.0595 
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


#ifndef __NuiKinectFusionBase_h__
#define __NuiKinectFusionBase_h__

#if defined(_MSC_VER) && (_MSC_VER >= 1020)
#pragma once
#endif

/* Forward Declarations */ 

/* header files for imported files */
#include "oaidl.h"
#include "ocidl.h"

#ifdef __cplusplus
extern "C"{
#endif 


/* interface __MIDL_itf_KinectFusionBase_0000_0000 */
/* [local] */ 

//----------------------------------------------------------------------------
// Copyright (c) Microsoft Corporation. All rights reserved.
//----------------------------------------------------------------------------

#ifndef FUSIONAPI
#error "You must include NuiKinectFusionApi.h rather than including NuiKinectFusionBase.h directly"
#endif

#include <pshpack8.h>

#define FACILITY_NUI 0x301
#define E_NUI_FEATURE_NOT_INITIALIZED    MAKE_HRESULT(SEVERITY_ERROR, FACILITY_NUI, 1)
#define E_NUI_DEVICE_NOT_CONNECTED    MAKE_HRESULT(SEVERITY_ERROR, FACILITY_NUI, 2)
#define E_NUI_BADINDEX    MAKE_HRESULT(SEVERITY_ERROR, FACILITY_NUI, /* 1413 */ ERROR_INVALID_INDEX)

#ifndef _Matrix4_
#define _Matrix4_
typedef struct _Matrix4
    {
    FLOAT M11;
    FLOAT M12;
    FLOAT M13;
    FLOAT M14;
    FLOAT M21;
    FLOAT M22;
    FLOAT M23;
    FLOAT M24;
    FLOAT M31;
    FLOAT M32;
    FLOAT M33;
    FLOAT M34;
    FLOAT M41;
    FLOAT M42;
    FLOAT M43;
    FLOAT M44;
    } 	Matrix4;

#endif _Matrix4_

#ifndef _NUI_FUSION_BUFFER_
#define _NUI_FUSION_BUFFER_
typedef struct _NUI_FUSION_BUFFER
    {
    UINT Pitch;
    BYTE *pBits;
    } 	NUI_FUSION_BUFFER;

#endif _NUI_FUSION_BUFFER_



extern RPC_IF_HANDLE __MIDL_itf_KinectFusionBase_0000_0000_v0_0_c_ifspec;
extern RPC_IF_HANDLE __MIDL_itf_KinectFusionBase_0000_0000_v0_0_s_ifspec;

/* Additional Prototypes for ALL interfaces */

/* end of Additional Prototypes */

#ifdef __cplusplus
}
#endif

#endif


