

/* this ALWAYS GENERATED file contains the definitions for the interfaces */


 /* File created by MIDL compiler version 8.00.0595 */
/* at Sun Oct 19 13:16:54 2014
 */
/* Compiler settings for ..\..\idl\Kinect.Face.idl:
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

#ifndef COM_NO_WINDOWS_H
#include "windows.h"
#include "ole2.h"
#endif /*COM_NO_WINDOWS_H*/

#ifndef __Kinect2EFace_h__
#define __Kinect2EFace_h__

#if defined(_MSC_VER) && (_MSC_VER >= 1020)
#pragma once
#endif

/* Forward Declarations */ 

#ifndef __IFaceFrameSource_FWD_DEFINED__
#define __IFaceFrameSource_FWD_DEFINED__
typedef interface IFaceFrameSource IFaceFrameSource;

#endif 	/* __IFaceFrameSource_FWD_DEFINED__ */


#ifndef __IFaceFrameReader_FWD_DEFINED__
#define __IFaceFrameReader_FWD_DEFINED__
typedef interface IFaceFrameReader IFaceFrameReader;

#endif 	/* __IFaceFrameReader_FWD_DEFINED__ */


#ifndef __IFaceFrameReference_FWD_DEFINED__
#define __IFaceFrameReference_FWD_DEFINED__
typedef interface IFaceFrameReference IFaceFrameReference;

#endif 	/* __IFaceFrameReference_FWD_DEFINED__ */


#ifndef __IFaceFrameArrivedEventArgs_FWD_DEFINED__
#define __IFaceFrameArrivedEventArgs_FWD_DEFINED__
typedef interface IFaceFrameArrivedEventArgs IFaceFrameArrivedEventArgs;

#endif 	/* __IFaceFrameArrivedEventArgs_FWD_DEFINED__ */


#ifndef __IFaceFrame_FWD_DEFINED__
#define __IFaceFrame_FWD_DEFINED__
typedef interface IFaceFrame IFaceFrame;

#endif 	/* __IFaceFrame_FWD_DEFINED__ */


#ifndef __IFaceFrameResult_FWD_DEFINED__
#define __IFaceFrameResult_FWD_DEFINED__
typedef interface IFaceFrameResult IFaceFrameResult;

#endif 	/* __IFaceFrameResult_FWD_DEFINED__ */


#ifndef __ITrackingIdLostEventArgs_FWD_DEFINED__
#define __ITrackingIdLostEventArgs_FWD_DEFINED__
typedef interface ITrackingIdLostEventArgs ITrackingIdLostEventArgs;

#endif 	/* __ITrackingIdLostEventArgs_FWD_DEFINED__ */


#ifndef __ICollectionStatusChangedEventArgs_FWD_DEFINED__
#define __ICollectionStatusChangedEventArgs_FWD_DEFINED__
typedef interface ICollectionStatusChangedEventArgs ICollectionStatusChangedEventArgs;

#endif 	/* __ICollectionStatusChangedEventArgs_FWD_DEFINED__ */


#ifndef __ICaptureStatusChangedEventArgs_FWD_DEFINED__
#define __ICaptureStatusChangedEventArgs_FWD_DEFINED__
typedef interface ICaptureStatusChangedEventArgs ICaptureStatusChangedEventArgs;

#endif 	/* __ICaptureStatusChangedEventArgs_FWD_DEFINED__ */


#ifndef __IFaceAlignment_FWD_DEFINED__
#define __IFaceAlignment_FWD_DEFINED__
typedef interface IFaceAlignment IFaceAlignment;

#endif 	/* __IFaceAlignment_FWD_DEFINED__ */


#ifndef __IFaceModel_FWD_DEFINED__
#define __IFaceModel_FWD_DEFINED__
typedef interface IFaceModel IFaceModel;

#endif 	/* __IFaceModel_FWD_DEFINED__ */


#ifndef __IFaceModelBuilder_FWD_DEFINED__
#define __IFaceModelBuilder_FWD_DEFINED__
typedef interface IFaceModelBuilder IFaceModelBuilder;

#endif 	/* __IFaceModelBuilder_FWD_DEFINED__ */


#ifndef __IFaceModelData_FWD_DEFINED__
#define __IFaceModelData_FWD_DEFINED__
typedef interface IFaceModelData IFaceModelData;

#endif 	/* __IFaceModelData_FWD_DEFINED__ */


#ifndef __IHighDefinitionFaceFrameReference_FWD_DEFINED__
#define __IHighDefinitionFaceFrameReference_FWD_DEFINED__
typedef interface IHighDefinitionFaceFrameReference IHighDefinitionFaceFrameReference;

#endif 	/* __IHighDefinitionFaceFrameReference_FWD_DEFINED__ */


#ifndef __IHighDefinitionFaceFrameArrivedEventArgs_FWD_DEFINED__
#define __IHighDefinitionFaceFrameArrivedEventArgs_FWD_DEFINED__
typedef interface IHighDefinitionFaceFrameArrivedEventArgs IHighDefinitionFaceFrameArrivedEventArgs;

#endif 	/* __IHighDefinitionFaceFrameArrivedEventArgs_FWD_DEFINED__ */


#ifndef __IHighDefinitionFaceFrameSource_FWD_DEFINED__
#define __IHighDefinitionFaceFrameSource_FWD_DEFINED__
typedef interface IHighDefinitionFaceFrameSource IHighDefinitionFaceFrameSource;

#endif 	/* __IHighDefinitionFaceFrameSource_FWD_DEFINED__ */


#ifndef __IHighDefinitionFaceFrameReader_FWD_DEFINED__
#define __IHighDefinitionFaceFrameReader_FWD_DEFINED__
typedef interface IHighDefinitionFaceFrameReader IHighDefinitionFaceFrameReader;

#endif 	/* __IHighDefinitionFaceFrameReader_FWD_DEFINED__ */


#ifndef __IHighDefinitionFaceFrame_FWD_DEFINED__
#define __IHighDefinitionFaceFrame_FWD_DEFINED__
typedef interface IHighDefinitionFaceFrame IHighDefinitionFaceFrame;

#endif 	/* __IHighDefinitionFaceFrame_FWD_DEFINED__ */


/* header files for imported files */
#include "oaidl.h"
#include "ocidl.h"
#include "mmreg.h"
#include "Kinect.h"

#ifdef __cplusplus
extern "C"{
#endif 


/* interface __MIDL_itf_Kinect2EFace_0000_0000 */
/* [local] */ 


#ifndef _FacePointType_
#define _FacePointType_
typedef enum _FacePointType FacePointType;


enum _FacePointType
    {
        FacePointType_None	= -1,
        FacePointType_EyeLeft	= 0,
        FacePointType_EyeRight	= 1,
        FacePointType_Nose	= 2,
        FacePointType_MouthCornerLeft	= 3,
        FacePointType_MouthCornerRight	= 4,
        FacePointType_Count	= ( FacePointType_MouthCornerRight + 1 ) 
    } ;
#endif // _FacePointType_
#ifndef _FaceFrameFeatures_
#define _FaceFrameFeatures_
typedef enum _FaceFrameFeatures FaceFrameFeatures;


enum _FaceFrameFeatures
    {
        FaceFrameFeatures_None	= 0,
        FaceFrameFeatures_BoundingBoxInInfraredSpace	= 0x1,
        FaceFrameFeatures_PointsInInfraredSpace	= 0x2,
        FaceFrameFeatures_BoundingBoxInColorSpace	= 0x4,
        FaceFrameFeatures_PointsInColorSpace	= 0x8,
        FaceFrameFeatures_RotationOrientation	= 0x10,
        FaceFrameFeatures_Happy	= 0x20,
        FaceFrameFeatures_RightEyeClosed	= 0x40,
        FaceFrameFeatures_LeftEyeClosed	= 0x80,
        FaceFrameFeatures_MouthOpen	= 0x100,
        FaceFrameFeatures_MouthMoved	= 0x200,
        FaceFrameFeatures_LookingAway	= 0x400,
        FaceFrameFeatures_Glasses	= 0x800,
        FaceFrameFeatures_FaceEngagement	= 0x1000
    } ;
#endif // _FaceFrameFeatures_
#ifndef _FaceProperty_
#define _FaceProperty_
typedef enum _FaceProperty FaceProperty;


enum _FaceProperty
    {
        FaceProperty_Happy	= 0,
        FaceProperty_Engaged	= 1,
        FaceProperty_WearingGlasses	= 2,
        FaceProperty_LeftEyeClosed	= 3,
        FaceProperty_RightEyeClosed	= 4,
        FaceProperty_MouthOpen	= 5,
        FaceProperty_MouthMoved	= 6,
        FaceProperty_LookingAway	= 7,
        FaceProperty_Count	= ( FaceProperty_LookingAway + 1 ) 
    } ;
#endif // _FaceProperty_
#ifndef _HighDetailFacePoints_
#define _HighDetailFacePoints_
typedef enum _HighDetailFacePoints HighDetailFacePoints;


enum _HighDetailFacePoints
    {
        HighDetailFacePoints_LefteyeInnercorner	= 210,
        HighDetailFacePoints_LefteyeOutercorner	= 469,
        HighDetailFacePoints_LefteyeMidtop	= 241,
        HighDetailFacePoints_LefteyeMidbottom	= 1104,
        HighDetailFacePoints_RighteyeInnercorner	= 843,
        HighDetailFacePoints_RighteyeOutercorner	= 1117,
        HighDetailFacePoints_RighteyeMidtop	= 731,
        HighDetailFacePoints_RighteyeMidbottom	= 1090,
        HighDetailFacePoints_LefteyebrowInner	= 346,
        HighDetailFacePoints_LefteyebrowOuter	= 140,
        HighDetailFacePoints_LefteyebrowCenter	= 222,
        HighDetailFacePoints_RighteyebrowInner	= 803,
        HighDetailFacePoints_RighteyebrowOuter	= 758,
        HighDetailFacePoints_RighteyebrowCenter	= 849,
        HighDetailFacePoints_MouthLeftcorner	= 91,
        HighDetailFacePoints_MouthRightcorner	= 687,
        HighDetailFacePoints_MouthUpperlipMidtop	= 19,
        HighDetailFacePoints_MouthUpperlipMidbottom	= 1072,
        HighDetailFacePoints_MouthLowerlipMidtop	= 10,
        HighDetailFacePoints_MouthLowerlipMidbottom	= 8,
        HighDetailFacePoints_NoseTip	= 18,
        HighDetailFacePoints_NoseBottom	= 14,
        HighDetailFacePoints_NoseBottomleft	= 156,
        HighDetailFacePoints_NoseBottomright	= 783,
        HighDetailFacePoints_NoseTop	= 24,
        HighDetailFacePoints_NoseTopleft	= 151,
        HighDetailFacePoints_NoseTopright	= 772,
        HighDetailFacePoints_ForeheadCenter	= 28,
        HighDetailFacePoints_LeftcheekCenter	= 412,
        HighDetailFacePoints_RightcheekCenter	= 933,
        HighDetailFacePoints_Leftcheekbone	= 458,
        HighDetailFacePoints_Rightcheekbone	= 674,
        HighDetailFacePoints_ChinCenter	= 4,
        HighDetailFacePoints_LowerjawLeftend	= 1307,
        HighDetailFacePoints_LowerjawRightend	= 1327
    } ;
#endif // _HighDetailFacePoints_
#ifndef _FaceShapeAnimations_
#define _FaceShapeAnimations_
typedef enum _FaceShapeAnimations FaceShapeAnimations;


enum _FaceShapeAnimations
    {
        FaceShapeAnimations_JawOpen	= 0,
        FaceShapeAnimations_LipPucker	= 1,
        FaceShapeAnimations_JawSlideRight	= 2,
        FaceShapeAnimations_LipStretcherRight	= 3,
        FaceShapeAnimations_LipStretcherLeft	= 4,
        FaceShapeAnimations_LipCornerPullerLeft	= 5,
        FaceShapeAnimations_LipCornerPullerRight	= 6,
        FaceShapeAnimations_LipCornerDepressorLeft	= 7,
        FaceShapeAnimations_LipCornerDepressorRight	= 8,
        FaceShapeAnimations_LeftcheekPuff	= 9,
        FaceShapeAnimations_RightcheekPuff	= 10,
        FaceShapeAnimations_LefteyeClosed	= 11,
        FaceShapeAnimations_RighteyeClosed	= 12,
        FaceShapeAnimations_RighteyebrowLowerer	= 13,
        FaceShapeAnimations_LefteyebrowLowerer	= 14,
        FaceShapeAnimations_LowerlipDepressorLeft	= 15,
        FaceShapeAnimations_LowerlipDepressorRight	= 16,
        FaceShapeAnimations_Count	= ( FaceShapeAnimations_LowerlipDepressorRight + 1 ) 
    } ;
#endif // _FaceShapeAnimations_
#ifndef _FaceShapeDeformations_
#define _FaceShapeDeformations_
typedef enum _FaceShapeDeformations FaceShapeDeformations;


enum _FaceShapeDeformations
    {
        FaceShapeDeformations_PCA01	= 0,
        FaceShapeDeformations_PCA02	= 1,
        FaceShapeDeformations_PCA03	= 2,
        FaceShapeDeformations_PCA04	= 3,
        FaceShapeDeformations_PCA05	= 4,
        FaceShapeDeformations_PCA06	= 5,
        FaceShapeDeformations_PCA07	= 6,
        FaceShapeDeformations_PCA08	= 7,
        FaceShapeDeformations_PCA09	= 8,
        FaceShapeDeformations_PCA10	= 9,
        FaceShapeDeformations_Chin03	= 10,
        FaceShapeDeformations_Forehead00	= 11,
        FaceShapeDeformations_Cheeks02	= 12,
        FaceShapeDeformations_Cheeks01	= 13,
        FaceShapeDeformations_MouthBag01	= 14,
        FaceShapeDeformations_MouthBag02	= 15,
        FaceShapeDeformations_Eyes02	= 16,
        FaceShapeDeformations_MouthBag03	= 17,
        FaceShapeDeformations_Forehead04	= 18,
        FaceShapeDeformations_Nose00	= 19,
        FaceShapeDeformations_Nose01	= 20,
        FaceShapeDeformations_Nose02	= 21,
        FaceShapeDeformations_MouthBag06	= 22,
        FaceShapeDeformations_MouthBag05	= 23,
        FaceShapeDeformations_Cheeks00	= 24,
        FaceShapeDeformations_Mask03	= 25,
        FaceShapeDeformations_Eyes03	= 26,
        FaceShapeDeformations_Nose03	= 27,
        FaceShapeDeformations_Eyes08	= 28,
        FaceShapeDeformations_MouthBag07	= 29,
        FaceShapeDeformations_Eyes00	= 30,
        FaceShapeDeformations_Nose04	= 31,
        FaceShapeDeformations_Mask04	= 32,
        FaceShapeDeformations_Chin04	= 33,
        FaceShapeDeformations_Forehead05	= 34,
        FaceShapeDeformations_Eyes06	= 35,
        FaceShapeDeformations_Eyes11	= 36,
        FaceShapeDeformations_Nose05	= 37,
        FaceShapeDeformations_Mouth07	= 38,
        FaceShapeDeformations_Cheeks08	= 39,
        FaceShapeDeformations_Eyes09	= 40,
        FaceShapeDeformations_Mask10	= 41,
        FaceShapeDeformations_Mouth09	= 42,
        FaceShapeDeformations_Nose07	= 43,
        FaceShapeDeformations_Nose08	= 44,
        FaceShapeDeformations_Cheeks07	= 45,
        FaceShapeDeformations_Mask07	= 46,
        FaceShapeDeformations_MouthBag09	= 47,
        FaceShapeDeformations_Nose06	= 48,
        FaceShapeDeformations_Chin02	= 49,
        FaceShapeDeformations_Eyes07	= 50,
        FaceShapeDeformations_Cheeks10	= 51,
        FaceShapeDeformations_Rim20	= 52,
        FaceShapeDeformations_Mask22	= 53,
        FaceShapeDeformations_MouthBag15	= 54,
        FaceShapeDeformations_Chin01	= 55,
        FaceShapeDeformations_Cheeks04	= 56,
        FaceShapeDeformations_Eyes17	= 57,
        FaceShapeDeformations_Cheeks13	= 58,
        FaceShapeDeformations_Mouth02	= 59,
        FaceShapeDeformations_MouthBag12	= 60,
        FaceShapeDeformations_Mask19	= 61,
        FaceShapeDeformations_Mask20	= 62,
        FaceShapeDeformations_Forehead06	= 63,
        FaceShapeDeformations_Mouth13	= 64,
        FaceShapeDeformations_Mask25	= 65,
        FaceShapeDeformations_Chin05	= 66,
        FaceShapeDeformations_Cheeks20	= 67,
        FaceShapeDeformations_Nose09	= 68,
        FaceShapeDeformations_Nose10	= 69,
        FaceShapeDeformations_MouthBag27	= 70,
        FaceShapeDeformations_Mouth11	= 71,
        FaceShapeDeformations_Cheeks14	= 72,
        FaceShapeDeformations_Eyes16	= 73,
        FaceShapeDeformations_Mask29	= 74,
        FaceShapeDeformations_Nose15	= 75,
        FaceShapeDeformations_Cheeks11	= 76,
        FaceShapeDeformations_Mouth16	= 77,
        FaceShapeDeformations_Eyes19	= 78,
        FaceShapeDeformations_Mouth17	= 79,
        FaceShapeDeformations_MouthBag36	= 80,
        FaceShapeDeformations_Mouth15	= 81,
        FaceShapeDeformations_Cheeks25	= 82,
        FaceShapeDeformations_Cheeks16	= 83,
        FaceShapeDeformations_Cheeks18	= 84,
        FaceShapeDeformations_Rim07	= 85,
        FaceShapeDeformations_Nose13	= 86,
        FaceShapeDeformations_Mouth18	= 87,
        FaceShapeDeformations_Cheeks19	= 88,
        FaceShapeDeformations_Rim21	= 89,
        FaceShapeDeformations_Mouth22	= 90,
        FaceShapeDeformations_Nose18	= 91,
        FaceShapeDeformations_Nose16	= 92,
        FaceShapeDeformations_Rim22	= 93,
        FaceShapeDeformations_Count	= ( FaceShapeDeformations_Rim22 + 1 ) 
    } ;
#endif // _FaceShapeDeformations_
#ifndef _FaceAlignmentQuality_
#define _FaceAlignmentQuality_
typedef enum _FaceAlignmentQuality FaceAlignmentQuality;


enum _FaceAlignmentQuality
    {
        FaceAlignmentQuality_High	= 0,
        FaceAlignmentQuality_Low	= 1
    } ;
#endif // _FaceAlignmentQuality_
#ifndef _FaceModelBuilderCollectionStatus_
#define _FaceModelBuilderCollectionStatus_
typedef enum _FaceModelBuilderCollectionStatus FaceModelBuilderCollectionStatus;


enum _FaceModelBuilderCollectionStatus
    {
        FaceModelBuilderCollectionStatus_Complete	= 0,
        FaceModelBuilderCollectionStatus_MoreFramesNeeded	= 0x1,
        FaceModelBuilderCollectionStatus_FrontViewFramesNeeded	= 0x2,
        FaceModelBuilderCollectionStatus_LeftViewsNeeded	= 0x4,
        FaceModelBuilderCollectionStatus_RightViewsNeeded	= 0x8,
        FaceModelBuilderCollectionStatus_TiltedUpViewsNeeded	= 0x10
    } ;
#endif // _FaceModelBuilderCollectionStatus_
#ifndef _FaceModelBuilderCaptureStatus_
#define _FaceModelBuilderCaptureStatus_
typedef enum _FaceModelBuilderCaptureStatus FaceModelBuilderCaptureStatus;


enum _FaceModelBuilderCaptureStatus
    {
        FaceModelBuilderCaptureStatus_GoodFrameCapture	= 0,
        FaceModelBuilderCaptureStatus_OtherViewsNeeded	= 1,
        FaceModelBuilderCaptureStatus_LostFaceTrack	= 2,
        FaceModelBuilderCaptureStatus_FaceTooFar	= 3,
        FaceModelBuilderCaptureStatus_FaceTooNear	= 4,
        FaceModelBuilderCaptureStatus_MovingTooFast	= 5,
        FaceModelBuilderCaptureStatus_SystemError	= 6
    } ;
#endif // _FaceModelBuilderCaptureStatus_
#ifndef _FaceModelBuilderAttributes_
#define _FaceModelBuilderAttributes_
typedef enum _FaceModelBuilderAttributes FaceModelBuilderAttributes;


enum _FaceModelBuilderAttributes
    {
        FaceModelBuilderAttributes_None	= 0,
        FaceModelBuilderAttributes_SkinColor	= 0x1,
        FaceModelBuilderAttributes_HairColor	= 0x2
    } ;
#endif // _FaceModelBuilderAttributes_
#ifndef _RectI_
#define _RectI_
typedef struct _RectI
    {
    INT32 Left;
    INT32 Top;
    INT32 Right;
    INT32 Bottom;
    } 	RectI;

#endif // _RectI_





















extern RPC_IF_HANDLE __MIDL_itf_Kinect2EFace_0000_0000_v0_0_c_ifspec;
extern RPC_IF_HANDLE __MIDL_itf_Kinect2EFace_0000_0000_v0_0_s_ifspec;

#ifndef __IFaceFrameSource_INTERFACE_DEFINED__
#define __IFaceFrameSource_INTERFACE_DEFINED__

/* interface IFaceFrameSource */
/* [object][local][uuid] */ 


EXTERN_C const IID IID_IFaceFrameSource;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("3A348391-43AD-F7D8-437D-D790FA5833DE")
    IFaceFrameSource : public IUnknown
    {
    public:
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_IsActive( 
            /* [annotation][out][retval] */ 
            _Out_  BOOLEAN *isActive) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE OpenReader( 
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IFaceFrameReader **reader) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_KinectSensor( 
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IKinectSensor **sensor) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE SubscribeTrackingIdLost( 
            /* [annotation][out][retval] */ 
            _Out_  WAITABLE_HANDLE *waitableHandle) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE UnsubscribeTrackingIdLost( 
            /* [annotation][in] */ 
            _In_  WAITABLE_HANDLE waitableHandle) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE GetTrackingIdLostEventData( 
            /* [annotation][in] */ 
            _In_  WAITABLE_HANDLE waitableHandle,
            /* [annotation][out][retval] */ 
            _Out_  ITrackingIdLostEventArgs **eventData) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_TrackingId( 
            /* [annotation][out][retval] */ 
            _Out_  UINT64 *trackingId) = 0;
        
        virtual /* [propput] */ HRESULT STDMETHODCALLTYPE put_TrackingId( 
            /* [annotation][in] */ 
            _In_  const UINT64 trackingId) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_IsTrackingIdValid( 
            /* [annotation][out][retval] */ 
            _Out_  BOOLEAN *isTrackingIdValid) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_FaceFrameFeatures( 
            /* [annotation][out][retval] */ 
            _Out_  DWORD *faceFrameFeatures) = 0;
        
        virtual /* [propput] */ HRESULT STDMETHODCALLTYPE put_FaceFrameFeatures( 
            /* [annotation][in] */ 
            _In_  const DWORD faceFrameFeatures) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct IFaceFrameSourceVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            IFaceFrameSource * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            IFaceFrameSource * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            IFaceFrameSource * This);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_IsActive )( 
            IFaceFrameSource * This,
            /* [annotation][out][retval] */ 
            _Out_  BOOLEAN *isActive);
        
        HRESULT ( STDMETHODCALLTYPE *OpenReader )( 
            IFaceFrameSource * This,
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IFaceFrameReader **reader);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_KinectSensor )( 
            IFaceFrameSource * This,
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IKinectSensor **sensor);
        
        HRESULT ( STDMETHODCALLTYPE *SubscribeTrackingIdLost )( 
            IFaceFrameSource * This,
            /* [annotation][out][retval] */ 
            _Out_  WAITABLE_HANDLE *waitableHandle);
        
        HRESULT ( STDMETHODCALLTYPE *UnsubscribeTrackingIdLost )( 
            IFaceFrameSource * This,
            /* [annotation][in] */ 
            _In_  WAITABLE_HANDLE waitableHandle);
        
        HRESULT ( STDMETHODCALLTYPE *GetTrackingIdLostEventData )( 
            IFaceFrameSource * This,
            /* [annotation][in] */ 
            _In_  WAITABLE_HANDLE waitableHandle,
            /* [annotation][out][retval] */ 
            _Out_  ITrackingIdLostEventArgs **eventData);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_TrackingId )( 
            IFaceFrameSource * This,
            /* [annotation][out][retval] */ 
            _Out_  UINT64 *trackingId);
        
        /* [propput] */ HRESULT ( STDMETHODCALLTYPE *put_TrackingId )( 
            IFaceFrameSource * This,
            /* [annotation][in] */ 
            _In_  const UINT64 trackingId);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_IsTrackingIdValid )( 
            IFaceFrameSource * This,
            /* [annotation][out][retval] */ 
            _Out_  BOOLEAN *isTrackingIdValid);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_FaceFrameFeatures )( 
            IFaceFrameSource * This,
            /* [annotation][out][retval] */ 
            _Out_  DWORD *faceFrameFeatures);
        
        /* [propput] */ HRESULT ( STDMETHODCALLTYPE *put_FaceFrameFeatures )( 
            IFaceFrameSource * This,
            /* [annotation][in] */ 
            _In_  const DWORD faceFrameFeatures);
        
        END_INTERFACE
    } IFaceFrameSourceVtbl;

    interface IFaceFrameSource
    {
        CONST_VTBL struct IFaceFrameSourceVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define IFaceFrameSource_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define IFaceFrameSource_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define IFaceFrameSource_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define IFaceFrameSource_get_IsActive(This,isActive)	\
    ( (This)->lpVtbl -> get_IsActive(This,isActive) ) 

#define IFaceFrameSource_OpenReader(This,reader)	\
    ( (This)->lpVtbl -> OpenReader(This,reader) ) 

#define IFaceFrameSource_get_KinectSensor(This,sensor)	\
    ( (This)->lpVtbl -> get_KinectSensor(This,sensor) ) 

#define IFaceFrameSource_SubscribeTrackingIdLost(This,waitableHandle)	\
    ( (This)->lpVtbl -> SubscribeTrackingIdLost(This,waitableHandle) ) 

#define IFaceFrameSource_UnsubscribeTrackingIdLost(This,waitableHandle)	\
    ( (This)->lpVtbl -> UnsubscribeTrackingIdLost(This,waitableHandle) ) 

#define IFaceFrameSource_GetTrackingIdLostEventData(This,waitableHandle,eventData)	\
    ( (This)->lpVtbl -> GetTrackingIdLostEventData(This,waitableHandle,eventData) ) 

#define IFaceFrameSource_get_TrackingId(This,trackingId)	\
    ( (This)->lpVtbl -> get_TrackingId(This,trackingId) ) 

#define IFaceFrameSource_put_TrackingId(This,trackingId)	\
    ( (This)->lpVtbl -> put_TrackingId(This,trackingId) ) 

#define IFaceFrameSource_get_IsTrackingIdValid(This,isTrackingIdValid)	\
    ( (This)->lpVtbl -> get_IsTrackingIdValid(This,isTrackingIdValid) ) 

#define IFaceFrameSource_get_FaceFrameFeatures(This,faceFrameFeatures)	\
    ( (This)->lpVtbl -> get_FaceFrameFeatures(This,faceFrameFeatures) ) 

#define IFaceFrameSource_put_FaceFrameFeatures(This,faceFrameFeatures)	\
    ( (This)->lpVtbl -> put_FaceFrameFeatures(This,faceFrameFeatures) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __IFaceFrameSource_INTERFACE_DEFINED__ */


#ifndef __IFaceFrameReader_INTERFACE_DEFINED__
#define __IFaceFrameReader_INTERFACE_DEFINED__

/* interface IFaceFrameReader */
/* [object][local][uuid] */ 


EXTERN_C const IID IID_IFaceFrameReader;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("AD0D735F-4E42-82BB-5C03-1BBE5E24CB19")
    IFaceFrameReader : public IUnknown
    {
    public:
        virtual HRESULT STDMETHODCALLTYPE SubscribeFrameArrived( 
            /* [annotation][out][retval] */ 
            _Out_  WAITABLE_HANDLE *waitableHandle) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE UnsubscribeFrameArrived( 
            /* [annotation][in] */ 
            _In_  WAITABLE_HANDLE waitableHandle) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE GetFrameArrivedEventData( 
            /* [annotation][in] */ 
            _In_  WAITABLE_HANDLE waitableHandle,
            /* [annotation][out][retval] */ 
            _Out_  IFaceFrameArrivedEventArgs **eventData) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE AcquireLatestFrame( 
            /* [annotation][out][retval] */ 
            _COM_Outptr_result_maybenull_  IFaceFrame **faceFrame) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_IsPaused( 
            /* [annotation][out][retval] */ 
            _Out_  BOOLEAN *isPaused) = 0;
        
        virtual /* [propput] */ HRESULT STDMETHODCALLTYPE put_IsPaused( 
            BOOLEAN isPaused) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_FaceFrameSource( 
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IFaceFrameSource **faceFrameSource) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct IFaceFrameReaderVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            IFaceFrameReader * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            IFaceFrameReader * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            IFaceFrameReader * This);
        
        HRESULT ( STDMETHODCALLTYPE *SubscribeFrameArrived )( 
            IFaceFrameReader * This,
            /* [annotation][out][retval] */ 
            _Out_  WAITABLE_HANDLE *waitableHandle);
        
        HRESULT ( STDMETHODCALLTYPE *UnsubscribeFrameArrived )( 
            IFaceFrameReader * This,
            /* [annotation][in] */ 
            _In_  WAITABLE_HANDLE waitableHandle);
        
        HRESULT ( STDMETHODCALLTYPE *GetFrameArrivedEventData )( 
            IFaceFrameReader * This,
            /* [annotation][in] */ 
            _In_  WAITABLE_HANDLE waitableHandle,
            /* [annotation][out][retval] */ 
            _Out_  IFaceFrameArrivedEventArgs **eventData);
        
        HRESULT ( STDMETHODCALLTYPE *AcquireLatestFrame )( 
            IFaceFrameReader * This,
            /* [annotation][out][retval] */ 
            _COM_Outptr_result_maybenull_  IFaceFrame **faceFrame);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_IsPaused )( 
            IFaceFrameReader * This,
            /* [annotation][out][retval] */ 
            _Out_  BOOLEAN *isPaused);
        
        /* [propput] */ HRESULT ( STDMETHODCALLTYPE *put_IsPaused )( 
            IFaceFrameReader * This,
            BOOLEAN isPaused);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_FaceFrameSource )( 
            IFaceFrameReader * This,
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IFaceFrameSource **faceFrameSource);
        
        END_INTERFACE
    } IFaceFrameReaderVtbl;

    interface IFaceFrameReader
    {
        CONST_VTBL struct IFaceFrameReaderVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define IFaceFrameReader_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define IFaceFrameReader_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define IFaceFrameReader_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define IFaceFrameReader_SubscribeFrameArrived(This,waitableHandle)	\
    ( (This)->lpVtbl -> SubscribeFrameArrived(This,waitableHandle) ) 

#define IFaceFrameReader_UnsubscribeFrameArrived(This,waitableHandle)	\
    ( (This)->lpVtbl -> UnsubscribeFrameArrived(This,waitableHandle) ) 

#define IFaceFrameReader_GetFrameArrivedEventData(This,waitableHandle,eventData)	\
    ( (This)->lpVtbl -> GetFrameArrivedEventData(This,waitableHandle,eventData) ) 

#define IFaceFrameReader_AcquireLatestFrame(This,faceFrame)	\
    ( (This)->lpVtbl -> AcquireLatestFrame(This,faceFrame) ) 

#define IFaceFrameReader_get_IsPaused(This,isPaused)	\
    ( (This)->lpVtbl -> get_IsPaused(This,isPaused) ) 

#define IFaceFrameReader_put_IsPaused(This,isPaused)	\
    ( (This)->lpVtbl -> put_IsPaused(This,isPaused) ) 

#define IFaceFrameReader_get_FaceFrameSource(This,faceFrameSource)	\
    ( (This)->lpVtbl -> get_FaceFrameSource(This,faceFrameSource) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __IFaceFrameReader_INTERFACE_DEFINED__ */


#ifndef __IFaceFrameReference_INTERFACE_DEFINED__
#define __IFaceFrameReference_INTERFACE_DEFINED__

/* interface IFaceFrameReference */
/* [object][local][uuid] */ 


EXTERN_C const IID IID_IFaceFrameReference;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("78E06811-4C53-A4BD-4CE7-8C995CAD882F")
    IFaceFrameReference : public IUnknown
    {
    public:
        virtual HRESULT STDMETHODCALLTYPE AcquireFrame( 
            /* [annotation][out][retval] */ 
            _COM_Outptr_result_maybenull_  IFaceFrame **faceFrame) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_RelativeTime( 
            /* [annotation][out][retval] */ 
            _Out_  TIMESPAN *relativeTime) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct IFaceFrameReferenceVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            IFaceFrameReference * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            IFaceFrameReference * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            IFaceFrameReference * This);
        
        HRESULT ( STDMETHODCALLTYPE *AcquireFrame )( 
            IFaceFrameReference * This,
            /* [annotation][out][retval] */ 
            _COM_Outptr_result_maybenull_  IFaceFrame **faceFrame);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_RelativeTime )( 
            IFaceFrameReference * This,
            /* [annotation][out][retval] */ 
            _Out_  TIMESPAN *relativeTime);
        
        END_INTERFACE
    } IFaceFrameReferenceVtbl;

    interface IFaceFrameReference
    {
        CONST_VTBL struct IFaceFrameReferenceVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define IFaceFrameReference_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define IFaceFrameReference_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define IFaceFrameReference_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define IFaceFrameReference_AcquireFrame(This,faceFrame)	\
    ( (This)->lpVtbl -> AcquireFrame(This,faceFrame) ) 

#define IFaceFrameReference_get_RelativeTime(This,relativeTime)	\
    ( (This)->lpVtbl -> get_RelativeTime(This,relativeTime) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __IFaceFrameReference_INTERFACE_DEFINED__ */


#ifndef __IFaceFrameArrivedEventArgs_INTERFACE_DEFINED__
#define __IFaceFrameArrivedEventArgs_INTERFACE_DEFINED__

/* interface IFaceFrameArrivedEventArgs */
/* [object][local][uuid] */ 


EXTERN_C const IID IID_IFaceFrameArrivedEventArgs;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("C270AC82-4C03-3BC6-F13E-E680AE7964DA")
    IFaceFrameArrivedEventArgs : public IUnknown
    {
    public:
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_FrameReference( 
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IFaceFrameReference **faceFrameReference) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct IFaceFrameArrivedEventArgsVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            IFaceFrameArrivedEventArgs * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            IFaceFrameArrivedEventArgs * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            IFaceFrameArrivedEventArgs * This);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_FrameReference )( 
            IFaceFrameArrivedEventArgs * This,
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IFaceFrameReference **faceFrameReference);
        
        END_INTERFACE
    } IFaceFrameArrivedEventArgsVtbl;

    interface IFaceFrameArrivedEventArgs
    {
        CONST_VTBL struct IFaceFrameArrivedEventArgsVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define IFaceFrameArrivedEventArgs_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define IFaceFrameArrivedEventArgs_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define IFaceFrameArrivedEventArgs_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define IFaceFrameArrivedEventArgs_get_FrameReference(This,faceFrameReference)	\
    ( (This)->lpVtbl -> get_FrameReference(This,faceFrameReference) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __IFaceFrameArrivedEventArgs_INTERFACE_DEFINED__ */


#ifndef __IFaceFrame_INTERFACE_DEFINED__
#define __IFaceFrame_INTERFACE_DEFINED__

/* interface IFaceFrame */
/* [object][local][uuid] */ 


EXTERN_C const IID IID_IFaceFrame;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("5A59B4D1-489E-7211-9676-44AF6E22E247")
    IFaceFrame : public IUnknown
    {
    public:
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_FaceFrameResult( 
            /* [annotation][out][retval] */ 
            _COM_Outptr_result_maybenull_  IFaceFrameResult **faceFrameResult) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_TrackingId( 
            /* [annotation][out][retval] */ 
            _Out_  UINT64 *trackingId) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_IsTrackingIdValid( 
            /* [annotation][out][retval] */ 
            _Out_  BOOLEAN *isTrackingIdValid) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_RelativeTime( 
            /* [annotation][out][retval] */ 
            _Out_  TIMESPAN *relativeTime) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_FaceFrameSource( 
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IFaceFrameSource **faceFrameSource) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_BodyFrameReference( 
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IBodyFrameReference **bodyFrameReference) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_ColorFrameReference( 
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IColorFrameReference **colorFrameReference) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_DepthFrameReference( 
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IDepthFrameReference **depthFrameReference) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_InfraredFrameReference( 
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IInfraredFrameReference **infraredFrameReference) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct IFaceFrameVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            IFaceFrame * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            IFaceFrame * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            IFaceFrame * This);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_FaceFrameResult )( 
            IFaceFrame * This,
            /* [annotation][out][retval] */ 
            _COM_Outptr_result_maybenull_  IFaceFrameResult **faceFrameResult);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_TrackingId )( 
            IFaceFrame * This,
            /* [annotation][out][retval] */ 
            _Out_  UINT64 *trackingId);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_IsTrackingIdValid )( 
            IFaceFrame * This,
            /* [annotation][out][retval] */ 
            _Out_  BOOLEAN *isTrackingIdValid);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_RelativeTime )( 
            IFaceFrame * This,
            /* [annotation][out][retval] */ 
            _Out_  TIMESPAN *relativeTime);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_FaceFrameSource )( 
            IFaceFrame * This,
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IFaceFrameSource **faceFrameSource);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_BodyFrameReference )( 
            IFaceFrame * This,
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IBodyFrameReference **bodyFrameReference);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_ColorFrameReference )( 
            IFaceFrame * This,
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IColorFrameReference **colorFrameReference);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_DepthFrameReference )( 
            IFaceFrame * This,
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IDepthFrameReference **depthFrameReference);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_InfraredFrameReference )( 
            IFaceFrame * This,
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IInfraredFrameReference **infraredFrameReference);
        
        END_INTERFACE
    } IFaceFrameVtbl;

    interface IFaceFrame
    {
        CONST_VTBL struct IFaceFrameVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define IFaceFrame_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define IFaceFrame_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define IFaceFrame_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define IFaceFrame_get_FaceFrameResult(This,faceFrameResult)	\
    ( (This)->lpVtbl -> get_FaceFrameResult(This,faceFrameResult) ) 

#define IFaceFrame_get_TrackingId(This,trackingId)	\
    ( (This)->lpVtbl -> get_TrackingId(This,trackingId) ) 

#define IFaceFrame_get_IsTrackingIdValid(This,isTrackingIdValid)	\
    ( (This)->lpVtbl -> get_IsTrackingIdValid(This,isTrackingIdValid) ) 

#define IFaceFrame_get_RelativeTime(This,relativeTime)	\
    ( (This)->lpVtbl -> get_RelativeTime(This,relativeTime) ) 

#define IFaceFrame_get_FaceFrameSource(This,faceFrameSource)	\
    ( (This)->lpVtbl -> get_FaceFrameSource(This,faceFrameSource) ) 

#define IFaceFrame_get_BodyFrameReference(This,bodyFrameReference)	\
    ( (This)->lpVtbl -> get_BodyFrameReference(This,bodyFrameReference) ) 

#define IFaceFrame_get_ColorFrameReference(This,colorFrameReference)	\
    ( (This)->lpVtbl -> get_ColorFrameReference(This,colorFrameReference) ) 

#define IFaceFrame_get_DepthFrameReference(This,depthFrameReference)	\
    ( (This)->lpVtbl -> get_DepthFrameReference(This,depthFrameReference) ) 

#define IFaceFrame_get_InfraredFrameReference(This,infraredFrameReference)	\
    ( (This)->lpVtbl -> get_InfraredFrameReference(This,infraredFrameReference) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __IFaceFrame_INTERFACE_DEFINED__ */


#ifndef __IFaceFrameResult_INTERFACE_DEFINED__
#define __IFaceFrameResult_INTERFACE_DEFINED__

/* interface IFaceFrameResult */
/* [object][local][uuid] */ 


EXTERN_C const IID IID_IFaceFrameResult;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("8873C3F1-445F-BD37-404C-ED92C2906523")
    IFaceFrameResult : public IUnknown
    {
    public:
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_RelativeTime( 
            /* [annotation][out][retval] */ 
            _Out_  TIMESPAN *relativeTime) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_FaceFrameFeatures( 
            /* [annotation][out][retval] */ 
            _Out_  DWORD *faceFrameFeatures) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_TrackingId( 
            /* [annotation][out][retval] */ 
            _Out_  UINT64 *trackingId) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_FaceRotationQuaternion( 
            /* [annotation][out][retval] */ 
            _Out_  Vector4 *rotationQuaternion) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_FaceBoundingBoxInColorSpace( 
            /* [annotation][out][retval] */ 
            _Out_  RectI *boundingBox) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_FaceBoundingBoxInInfraredSpace( 
            /* [annotation][out][retval] */ 
            _Out_  RectI *boundingBox) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE GetFacePointsInColorSpace( 
            /* [annotation][in][annotation] */ 
            _In_  const UINT capacity,
            /* [annotation][size_is][out] */ 
            _Out_writes_all_(capacity)  PointF *facePoints) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE GetFacePointsInInfraredSpace( 
            /* [annotation][in][annotation] */ 
            _In_  const UINT capacity,
            /* [annotation][size_is][out] */ 
            _Out_writes_all_(capacity)  PointF *facePoints) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE GetFaceProperties( 
            /* [annotation][in][annotation] */ 
            _In_  const UINT capacity,
            /* [annotation][size_is][out] */ 
            _Out_writes_all_(capacity)  DetectionResult *detectionResults) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct IFaceFrameResultVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            IFaceFrameResult * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            IFaceFrameResult * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            IFaceFrameResult * This);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_RelativeTime )( 
            IFaceFrameResult * This,
            /* [annotation][out][retval] */ 
            _Out_  TIMESPAN *relativeTime);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_FaceFrameFeatures )( 
            IFaceFrameResult * This,
            /* [annotation][out][retval] */ 
            _Out_  DWORD *faceFrameFeatures);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_TrackingId )( 
            IFaceFrameResult * This,
            /* [annotation][out][retval] */ 
            _Out_  UINT64 *trackingId);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_FaceRotationQuaternion )( 
            IFaceFrameResult * This,
            /* [annotation][out][retval] */ 
            _Out_  Vector4 *rotationQuaternion);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_FaceBoundingBoxInColorSpace )( 
            IFaceFrameResult * This,
            /* [annotation][out][retval] */ 
            _Out_  RectI *boundingBox);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_FaceBoundingBoxInInfraredSpace )( 
            IFaceFrameResult * This,
            /* [annotation][out][retval] */ 
            _Out_  RectI *boundingBox);
        
        HRESULT ( STDMETHODCALLTYPE *GetFacePointsInColorSpace )( 
            IFaceFrameResult * This,
            /* [annotation][in][annotation] */ 
            _In_  const UINT capacity,
            /* [annotation][size_is][out] */ 
            _Out_writes_all_(capacity)  PointF *facePoints);
        
        HRESULT ( STDMETHODCALLTYPE *GetFacePointsInInfraredSpace )( 
            IFaceFrameResult * This,
            /* [annotation][in][annotation] */ 
            _In_  const UINT capacity,
            /* [annotation][size_is][out] */ 
            _Out_writes_all_(capacity)  PointF *facePoints);
        
        HRESULT ( STDMETHODCALLTYPE *GetFaceProperties )( 
            IFaceFrameResult * This,
            /* [annotation][in][annotation] */ 
            _In_  const UINT capacity,
            /* [annotation][size_is][out] */ 
            _Out_writes_all_(capacity)  DetectionResult *detectionResults);
        
        END_INTERFACE
    } IFaceFrameResultVtbl;

    interface IFaceFrameResult
    {
        CONST_VTBL struct IFaceFrameResultVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define IFaceFrameResult_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define IFaceFrameResult_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define IFaceFrameResult_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define IFaceFrameResult_get_RelativeTime(This,relativeTime)	\
    ( (This)->lpVtbl -> get_RelativeTime(This,relativeTime) ) 

#define IFaceFrameResult_get_FaceFrameFeatures(This,faceFrameFeatures)	\
    ( (This)->lpVtbl -> get_FaceFrameFeatures(This,faceFrameFeatures) ) 

#define IFaceFrameResult_get_TrackingId(This,trackingId)	\
    ( (This)->lpVtbl -> get_TrackingId(This,trackingId) ) 

#define IFaceFrameResult_get_FaceRotationQuaternion(This,rotationQuaternion)	\
    ( (This)->lpVtbl -> get_FaceRotationQuaternion(This,rotationQuaternion) ) 

#define IFaceFrameResult_get_FaceBoundingBoxInColorSpace(This,boundingBox)	\
    ( (This)->lpVtbl -> get_FaceBoundingBoxInColorSpace(This,boundingBox) ) 

#define IFaceFrameResult_get_FaceBoundingBoxInInfraredSpace(This,boundingBox)	\
    ( (This)->lpVtbl -> get_FaceBoundingBoxInInfraredSpace(This,boundingBox) ) 

#define IFaceFrameResult_GetFacePointsInColorSpace(This,capacity,facePoints)	\
    ( (This)->lpVtbl -> GetFacePointsInColorSpace(This,capacity,facePoints) ) 

#define IFaceFrameResult_GetFacePointsInInfraredSpace(This,capacity,facePoints)	\
    ( (This)->lpVtbl -> GetFacePointsInInfraredSpace(This,capacity,facePoints) ) 

#define IFaceFrameResult_GetFaceProperties(This,capacity,detectionResults)	\
    ( (This)->lpVtbl -> GetFaceProperties(This,capacity,detectionResults) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __IFaceFrameResult_INTERFACE_DEFINED__ */


#ifndef __ITrackingIdLostEventArgs_INTERFACE_DEFINED__
#define __ITrackingIdLostEventArgs_INTERFACE_DEFINED__

/* interface ITrackingIdLostEventArgs */
/* [object][local][uuid] */ 


EXTERN_C const IID IID_ITrackingIdLostEventArgs;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("1EFE99CA-4356-FED8-D564-BDA0CB6F15C4")
    ITrackingIdLostEventArgs : public IUnknown
    {
    public:
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_TrackingId( 
            /* [annotation][out][retval] */ 
            _Out_  UINT64 *trackingId) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct ITrackingIdLostEventArgsVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            ITrackingIdLostEventArgs * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            ITrackingIdLostEventArgs * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            ITrackingIdLostEventArgs * This);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_TrackingId )( 
            ITrackingIdLostEventArgs * This,
            /* [annotation][out][retval] */ 
            _Out_  UINT64 *trackingId);
        
        END_INTERFACE
    } ITrackingIdLostEventArgsVtbl;

    interface ITrackingIdLostEventArgs
    {
        CONST_VTBL struct ITrackingIdLostEventArgsVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define ITrackingIdLostEventArgs_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define ITrackingIdLostEventArgs_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define ITrackingIdLostEventArgs_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define ITrackingIdLostEventArgs_get_TrackingId(This,trackingId)	\
    ( (This)->lpVtbl -> get_TrackingId(This,trackingId) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __ITrackingIdLostEventArgs_INTERFACE_DEFINED__ */


#ifndef __ICollectionStatusChangedEventArgs_INTERFACE_DEFINED__
#define __ICollectionStatusChangedEventArgs_INTERFACE_DEFINED__

/* interface ICollectionStatusChangedEventArgs */
/* [object][local][uuid] */ 


EXTERN_C const IID IID_ICollectionStatusChangedEventArgs;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("B48D57AD-A414-4542-BE89-C4898816A678")
    ICollectionStatusChangedEventArgs : public IUnknown
    {
    public:
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_PreviousCollectionStatus( 
            /* [annotation][out][retval] */ 
            _Out_  FaceModelBuilderCollectionStatus *pCollectionStatus) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct ICollectionStatusChangedEventArgsVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            ICollectionStatusChangedEventArgs * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            ICollectionStatusChangedEventArgs * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            ICollectionStatusChangedEventArgs * This);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_PreviousCollectionStatus )( 
            ICollectionStatusChangedEventArgs * This,
            /* [annotation][out][retval] */ 
            _Out_  FaceModelBuilderCollectionStatus *pCollectionStatus);
        
        END_INTERFACE
    } ICollectionStatusChangedEventArgsVtbl;

    interface ICollectionStatusChangedEventArgs
    {
        CONST_VTBL struct ICollectionStatusChangedEventArgsVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define ICollectionStatusChangedEventArgs_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define ICollectionStatusChangedEventArgs_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define ICollectionStatusChangedEventArgs_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define ICollectionStatusChangedEventArgs_get_PreviousCollectionStatus(This,pCollectionStatus)	\
    ( (This)->lpVtbl -> get_PreviousCollectionStatus(This,pCollectionStatus) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __ICollectionStatusChangedEventArgs_INTERFACE_DEFINED__ */


#ifndef __ICaptureStatusChangedEventArgs_INTERFACE_DEFINED__
#define __ICaptureStatusChangedEventArgs_INTERFACE_DEFINED__

/* interface ICaptureStatusChangedEventArgs */
/* [object][local][uuid] */ 


EXTERN_C const IID IID_ICaptureStatusChangedEventArgs;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("BD8BA0B8-6CD5-4051-9246-01A15D023BA9")
    ICaptureStatusChangedEventArgs : public IUnknown
    {
    public:
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_PreviousCaptureStatus( 
            /* [annotation][out][retval] */ 
            _Out_  FaceModelBuilderCaptureStatus *pCaptureStatus) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct ICaptureStatusChangedEventArgsVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            ICaptureStatusChangedEventArgs * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            ICaptureStatusChangedEventArgs * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            ICaptureStatusChangedEventArgs * This);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_PreviousCaptureStatus )( 
            ICaptureStatusChangedEventArgs * This,
            /* [annotation][out][retval] */ 
            _Out_  FaceModelBuilderCaptureStatus *pCaptureStatus);
        
        END_INTERFACE
    } ICaptureStatusChangedEventArgsVtbl;

    interface ICaptureStatusChangedEventArgs
    {
        CONST_VTBL struct ICaptureStatusChangedEventArgsVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define ICaptureStatusChangedEventArgs_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define ICaptureStatusChangedEventArgs_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define ICaptureStatusChangedEventArgs_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define ICaptureStatusChangedEventArgs_get_PreviousCaptureStatus(This,pCaptureStatus)	\
    ( (This)->lpVtbl -> get_PreviousCaptureStatus(This,pCaptureStatus) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __ICaptureStatusChangedEventArgs_INTERFACE_DEFINED__ */


#ifndef __IFaceAlignment_INTERFACE_DEFINED__
#define __IFaceAlignment_INTERFACE_DEFINED__

/* interface IFaceAlignment */
/* [object][local][uuid] */ 


EXTERN_C const IID IID_IFaceAlignment;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("38DFFDE8-2C4D-476D-B1DF-43EF4529F2AF")
    IFaceAlignment : public IUnknown
    {
    public:
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_FaceOrientation( 
            /* [annotation][out][retval] */ 
            _Out_  Vector4 *faceOrientation) = 0;
        
        virtual /* [propput] */ HRESULT STDMETHODCALLTYPE put_FaceOrientation( 
            Vector4 faceOrientation) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_HeadPivotPoint( 
            /* [annotation][out][retval] */ 
            _Out_  CameraSpacePoint *headPivotPoint) = 0;
        
        virtual /* [propput] */ HRESULT STDMETHODCALLTYPE put_HeadPivotPoint( 
            CameraSpacePoint headPivotPoint) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_FaceBoundingBox( 
            /* [annotation][out][retval] */ 
            _Out_  RectI *boundingBox) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE GetAnimationUnits( 
            UINT capacity,
            /* [annotation][size_is][out] */ 
            _Out_writes_all_(capacity)  float *animationUnits) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_Quality( 
            /* [annotation][out][retval] */ 
            _Out_  FaceAlignmentQuality *quality) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct IFaceAlignmentVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            IFaceAlignment * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            IFaceAlignment * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            IFaceAlignment * This);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_FaceOrientation )( 
            IFaceAlignment * This,
            /* [annotation][out][retval] */ 
            _Out_  Vector4 *faceOrientation);
        
        /* [propput] */ HRESULT ( STDMETHODCALLTYPE *put_FaceOrientation )( 
            IFaceAlignment * This,
            Vector4 faceOrientation);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_HeadPivotPoint )( 
            IFaceAlignment * This,
            /* [annotation][out][retval] */ 
            _Out_  CameraSpacePoint *headPivotPoint);
        
        /* [propput] */ HRESULT ( STDMETHODCALLTYPE *put_HeadPivotPoint )( 
            IFaceAlignment * This,
            CameraSpacePoint headPivotPoint);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_FaceBoundingBox )( 
            IFaceAlignment * This,
            /* [annotation][out][retval] */ 
            _Out_  RectI *boundingBox);
        
        HRESULT ( STDMETHODCALLTYPE *GetAnimationUnits )( 
            IFaceAlignment * This,
            UINT capacity,
            /* [annotation][size_is][out] */ 
            _Out_writes_all_(capacity)  float *animationUnits);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_Quality )( 
            IFaceAlignment * This,
            /* [annotation][out][retval] */ 
            _Out_  FaceAlignmentQuality *quality);
        
        END_INTERFACE
    } IFaceAlignmentVtbl;

    interface IFaceAlignment
    {
        CONST_VTBL struct IFaceAlignmentVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define IFaceAlignment_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define IFaceAlignment_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define IFaceAlignment_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define IFaceAlignment_get_FaceOrientation(This,faceOrientation)	\
    ( (This)->lpVtbl -> get_FaceOrientation(This,faceOrientation) ) 

#define IFaceAlignment_put_FaceOrientation(This,faceOrientation)	\
    ( (This)->lpVtbl -> put_FaceOrientation(This,faceOrientation) ) 

#define IFaceAlignment_get_HeadPivotPoint(This,headPivotPoint)	\
    ( (This)->lpVtbl -> get_HeadPivotPoint(This,headPivotPoint) ) 

#define IFaceAlignment_put_HeadPivotPoint(This,headPivotPoint)	\
    ( (This)->lpVtbl -> put_HeadPivotPoint(This,headPivotPoint) ) 

#define IFaceAlignment_get_FaceBoundingBox(This,boundingBox)	\
    ( (This)->lpVtbl -> get_FaceBoundingBox(This,boundingBox) ) 

#define IFaceAlignment_GetAnimationUnits(This,capacity,animationUnits)	\
    ( (This)->lpVtbl -> GetAnimationUnits(This,capacity,animationUnits) ) 

#define IFaceAlignment_get_Quality(This,quality)	\
    ( (This)->lpVtbl -> get_Quality(This,quality) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __IFaceAlignment_INTERFACE_DEFINED__ */


#ifndef __IFaceModel_INTERFACE_DEFINED__
#define __IFaceModel_INTERFACE_DEFINED__

/* interface IFaceModel */
/* [object][local][uuid] */ 


EXTERN_C const IID IID_IFaceModel;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("7B28C340-B6D0-44AC-8CF0-2D3F568C7D7E")
    IFaceModel : public IUnknown
    {
    public:
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_Scale( 
            /* [annotation][out][retval] */ 
            _Out_  float *scale) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_SkinColor( 
            /* [annotation][out][retval] */ 
            _Out_  UINT32 *skinColor) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_HairColor( 
            /* [annotation][out][retval] */ 
            _Out_  UINT32 *hairColor) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE GetFaceShapeDeformations( 
            UINT32 capacity,
            /* [annotation][size_is][out] */ 
            _Out_writes_all_(capacity)  float *faceShapeDeformations) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE CalculateVerticesForAlignment( 
            /* [annotation][in] */ 
            _In_  IFaceAlignment *faceAlignment,
            UINT capacity,
            /* [annotation][size_is][out] */ 
            _Out_writes_all_(capacity)  CameraSpacePoint *vertices) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct IFaceModelVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            IFaceModel * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            IFaceModel * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            IFaceModel * This);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_Scale )( 
            IFaceModel * This,
            /* [annotation][out][retval] */ 
            _Out_  float *scale);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_SkinColor )( 
            IFaceModel * This,
            /* [annotation][out][retval] */ 
            _Out_  UINT32 *skinColor);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_HairColor )( 
            IFaceModel * This,
            /* [annotation][out][retval] */ 
            _Out_  UINT32 *hairColor);
        
        HRESULT ( STDMETHODCALLTYPE *GetFaceShapeDeformations )( 
            IFaceModel * This,
            UINT32 capacity,
            /* [annotation][size_is][out] */ 
            _Out_writes_all_(capacity)  float *faceShapeDeformations);
        
        HRESULT ( STDMETHODCALLTYPE *CalculateVerticesForAlignment )( 
            IFaceModel * This,
            /* [annotation][in] */ 
            _In_  IFaceAlignment *faceAlignment,
            UINT capacity,
            /* [annotation][size_is][out] */ 
            _Out_writes_all_(capacity)  CameraSpacePoint *vertices);
        
        END_INTERFACE
    } IFaceModelVtbl;

    interface IFaceModel
    {
        CONST_VTBL struct IFaceModelVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define IFaceModel_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define IFaceModel_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define IFaceModel_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define IFaceModel_get_Scale(This,scale)	\
    ( (This)->lpVtbl -> get_Scale(This,scale) ) 

#define IFaceModel_get_SkinColor(This,skinColor)	\
    ( (This)->lpVtbl -> get_SkinColor(This,skinColor) ) 

#define IFaceModel_get_HairColor(This,hairColor)	\
    ( (This)->lpVtbl -> get_HairColor(This,hairColor) ) 

#define IFaceModel_GetFaceShapeDeformations(This,capacity,faceShapeDeformations)	\
    ( (This)->lpVtbl -> GetFaceShapeDeformations(This,capacity,faceShapeDeformations) ) 

#define IFaceModel_CalculateVerticesForAlignment(This,faceAlignment,capacity,vertices)	\
    ( (This)->lpVtbl -> CalculateVerticesForAlignment(This,faceAlignment,capacity,vertices) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __IFaceModel_INTERFACE_DEFINED__ */


#ifndef __IFaceModelBuilder_INTERFACE_DEFINED__
#define __IFaceModelBuilder_INTERFACE_DEFINED__

/* interface IFaceModelBuilder */
/* [object][local][uuid] */ 


EXTERN_C const IID IID_IFaceModelBuilder;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("BD92A663-1D1E-4CCF-B202-66DD49834BBE")
    IFaceModelBuilder : public IUnknown
    {
    public:
        virtual HRESULT STDMETHODCALLTYPE SubscribeCollectionStatusChanged( 
            /* [annotation][out][retval] */ 
            _Out_  WAITABLE_HANDLE *waitableHandle) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE UnsubscribeCollectionStatusChanged( 
            /* [annotation][in] */ 
            _In_  WAITABLE_HANDLE waitableHandle) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE GetCollectionStatusChangedEventData( 
            /* [annotation][in] */ 
            _In_  WAITABLE_HANDLE waitableHandle,
            /* [annotation][out][retval] */ 
            _Out_  ICollectionStatusChangedEventArgs **eventData) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE SubscribeCaptureStatusChanged( 
            /* [annotation][out][retval] */ 
            _Out_  WAITABLE_HANDLE *waitableHandle) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE UnsubscribeCaptureStatusChanged( 
            /* [annotation][in] */ 
            _In_  WAITABLE_HANDLE waitableHandle) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE GetCaptureStatusChangedEventData( 
            /* [annotation][in] */ 
            _In_  WAITABLE_HANDLE waitableHandle,
            /* [annotation][out][retval] */ 
            _Out_  ICaptureStatusChangedEventArgs **eventData) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_CollectionStatus( 
            /* [annotation][out][retval] */ 
            _Out_  FaceModelBuilderCollectionStatus *currrentCollectionStatus) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_CaptureStatus( 
            /* [annotation][out][retval] */ 
            _Out_  FaceModelBuilderCaptureStatus *currentCaptureStatus) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE BeginFaceDataCollection( void) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE GetFaceData( 
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IFaceModelData **faceModelData) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct IFaceModelBuilderVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            IFaceModelBuilder * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            IFaceModelBuilder * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            IFaceModelBuilder * This);
        
        HRESULT ( STDMETHODCALLTYPE *SubscribeCollectionStatusChanged )( 
            IFaceModelBuilder * This,
            /* [annotation][out][retval] */ 
            _Out_  WAITABLE_HANDLE *waitableHandle);
        
        HRESULT ( STDMETHODCALLTYPE *UnsubscribeCollectionStatusChanged )( 
            IFaceModelBuilder * This,
            /* [annotation][in] */ 
            _In_  WAITABLE_HANDLE waitableHandle);
        
        HRESULT ( STDMETHODCALLTYPE *GetCollectionStatusChangedEventData )( 
            IFaceModelBuilder * This,
            /* [annotation][in] */ 
            _In_  WAITABLE_HANDLE waitableHandle,
            /* [annotation][out][retval] */ 
            _Out_  ICollectionStatusChangedEventArgs **eventData);
        
        HRESULT ( STDMETHODCALLTYPE *SubscribeCaptureStatusChanged )( 
            IFaceModelBuilder * This,
            /* [annotation][out][retval] */ 
            _Out_  WAITABLE_HANDLE *waitableHandle);
        
        HRESULT ( STDMETHODCALLTYPE *UnsubscribeCaptureStatusChanged )( 
            IFaceModelBuilder * This,
            /* [annotation][in] */ 
            _In_  WAITABLE_HANDLE waitableHandle);
        
        HRESULT ( STDMETHODCALLTYPE *GetCaptureStatusChangedEventData )( 
            IFaceModelBuilder * This,
            /* [annotation][in] */ 
            _In_  WAITABLE_HANDLE waitableHandle,
            /* [annotation][out][retval] */ 
            _Out_  ICaptureStatusChangedEventArgs **eventData);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_CollectionStatus )( 
            IFaceModelBuilder * This,
            /* [annotation][out][retval] */ 
            _Out_  FaceModelBuilderCollectionStatus *currrentCollectionStatus);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_CaptureStatus )( 
            IFaceModelBuilder * This,
            /* [annotation][out][retval] */ 
            _Out_  FaceModelBuilderCaptureStatus *currentCaptureStatus);
        
        HRESULT ( STDMETHODCALLTYPE *BeginFaceDataCollection )( 
            IFaceModelBuilder * This);
        
        HRESULT ( STDMETHODCALLTYPE *GetFaceData )( 
            IFaceModelBuilder * This,
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IFaceModelData **faceModelData);
        
        END_INTERFACE
    } IFaceModelBuilderVtbl;

    interface IFaceModelBuilder
    {
        CONST_VTBL struct IFaceModelBuilderVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define IFaceModelBuilder_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define IFaceModelBuilder_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define IFaceModelBuilder_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define IFaceModelBuilder_SubscribeCollectionStatusChanged(This,waitableHandle)	\
    ( (This)->lpVtbl -> SubscribeCollectionStatusChanged(This,waitableHandle) ) 

#define IFaceModelBuilder_UnsubscribeCollectionStatusChanged(This,waitableHandle)	\
    ( (This)->lpVtbl -> UnsubscribeCollectionStatusChanged(This,waitableHandle) ) 

#define IFaceModelBuilder_GetCollectionStatusChangedEventData(This,waitableHandle,eventData)	\
    ( (This)->lpVtbl -> GetCollectionStatusChangedEventData(This,waitableHandle,eventData) ) 

#define IFaceModelBuilder_SubscribeCaptureStatusChanged(This,waitableHandle)	\
    ( (This)->lpVtbl -> SubscribeCaptureStatusChanged(This,waitableHandle) ) 

#define IFaceModelBuilder_UnsubscribeCaptureStatusChanged(This,waitableHandle)	\
    ( (This)->lpVtbl -> UnsubscribeCaptureStatusChanged(This,waitableHandle) ) 

#define IFaceModelBuilder_GetCaptureStatusChangedEventData(This,waitableHandle,eventData)	\
    ( (This)->lpVtbl -> GetCaptureStatusChangedEventData(This,waitableHandle,eventData) ) 

#define IFaceModelBuilder_get_CollectionStatus(This,currrentCollectionStatus)	\
    ( (This)->lpVtbl -> get_CollectionStatus(This,currrentCollectionStatus) ) 

#define IFaceModelBuilder_get_CaptureStatus(This,currentCaptureStatus)	\
    ( (This)->lpVtbl -> get_CaptureStatus(This,currentCaptureStatus) ) 

#define IFaceModelBuilder_BeginFaceDataCollection(This)	\
    ( (This)->lpVtbl -> BeginFaceDataCollection(This) ) 

#define IFaceModelBuilder_GetFaceData(This,faceModelData)	\
    ( (This)->lpVtbl -> GetFaceData(This,faceModelData) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __IFaceModelBuilder_INTERFACE_DEFINED__ */


#ifndef __IFaceModelData_INTERFACE_DEFINED__
#define __IFaceModelData_INTERFACE_DEFINED__

/* interface IFaceModelData */
/* [object][local][uuid] */ 


EXTERN_C const IID IID_IFaceModelData;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("ABD8D92D-E579-466E-BD9E-232E0134BCE0")
    IFaceModelData : public IUnknown
    {
    public:
        virtual HRESULT STDMETHODCALLTYPE ProduceFaceModel( 
            /* [annotation][out][retval] */ 
            _COM_Outptr_result_maybenull_  IFaceModel **faceModel) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct IFaceModelDataVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            IFaceModelData * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            IFaceModelData * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            IFaceModelData * This);
        
        HRESULT ( STDMETHODCALLTYPE *ProduceFaceModel )( 
            IFaceModelData * This,
            /* [annotation][out][retval] */ 
            _COM_Outptr_result_maybenull_  IFaceModel **faceModel);
        
        END_INTERFACE
    } IFaceModelDataVtbl;

    interface IFaceModelData
    {
        CONST_VTBL struct IFaceModelDataVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define IFaceModelData_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define IFaceModelData_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define IFaceModelData_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define IFaceModelData_ProduceFaceModel(This,faceModel)	\
    ( (This)->lpVtbl -> ProduceFaceModel(This,faceModel) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __IFaceModelData_INTERFACE_DEFINED__ */


#ifndef __IHighDefinitionFaceFrameReference_INTERFACE_DEFINED__
#define __IHighDefinitionFaceFrameReference_INTERFACE_DEFINED__

/* interface IHighDefinitionFaceFrameReference */
/* [object][local][uuid] */ 


EXTERN_C const IID IID_IHighDefinitionFaceFrameReference;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("16806807-70B9-4D0C-A74B-E807783969C3")
    IHighDefinitionFaceFrameReference : public IUnknown
    {
    public:
        virtual HRESULT STDMETHODCALLTYPE AcquireFrame( 
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IHighDefinitionFaceFrame **highDefinitonFaceFrame) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_RelativeTime( 
            /* [annotation][out][retval] */ 
            _Out_  TIMESPAN *relativeTime) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct IHighDefinitionFaceFrameReferenceVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            IHighDefinitionFaceFrameReference * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            IHighDefinitionFaceFrameReference * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            IHighDefinitionFaceFrameReference * This);
        
        HRESULT ( STDMETHODCALLTYPE *AcquireFrame )( 
            IHighDefinitionFaceFrameReference * This,
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IHighDefinitionFaceFrame **highDefinitonFaceFrame);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_RelativeTime )( 
            IHighDefinitionFaceFrameReference * This,
            /* [annotation][out][retval] */ 
            _Out_  TIMESPAN *relativeTime);
        
        END_INTERFACE
    } IHighDefinitionFaceFrameReferenceVtbl;

    interface IHighDefinitionFaceFrameReference
    {
        CONST_VTBL struct IHighDefinitionFaceFrameReferenceVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define IHighDefinitionFaceFrameReference_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define IHighDefinitionFaceFrameReference_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define IHighDefinitionFaceFrameReference_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define IHighDefinitionFaceFrameReference_AcquireFrame(This,highDefinitonFaceFrame)	\
    ( (This)->lpVtbl -> AcquireFrame(This,highDefinitonFaceFrame) ) 

#define IHighDefinitionFaceFrameReference_get_RelativeTime(This,relativeTime)	\
    ( (This)->lpVtbl -> get_RelativeTime(This,relativeTime) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __IHighDefinitionFaceFrameReference_INTERFACE_DEFINED__ */


#ifndef __IHighDefinitionFaceFrameArrivedEventArgs_INTERFACE_DEFINED__
#define __IHighDefinitionFaceFrameArrivedEventArgs_INTERFACE_DEFINED__

/* interface IHighDefinitionFaceFrameArrivedEventArgs */
/* [object][local][uuid] */ 


EXTERN_C const IID IID_IHighDefinitionFaceFrameArrivedEventArgs;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("9B0BAB99-4EC6-4CAB-83F4-3A3C9D864333")
    IHighDefinitionFaceFrameArrivedEventArgs : public IUnknown
    {
    public:
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_FrameReference( 
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IHighDefinitionFaceFrameReference **highDefinitonFaceFrameReference) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct IHighDefinitionFaceFrameArrivedEventArgsVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            IHighDefinitionFaceFrameArrivedEventArgs * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            IHighDefinitionFaceFrameArrivedEventArgs * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            IHighDefinitionFaceFrameArrivedEventArgs * This);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_FrameReference )( 
            IHighDefinitionFaceFrameArrivedEventArgs * This,
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IHighDefinitionFaceFrameReference **highDefinitonFaceFrameReference);
        
        END_INTERFACE
    } IHighDefinitionFaceFrameArrivedEventArgsVtbl;

    interface IHighDefinitionFaceFrameArrivedEventArgs
    {
        CONST_VTBL struct IHighDefinitionFaceFrameArrivedEventArgsVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define IHighDefinitionFaceFrameArrivedEventArgs_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define IHighDefinitionFaceFrameArrivedEventArgs_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define IHighDefinitionFaceFrameArrivedEventArgs_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define IHighDefinitionFaceFrameArrivedEventArgs_get_FrameReference(This,highDefinitonFaceFrameReference)	\
    ( (This)->lpVtbl -> get_FrameReference(This,highDefinitonFaceFrameReference) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __IHighDefinitionFaceFrameArrivedEventArgs_INTERFACE_DEFINED__ */


#ifndef __IHighDefinitionFaceFrameSource_INTERFACE_DEFINED__
#define __IHighDefinitionFaceFrameSource_INTERFACE_DEFINED__

/* interface IHighDefinitionFaceFrameSource */
/* [object][local][uuid] */ 


EXTERN_C const IID IID_IHighDefinitionFaceFrameSource;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("E6F33685-BEFC-48CA-B81A-B6A8A79C32FE")
    IHighDefinitionFaceFrameSource : public IUnknown
    {
    public:
        virtual HRESULT STDMETHODCALLTYPE SubscribeTrackingIdLost( 
            /* [annotation][out][retval] */ 
            _Out_  WAITABLE_HANDLE *waitableHandle) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE UnsubscribeTrackingIdLost( 
            /* [annotation][in] */ 
            _In_  WAITABLE_HANDLE waitableHandle) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE GetTrackingIdLostEventData( 
            /* [annotation][in] */ 
            _In_  WAITABLE_HANDLE waitableHandle,
            /* [annotation][out][retval] */ 
            _Out_  ITrackingIdLostEventArgs **eventData) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_IsOnline( 
            /* [annotation][out][retval] */ 
            _Out_  BOOLEAN *isOnline) = 0;
        
        virtual /* [propput] */ HRESULT STDMETHODCALLTYPE put_IsOnline( 
            BOOLEAN isOnline) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_IsActive( 
            /* [annotation][out][retval] */ 
            _Out_  BOOLEAN *isActive) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_TrackingId( 
            /* [annotation][out][retval] */ 
            _Out_  UINT64 *trackingId) = 0;
        
        virtual /* [propput] */ HRESULT STDMETHODCALLTYPE put_TrackingId( 
            UINT64 trackingId) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_IsTrackingIdValid( 
            /* [annotation][out][retval] */ 
            _Out_  BOOLEAN *isTrackingIdValid) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_TrackingQuality( 
            /* [annotation][out][retval] */ 
            _Out_  FaceAlignmentQuality *trackingQuality) = 0;
        
        virtual /* [propput] */ HRESULT STDMETHODCALLTYPE put_TrackingQuality( 
            FaceAlignmentQuality trackingQuality) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_FaceModel( 
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IFaceModel **faceModel) = 0;
        
        virtual /* [propput] */ HRESULT STDMETHODCALLTYPE put_FaceModel( 
            /* [annotation][in] */ 
            _In_  IFaceModel *faceModel) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE OpenReader( 
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IHighDefinitionFaceFrameReader **reader) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE OpenModelBuilder( 
            FaceModelBuilderAttributes enabledAttributes,
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IFaceModelBuilder **modelbuilder) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_KinectSensor( 
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IKinectSensor **sensor) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE FeedAndCalculateFrameData( 
            /* [annotation][in] */ 
            _In_  IBody *body,
            UINT infraredCapacity,
            /* [annotation][size_is][in] */ 
            _In_reads_(infraredCapacity)  UINT16 *infraredFrameBuffer,
            UINT colorCapacity,
            /* [annotation][size_is][in] */ 
            _In_reads_(colorCapacity)  BYTE *colorFrameBuffer,
            UINT depthCapacity,
            /* [annotation][size_is][in] */ 
            _In_reads_(depthCapacity)  UINT16 *depthFrameBuffer) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct IHighDefinitionFaceFrameSourceVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            IHighDefinitionFaceFrameSource * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            IHighDefinitionFaceFrameSource * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            IHighDefinitionFaceFrameSource * This);
        
        HRESULT ( STDMETHODCALLTYPE *SubscribeTrackingIdLost )( 
            IHighDefinitionFaceFrameSource * This,
            /* [annotation][out][retval] */ 
            _Out_  WAITABLE_HANDLE *waitableHandle);
        
        HRESULT ( STDMETHODCALLTYPE *UnsubscribeTrackingIdLost )( 
            IHighDefinitionFaceFrameSource * This,
            /* [annotation][in] */ 
            _In_  WAITABLE_HANDLE waitableHandle);
        
        HRESULT ( STDMETHODCALLTYPE *GetTrackingIdLostEventData )( 
            IHighDefinitionFaceFrameSource * This,
            /* [annotation][in] */ 
            _In_  WAITABLE_HANDLE waitableHandle,
            /* [annotation][out][retval] */ 
            _Out_  ITrackingIdLostEventArgs **eventData);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_IsOnline )( 
            IHighDefinitionFaceFrameSource * This,
            /* [annotation][out][retval] */ 
            _Out_  BOOLEAN *isOnline);
        
        /* [propput] */ HRESULT ( STDMETHODCALLTYPE *put_IsOnline )( 
            IHighDefinitionFaceFrameSource * This,
            BOOLEAN isOnline);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_IsActive )( 
            IHighDefinitionFaceFrameSource * This,
            /* [annotation][out][retval] */ 
            _Out_  BOOLEAN *isActive);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_TrackingId )( 
            IHighDefinitionFaceFrameSource * This,
            /* [annotation][out][retval] */ 
            _Out_  UINT64 *trackingId);
        
        /* [propput] */ HRESULT ( STDMETHODCALLTYPE *put_TrackingId )( 
            IHighDefinitionFaceFrameSource * This,
            UINT64 trackingId);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_IsTrackingIdValid )( 
            IHighDefinitionFaceFrameSource * This,
            /* [annotation][out][retval] */ 
            _Out_  BOOLEAN *isTrackingIdValid);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_TrackingQuality )( 
            IHighDefinitionFaceFrameSource * This,
            /* [annotation][out][retval] */ 
            _Out_  FaceAlignmentQuality *trackingQuality);
        
        /* [propput] */ HRESULT ( STDMETHODCALLTYPE *put_TrackingQuality )( 
            IHighDefinitionFaceFrameSource * This,
            FaceAlignmentQuality trackingQuality);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_FaceModel )( 
            IHighDefinitionFaceFrameSource * This,
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IFaceModel **faceModel);
        
        /* [propput] */ HRESULT ( STDMETHODCALLTYPE *put_FaceModel )( 
            IHighDefinitionFaceFrameSource * This,
            /* [annotation][in] */ 
            _In_  IFaceModel *faceModel);
        
        HRESULT ( STDMETHODCALLTYPE *OpenReader )( 
            IHighDefinitionFaceFrameSource * This,
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IHighDefinitionFaceFrameReader **reader);
        
        HRESULT ( STDMETHODCALLTYPE *OpenModelBuilder )( 
            IHighDefinitionFaceFrameSource * This,
            FaceModelBuilderAttributes enabledAttributes,
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IFaceModelBuilder **modelbuilder);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_KinectSensor )( 
            IHighDefinitionFaceFrameSource * This,
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IKinectSensor **sensor);
        
        HRESULT ( STDMETHODCALLTYPE *FeedAndCalculateFrameData )( 
            IHighDefinitionFaceFrameSource * This,
            /* [annotation][in] */ 
            _In_  IBody *body,
            UINT infraredCapacity,
            /* [annotation][size_is][in] */ 
            _In_reads_(infraredCapacity)  UINT16 *infraredFrameBuffer,
            UINT colorCapacity,
            /* [annotation][size_is][in] */ 
            _In_reads_(colorCapacity)  BYTE *colorFrameBuffer,
            UINT depthCapacity,
            /* [annotation][size_is][in] */ 
            _In_reads_(depthCapacity)  UINT16 *depthFrameBuffer);
        
        END_INTERFACE
    } IHighDefinitionFaceFrameSourceVtbl;

    interface IHighDefinitionFaceFrameSource
    {
        CONST_VTBL struct IHighDefinitionFaceFrameSourceVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define IHighDefinitionFaceFrameSource_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define IHighDefinitionFaceFrameSource_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define IHighDefinitionFaceFrameSource_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define IHighDefinitionFaceFrameSource_SubscribeTrackingIdLost(This,waitableHandle)	\
    ( (This)->lpVtbl -> SubscribeTrackingIdLost(This,waitableHandle) ) 

#define IHighDefinitionFaceFrameSource_UnsubscribeTrackingIdLost(This,waitableHandle)	\
    ( (This)->lpVtbl -> UnsubscribeTrackingIdLost(This,waitableHandle) ) 

#define IHighDefinitionFaceFrameSource_GetTrackingIdLostEventData(This,waitableHandle,eventData)	\
    ( (This)->lpVtbl -> GetTrackingIdLostEventData(This,waitableHandle,eventData) ) 

#define IHighDefinitionFaceFrameSource_get_IsOnline(This,isOnline)	\
    ( (This)->lpVtbl -> get_IsOnline(This,isOnline) ) 

#define IHighDefinitionFaceFrameSource_put_IsOnline(This,isOnline)	\
    ( (This)->lpVtbl -> put_IsOnline(This,isOnline) ) 

#define IHighDefinitionFaceFrameSource_get_IsActive(This,isActive)	\
    ( (This)->lpVtbl -> get_IsActive(This,isActive) ) 

#define IHighDefinitionFaceFrameSource_get_TrackingId(This,trackingId)	\
    ( (This)->lpVtbl -> get_TrackingId(This,trackingId) ) 

#define IHighDefinitionFaceFrameSource_put_TrackingId(This,trackingId)	\
    ( (This)->lpVtbl -> put_TrackingId(This,trackingId) ) 

#define IHighDefinitionFaceFrameSource_get_IsTrackingIdValid(This,isTrackingIdValid)	\
    ( (This)->lpVtbl -> get_IsTrackingIdValid(This,isTrackingIdValid) ) 

#define IHighDefinitionFaceFrameSource_get_TrackingQuality(This,trackingQuality)	\
    ( (This)->lpVtbl -> get_TrackingQuality(This,trackingQuality) ) 

#define IHighDefinitionFaceFrameSource_put_TrackingQuality(This,trackingQuality)	\
    ( (This)->lpVtbl -> put_TrackingQuality(This,trackingQuality) ) 

#define IHighDefinitionFaceFrameSource_get_FaceModel(This,faceModel)	\
    ( (This)->lpVtbl -> get_FaceModel(This,faceModel) ) 

#define IHighDefinitionFaceFrameSource_put_FaceModel(This,faceModel)	\
    ( (This)->lpVtbl -> put_FaceModel(This,faceModel) ) 

#define IHighDefinitionFaceFrameSource_OpenReader(This,reader)	\
    ( (This)->lpVtbl -> OpenReader(This,reader) ) 

#define IHighDefinitionFaceFrameSource_OpenModelBuilder(This,enabledAttributes,modelbuilder)	\
    ( (This)->lpVtbl -> OpenModelBuilder(This,enabledAttributes,modelbuilder) ) 

#define IHighDefinitionFaceFrameSource_get_KinectSensor(This,sensor)	\
    ( (This)->lpVtbl -> get_KinectSensor(This,sensor) ) 

#define IHighDefinitionFaceFrameSource_FeedAndCalculateFrameData(This,body,infraredCapacity,infraredFrameBuffer,colorCapacity,colorFrameBuffer,depthCapacity,depthFrameBuffer)	\
    ( (This)->lpVtbl -> FeedAndCalculateFrameData(This,body,infraredCapacity,infraredFrameBuffer,colorCapacity,colorFrameBuffer,depthCapacity,depthFrameBuffer) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __IHighDefinitionFaceFrameSource_INTERFACE_DEFINED__ */


#ifndef __IHighDefinitionFaceFrameReader_INTERFACE_DEFINED__
#define __IHighDefinitionFaceFrameReader_INTERFACE_DEFINED__

/* interface IHighDefinitionFaceFrameReader */
/* [object][local][uuid] */ 


EXTERN_C const IID IID_IHighDefinitionFaceFrameReader;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("FAB67369-E051-4790-8A1D-1CEDB9DE86CA")
    IHighDefinitionFaceFrameReader : public IUnknown
    {
    public:
        virtual HRESULT STDMETHODCALLTYPE SubscribeFrameArrived( 
            /* [annotation][out][retval] */ 
            _Out_  WAITABLE_HANDLE *waitableHandle) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE UnsubscribeFrameArrived( 
            /* [annotation][in] */ 
            _In_  WAITABLE_HANDLE waitableHandle) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE GetFrameArrivedEventData( 
            /* [annotation][in] */ 
            _In_  WAITABLE_HANDLE waitableHandle,
            /* [annotation][out][retval] */ 
            _Out_  IHighDefinitionFaceFrameArrivedEventArgs **eventData) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE AcquireLatestFrame( 
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IHighDefinitionFaceFrame **faceFrame) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_IsPaused( 
            /* [annotation][out][retval] */ 
            _Out_  BOOLEAN *isPaused) = 0;
        
        virtual /* [propput] */ HRESULT STDMETHODCALLTYPE put_IsPaused( 
            BOOLEAN isPaused) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_HighDefinitionFaceFrameSource( 
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IHighDefinitionFaceFrameSource **highDefinitionFaceFrameSource) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct IHighDefinitionFaceFrameReaderVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            IHighDefinitionFaceFrameReader * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            IHighDefinitionFaceFrameReader * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            IHighDefinitionFaceFrameReader * This);
        
        HRESULT ( STDMETHODCALLTYPE *SubscribeFrameArrived )( 
            IHighDefinitionFaceFrameReader * This,
            /* [annotation][out][retval] */ 
            _Out_  WAITABLE_HANDLE *waitableHandle);
        
        HRESULT ( STDMETHODCALLTYPE *UnsubscribeFrameArrived )( 
            IHighDefinitionFaceFrameReader * This,
            /* [annotation][in] */ 
            _In_  WAITABLE_HANDLE waitableHandle);
        
        HRESULT ( STDMETHODCALLTYPE *GetFrameArrivedEventData )( 
            IHighDefinitionFaceFrameReader * This,
            /* [annotation][in] */ 
            _In_  WAITABLE_HANDLE waitableHandle,
            /* [annotation][out][retval] */ 
            _Out_  IHighDefinitionFaceFrameArrivedEventArgs **eventData);
        
        HRESULT ( STDMETHODCALLTYPE *AcquireLatestFrame )( 
            IHighDefinitionFaceFrameReader * This,
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IHighDefinitionFaceFrame **faceFrame);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_IsPaused )( 
            IHighDefinitionFaceFrameReader * This,
            /* [annotation][out][retval] */ 
            _Out_  BOOLEAN *isPaused);
        
        /* [propput] */ HRESULT ( STDMETHODCALLTYPE *put_IsPaused )( 
            IHighDefinitionFaceFrameReader * This,
            BOOLEAN isPaused);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_HighDefinitionFaceFrameSource )( 
            IHighDefinitionFaceFrameReader * This,
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IHighDefinitionFaceFrameSource **highDefinitionFaceFrameSource);
        
        END_INTERFACE
    } IHighDefinitionFaceFrameReaderVtbl;

    interface IHighDefinitionFaceFrameReader
    {
        CONST_VTBL struct IHighDefinitionFaceFrameReaderVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define IHighDefinitionFaceFrameReader_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define IHighDefinitionFaceFrameReader_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define IHighDefinitionFaceFrameReader_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define IHighDefinitionFaceFrameReader_SubscribeFrameArrived(This,waitableHandle)	\
    ( (This)->lpVtbl -> SubscribeFrameArrived(This,waitableHandle) ) 

#define IHighDefinitionFaceFrameReader_UnsubscribeFrameArrived(This,waitableHandle)	\
    ( (This)->lpVtbl -> UnsubscribeFrameArrived(This,waitableHandle) ) 

#define IHighDefinitionFaceFrameReader_GetFrameArrivedEventData(This,waitableHandle,eventData)	\
    ( (This)->lpVtbl -> GetFrameArrivedEventData(This,waitableHandle,eventData) ) 

#define IHighDefinitionFaceFrameReader_AcquireLatestFrame(This,faceFrame)	\
    ( (This)->lpVtbl -> AcquireLatestFrame(This,faceFrame) ) 

#define IHighDefinitionFaceFrameReader_get_IsPaused(This,isPaused)	\
    ( (This)->lpVtbl -> get_IsPaused(This,isPaused) ) 

#define IHighDefinitionFaceFrameReader_put_IsPaused(This,isPaused)	\
    ( (This)->lpVtbl -> put_IsPaused(This,isPaused) ) 

#define IHighDefinitionFaceFrameReader_get_HighDefinitionFaceFrameSource(This,highDefinitionFaceFrameSource)	\
    ( (This)->lpVtbl -> get_HighDefinitionFaceFrameSource(This,highDefinitionFaceFrameSource) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __IHighDefinitionFaceFrameReader_INTERFACE_DEFINED__ */


#ifndef __IHighDefinitionFaceFrame_INTERFACE_DEFINED__
#define __IHighDefinitionFaceFrame_INTERFACE_DEFINED__

/* interface IHighDefinitionFaceFrame */
/* [object][local][uuid] */ 


EXTERN_C const IID IID_IHighDefinitionFaceFrame;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("303A5D6F-1825-41D0-BA14-647FDC7D8A89")
    IHighDefinitionFaceFrame : public IUnknown
    {
    public:
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_RelativeTime( 
            /* [annotation][out][retval] */ 
            _Out_  TIMESPAN *timeCounter) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_HighDefinitionFaceFrameSource( 
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IHighDefinitionFaceFrameSource **highDefinitionFaceFrameSource) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_BodyFrameReference( 
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IBodyFrameReference **bodyFrameReference) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_ColorFrameReference( 
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IColorFrameReference **colorFrameReference) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_DepthFrameReference( 
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IDepthFrameReference **depthFrameReference) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_InfraredFrameReference( 
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IInfraredFrameReference **infraredFrameReference) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_IsTrackingIdValid( 
            /* [annotation][out][retval] */ 
            _Out_  BOOLEAN *isTrackingIdValid) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_IsFaceTracked( 
            /* [annotation][out][retval] */ 
            _Out_  BOOLEAN *isFaceTracked) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_TrackingId( 
            /* [annotation][out][retval] */ 
            _Out_  UINT64 *trackingId) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_FaceModel( 
            /* [annotation][out][retval] */ 
            _COM_Outptr_result_maybenull_  IFaceModel **faceModel) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_FaceAlignmentQuality( 
            /* [annotation][out][retval] */ 
            _Out_  FaceAlignmentQuality *quality) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE GetAndRefreshFaceAlignmentResult( 
            /* [annotation][in] */ 
            _In_  IFaceAlignment *faceAlignmentResults) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct IHighDefinitionFaceFrameVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            IHighDefinitionFaceFrame * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            IHighDefinitionFaceFrame * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            IHighDefinitionFaceFrame * This);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_RelativeTime )( 
            IHighDefinitionFaceFrame * This,
            /* [annotation][out][retval] */ 
            _Out_  TIMESPAN *timeCounter);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_HighDefinitionFaceFrameSource )( 
            IHighDefinitionFaceFrame * This,
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IHighDefinitionFaceFrameSource **highDefinitionFaceFrameSource);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_BodyFrameReference )( 
            IHighDefinitionFaceFrame * This,
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IBodyFrameReference **bodyFrameReference);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_ColorFrameReference )( 
            IHighDefinitionFaceFrame * This,
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IColorFrameReference **colorFrameReference);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_DepthFrameReference )( 
            IHighDefinitionFaceFrame * This,
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IDepthFrameReference **depthFrameReference);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_InfraredFrameReference )( 
            IHighDefinitionFaceFrame * This,
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IInfraredFrameReference **infraredFrameReference);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_IsTrackingIdValid )( 
            IHighDefinitionFaceFrame * This,
            /* [annotation][out][retval] */ 
            _Out_  BOOLEAN *isTrackingIdValid);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_IsFaceTracked )( 
            IHighDefinitionFaceFrame * This,
            /* [annotation][out][retval] */ 
            _Out_  BOOLEAN *isFaceTracked);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_TrackingId )( 
            IHighDefinitionFaceFrame * This,
            /* [annotation][out][retval] */ 
            _Out_  UINT64 *trackingId);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_FaceModel )( 
            IHighDefinitionFaceFrame * This,
            /* [annotation][out][retval] */ 
            _COM_Outptr_result_maybenull_  IFaceModel **faceModel);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_FaceAlignmentQuality )( 
            IHighDefinitionFaceFrame * This,
            /* [annotation][out][retval] */ 
            _Out_  FaceAlignmentQuality *quality);
        
        HRESULT ( STDMETHODCALLTYPE *GetAndRefreshFaceAlignmentResult )( 
            IHighDefinitionFaceFrame * This,
            /* [annotation][in] */ 
            _In_  IFaceAlignment *faceAlignmentResults);
        
        END_INTERFACE
    } IHighDefinitionFaceFrameVtbl;

    interface IHighDefinitionFaceFrame
    {
        CONST_VTBL struct IHighDefinitionFaceFrameVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define IHighDefinitionFaceFrame_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define IHighDefinitionFaceFrame_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define IHighDefinitionFaceFrame_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define IHighDefinitionFaceFrame_get_RelativeTime(This,timeCounter)	\
    ( (This)->lpVtbl -> get_RelativeTime(This,timeCounter) ) 

#define IHighDefinitionFaceFrame_get_HighDefinitionFaceFrameSource(This,highDefinitionFaceFrameSource)	\
    ( (This)->lpVtbl -> get_HighDefinitionFaceFrameSource(This,highDefinitionFaceFrameSource) ) 

#define IHighDefinitionFaceFrame_get_BodyFrameReference(This,bodyFrameReference)	\
    ( (This)->lpVtbl -> get_BodyFrameReference(This,bodyFrameReference) ) 

#define IHighDefinitionFaceFrame_get_ColorFrameReference(This,colorFrameReference)	\
    ( (This)->lpVtbl -> get_ColorFrameReference(This,colorFrameReference) ) 

#define IHighDefinitionFaceFrame_get_DepthFrameReference(This,depthFrameReference)	\
    ( (This)->lpVtbl -> get_DepthFrameReference(This,depthFrameReference) ) 

#define IHighDefinitionFaceFrame_get_InfraredFrameReference(This,infraredFrameReference)	\
    ( (This)->lpVtbl -> get_InfraredFrameReference(This,infraredFrameReference) ) 

#define IHighDefinitionFaceFrame_get_IsTrackingIdValid(This,isTrackingIdValid)	\
    ( (This)->lpVtbl -> get_IsTrackingIdValid(This,isTrackingIdValid) ) 

#define IHighDefinitionFaceFrame_get_IsFaceTracked(This,isFaceTracked)	\
    ( (This)->lpVtbl -> get_IsFaceTracked(This,isFaceTracked) ) 

#define IHighDefinitionFaceFrame_get_TrackingId(This,trackingId)	\
    ( (This)->lpVtbl -> get_TrackingId(This,trackingId) ) 

#define IHighDefinitionFaceFrame_get_FaceModel(This,faceModel)	\
    ( (This)->lpVtbl -> get_FaceModel(This,faceModel) ) 

#define IHighDefinitionFaceFrame_get_FaceAlignmentQuality(This,quality)	\
    ( (This)->lpVtbl -> get_FaceAlignmentQuality(This,quality) ) 

#define IHighDefinitionFaceFrame_GetAndRefreshFaceAlignmentResult(This,faceAlignmentResults)	\
    ( (This)->lpVtbl -> GetAndRefreshFaceAlignmentResult(This,faceAlignmentResults) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __IHighDefinitionFaceFrame_INTERFACE_DEFINED__ */


/* interface __MIDL_itf_Kinect2EFace_0000_0018 */
/* [local] */ 

HRESULT WINAPI CreateFaceFrameSource(_In_ IKinectSensor* sensor, _In_ const UINT64 initialTrackingId, _In_ const DWORD initialFaceFrameFeatures, _COM_Outptr_ IFaceFrameSource** ppSource);
HRESULT WINAPI CreateHighDefinitionFaceFrameSource(_In_ IKinectSensor* sensor, _COM_Outptr_ IHighDefinitionFaceFrameSource** hdFaceFrameSource);
HRESULT WINAPI CreateFaceAlignment(_COM_Outptr_ IFaceAlignment** ppFaceAlignment);
HRESULT WINAPI CreateFaceModel(float scale, UINT32 capacity, _In_reads_(capacity) float* pDeformations, _COM_Outptr_ IFaceModel** ppFaceModel);
HRESULT WINAPI GetFaceModelVertexCount(_Out_ UINT32* pVertexCount);
HRESULT WINAPI GetFaceModelTriangleCount(_Out_ UINT32* pTriangleCount);
HRESULT WINAPI GetFaceModelTriangles(UINT32 capacity, _Out_writes_all_(capacity) UINT32* triangeVertices);


extern RPC_IF_HANDLE __MIDL_itf_Kinect2EFace_0000_0018_v0_0_c_ifspec;
extern RPC_IF_HANDLE __MIDL_itf_Kinect2EFace_0000_0018_v0_0_s_ifspec;

/* Additional Prototypes for ALL interfaces */

/* end of Additional Prototypes */

#ifdef __cplusplus
}
#endif

#endif


