

/* this ALWAYS GENERATED file contains the definitions for the interfaces */


 /* File created by MIDL compiler version 8.00.0595 */
/* at Sun Oct 19 13:17:53 2014
 */
/* Compiler settings for ..\..\idl\Kinect.VisualGestureBuilder.idl:
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

#ifndef __Kinect2EVisualGestureBuilder_h__
#define __Kinect2EVisualGestureBuilder_h__

#if defined(_MSC_VER) && (_MSC_VER >= 1020)
#pragma once
#endif

/* Forward Declarations */ 

#ifndef __ITrackingIdLostEventArgs_FWD_DEFINED__
#define __ITrackingIdLostEventArgs_FWD_DEFINED__
typedef interface ITrackingIdLostEventArgs ITrackingIdLostEventArgs;

#endif 	/* __ITrackingIdLostEventArgs_FWD_DEFINED__ */


#ifndef __IGesture_FWD_DEFINED__
#define __IGesture_FWD_DEFINED__
typedef interface IGesture IGesture;

#endif 	/* __IGesture_FWD_DEFINED__ */


#ifndef __IDiscreteGestureResult_FWD_DEFINED__
#define __IDiscreteGestureResult_FWD_DEFINED__
typedef interface IDiscreteGestureResult IDiscreteGestureResult;

#endif 	/* __IDiscreteGestureResult_FWD_DEFINED__ */


#ifndef __IContinuousGestureResult_FWD_DEFINED__
#define __IContinuousGestureResult_FWD_DEFINED__
typedef interface IContinuousGestureResult IContinuousGestureResult;

#endif 	/* __IContinuousGestureResult_FWD_DEFINED__ */


#ifndef __IVisualGestureBuilderDatabase_FWD_DEFINED__
#define __IVisualGestureBuilderDatabase_FWD_DEFINED__
typedef interface IVisualGestureBuilderDatabase IVisualGestureBuilderDatabase;

#endif 	/* __IVisualGestureBuilderDatabase_FWD_DEFINED__ */


#ifndef __IVisualGestureBuilderFrameSource_FWD_DEFINED__
#define __IVisualGestureBuilderFrameSource_FWD_DEFINED__
typedef interface IVisualGestureBuilderFrameSource IVisualGestureBuilderFrameSource;

#endif 	/* __IVisualGestureBuilderFrameSource_FWD_DEFINED__ */


#ifndef __IVisualGestureBuilderFrameReader_FWD_DEFINED__
#define __IVisualGestureBuilderFrameReader_FWD_DEFINED__
typedef interface IVisualGestureBuilderFrameReader IVisualGestureBuilderFrameReader;

#endif 	/* __IVisualGestureBuilderFrameReader_FWD_DEFINED__ */


#ifndef __IVisualGestureBuilderFrameReference_FWD_DEFINED__
#define __IVisualGestureBuilderFrameReference_FWD_DEFINED__
typedef interface IVisualGestureBuilderFrameReference IVisualGestureBuilderFrameReference;

#endif 	/* __IVisualGestureBuilderFrameReference_FWD_DEFINED__ */


#ifndef __IVisualGestureBuilderFrameArrivedEventArgs_FWD_DEFINED__
#define __IVisualGestureBuilderFrameArrivedEventArgs_FWD_DEFINED__
typedef interface IVisualGestureBuilderFrameArrivedEventArgs IVisualGestureBuilderFrameArrivedEventArgs;

#endif 	/* __IVisualGestureBuilderFrameArrivedEventArgs_FWD_DEFINED__ */


#ifndef __IVisualGestureBuilderFrame_FWD_DEFINED__
#define __IVisualGestureBuilderFrame_FWD_DEFINED__
typedef interface IVisualGestureBuilderFrame IVisualGestureBuilderFrame;

#endif 	/* __IVisualGestureBuilderFrame_FWD_DEFINED__ */


/* header files for imported files */
#include "oaidl.h"
#include "ocidl.h"
#include "mmreg.h"
#include "Kinect.h"

#ifdef __cplusplus
extern "C"{
#endif 


/* interface __MIDL_itf_Kinect2EVisualGestureBuilder_0000_0000 */
/* [local] */ 


#ifndef _GestureType_
#define _GestureType_
typedef enum _GestureType GestureType;


enum _GestureType
    {
        GestureType_None	= 0,
        GestureType_Discrete	= 1,
        GestureType_Continuous	= 2
    } ;
#endif // _GestureType_













extern RPC_IF_HANDLE __MIDL_itf_Kinect2EVisualGestureBuilder_0000_0000_v0_0_c_ifspec;
extern RPC_IF_HANDLE __MIDL_itf_Kinect2EVisualGestureBuilder_0000_0000_v0_0_s_ifspec;

#ifndef __ITrackingIdLostEventArgs_INTERFACE_DEFINED__
#define __ITrackingIdLostEventArgs_INTERFACE_DEFINED__

/* interface ITrackingIdLostEventArgs */
/* [object][local][uuid] */ 


EXTERN_C const IID IID_ITrackingIdLostEventArgs;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("288C0678-BC9E-4A05-8005-7814601B4338")
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


#ifndef __IGesture_INTERFACE_DEFINED__
#define __IGesture_INTERFACE_DEFINED__

/* interface IGesture */
/* [object][local][uuid] */ 


EXTERN_C const IID IID_IGesture;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("DFD279E6-D8BB-49C1-A61B-E4017D22B412")
    IGesture : public IUnknown
    {
    public:
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_Name( 
            UINT bufferSize,
            /* [annotation][out][retval] */ 
            _Out_writes_z_(bufferSize)  wchar_t *filePath) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_GestureType( 
            /* [annotation][out][retval] */ 
            _Out_  GestureType *gestureType) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct IGestureVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            IGesture * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            IGesture * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            IGesture * This);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_Name )( 
            IGesture * This,
            UINT bufferSize,
            /* [annotation][out][retval] */ 
            _Out_writes_z_(bufferSize)  wchar_t *filePath);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_GestureType )( 
            IGesture * This,
            /* [annotation][out][retval] */ 
            _Out_  GestureType *gestureType);
        
        END_INTERFACE
    } IGestureVtbl;

    interface IGesture
    {
        CONST_VTBL struct IGestureVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define IGesture_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define IGesture_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define IGesture_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define IGesture_get_Name(This,bufferSize,filePath)	\
    ( (This)->lpVtbl -> get_Name(This,bufferSize,filePath) ) 

#define IGesture_get_GestureType(This,gestureType)	\
    ( (This)->lpVtbl -> get_GestureType(This,gestureType) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __IGesture_INTERFACE_DEFINED__ */


#ifndef __IDiscreteGestureResult_INTERFACE_DEFINED__
#define __IDiscreteGestureResult_INTERFACE_DEFINED__

/* interface IDiscreteGestureResult */
/* [object][local][uuid] */ 


EXTERN_C const IID IID_IDiscreteGestureResult;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("5807207E-3BA7-489B-A9D9-5B91521627FF")
    IDiscreteGestureResult : public IUnknown
    {
    public:
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_Detected( 
            /* [annotation][out][retval] */ 
            _Out_  BOOLEAN *detected) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_Confidence( 
            /* [annotation][out][retval] */ 
            _Out_  float *confidence) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_FirstFrameDetected( 
            /* [annotation][out][retval] */ 
            _Out_  BOOLEAN *firstFrameDetected) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct IDiscreteGestureResultVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            IDiscreteGestureResult * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            IDiscreteGestureResult * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            IDiscreteGestureResult * This);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_Detected )( 
            IDiscreteGestureResult * This,
            /* [annotation][out][retval] */ 
            _Out_  BOOLEAN *detected);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_Confidence )( 
            IDiscreteGestureResult * This,
            /* [annotation][out][retval] */ 
            _Out_  float *confidence);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_FirstFrameDetected )( 
            IDiscreteGestureResult * This,
            /* [annotation][out][retval] */ 
            _Out_  BOOLEAN *firstFrameDetected);
        
        END_INTERFACE
    } IDiscreteGestureResultVtbl;

    interface IDiscreteGestureResult
    {
        CONST_VTBL struct IDiscreteGestureResultVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define IDiscreteGestureResult_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define IDiscreteGestureResult_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define IDiscreteGestureResult_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define IDiscreteGestureResult_get_Detected(This,detected)	\
    ( (This)->lpVtbl -> get_Detected(This,detected) ) 

#define IDiscreteGestureResult_get_Confidence(This,confidence)	\
    ( (This)->lpVtbl -> get_Confidence(This,confidence) ) 

#define IDiscreteGestureResult_get_FirstFrameDetected(This,firstFrameDetected)	\
    ( (This)->lpVtbl -> get_FirstFrameDetected(This,firstFrameDetected) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __IDiscreteGestureResult_INTERFACE_DEFINED__ */


#ifndef __IContinuousGestureResult_INTERFACE_DEFINED__
#define __IContinuousGestureResult_INTERFACE_DEFINED__

/* interface IContinuousGestureResult */
/* [object][local][uuid] */ 


EXTERN_C const IID IID_IContinuousGestureResult;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("074499A2-7144-4E92-8772-B233BF91F3A2")
    IContinuousGestureResult : public IUnknown
    {
    public:
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_Progress( 
            /* [annotation][out][retval] */ 
            _Out_  float *progress) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct IContinuousGestureResultVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            IContinuousGestureResult * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            IContinuousGestureResult * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            IContinuousGestureResult * This);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_Progress )( 
            IContinuousGestureResult * This,
            /* [annotation][out][retval] */ 
            _Out_  float *progress);
        
        END_INTERFACE
    } IContinuousGestureResultVtbl;

    interface IContinuousGestureResult
    {
        CONST_VTBL struct IContinuousGestureResultVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define IContinuousGestureResult_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define IContinuousGestureResult_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define IContinuousGestureResult_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define IContinuousGestureResult_get_Progress(This,progress)	\
    ( (This)->lpVtbl -> get_Progress(This,progress) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __IContinuousGestureResult_INTERFACE_DEFINED__ */


#ifndef __IVisualGestureBuilderDatabase_INTERFACE_DEFINED__
#define __IVisualGestureBuilderDatabase_INTERFACE_DEFINED__

/* interface IVisualGestureBuilderDatabase */
/* [object][local][uuid] */ 


EXTERN_C const IID IID_IVisualGestureBuilderDatabase;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("7FA8E82E-E43E-4DD6-A481-1E967DC4B7C8")
    IVisualGestureBuilderDatabase : public IUnknown
    {
    public:
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_AvailableGesturesCount( 
            /* [annotation][out][retval] */ 
            _Out_  UINT *numGestures) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_AvailableGestures( 
            /* [annotation][in] */ 
            _In_  UINT32 capacity,
            /* [annotation][size_is][out] */ 
            _Out_writes_all_(capacity)  IGesture **availableGestures) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct IVisualGestureBuilderDatabaseVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            IVisualGestureBuilderDatabase * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            IVisualGestureBuilderDatabase * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            IVisualGestureBuilderDatabase * This);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_AvailableGesturesCount )( 
            IVisualGestureBuilderDatabase * This,
            /* [annotation][out][retval] */ 
            _Out_  UINT *numGestures);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_AvailableGestures )( 
            IVisualGestureBuilderDatabase * This,
            /* [annotation][in] */ 
            _In_  UINT32 capacity,
            /* [annotation][size_is][out] */ 
            _Out_writes_all_(capacity)  IGesture **availableGestures);
        
        END_INTERFACE
    } IVisualGestureBuilderDatabaseVtbl;

    interface IVisualGestureBuilderDatabase
    {
        CONST_VTBL struct IVisualGestureBuilderDatabaseVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define IVisualGestureBuilderDatabase_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define IVisualGestureBuilderDatabase_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define IVisualGestureBuilderDatabase_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define IVisualGestureBuilderDatabase_get_AvailableGesturesCount(This,numGestures)	\
    ( (This)->lpVtbl -> get_AvailableGesturesCount(This,numGestures) ) 

#define IVisualGestureBuilderDatabase_get_AvailableGestures(This,capacity,availableGestures)	\
    ( (This)->lpVtbl -> get_AvailableGestures(This,capacity,availableGestures) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __IVisualGestureBuilderDatabase_INTERFACE_DEFINED__ */


#ifndef __IVisualGestureBuilderFrameSource_INTERFACE_DEFINED__
#define __IVisualGestureBuilderFrameSource_INTERFACE_DEFINED__

/* interface IVisualGestureBuilderFrameSource */
/* [object][local][uuid] */ 


EXTERN_C const IID IID_IVisualGestureBuilderFrameSource;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("51EB9645-BFED-4625-B95E-E8ECF7F16B15")
    IVisualGestureBuilderFrameSource : public IUnknown
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
            _Out_  BOOLEAN *isValid) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_HorizontalMirror( 
            /* [annotation][out][retval] */ 
            _Out_  BOOLEAN *horizontalMirror) = 0;
        
        virtual /* [propput] */ HRESULT STDMETHODCALLTYPE put_HorizontalMirror( 
            BOOLEAN horizontalMirror) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_GestureCount( 
            /* [annotation][out][retval] */ 
            _Out_  UINT *numGestures) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_Gestures( 
            /* [annotation][in] */ 
            _In_  UINT capacity,
            /* [annotation][size_is][out][in] */ 
            _Inout_updates_all_(capacity)  IGesture **gestures) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE AddGesture( 
            /* [annotation][in] */ 
            _In_  IGesture *gesture) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE AddGestures( 
            /* [annotation][in] */ 
            _In_  UINT capacity,
            /* [annotation][size_is][in] */ 
            _In_reads_(capacity)  IGesture **gestures) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE RemoveGesture( 
            /* [annotation][in] */ 
            _In_  IGesture *gesture) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE GetIsEnabled( 
            /* [annotation][in] */ 
            _In_  IGesture *gesture,
            /* [annotation][out] */ 
            _Out_  BOOLEAN *isEnabled) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE SetIsEnabled( 
            /* [annotation][in] */ 
            _In_  IGesture *gesture,
            BOOLEAN isEnabled) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE OpenReader( 
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IVisualGestureBuilderFrameReader **reader) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_KinectSensor( 
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IKinectSensor **sensor) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct IVisualGestureBuilderFrameSourceVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            IVisualGestureBuilderFrameSource * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            IVisualGestureBuilderFrameSource * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            IVisualGestureBuilderFrameSource * This);
        
        HRESULT ( STDMETHODCALLTYPE *SubscribeTrackingIdLost )( 
            IVisualGestureBuilderFrameSource * This,
            /* [annotation][out][retval] */ 
            _Out_  WAITABLE_HANDLE *waitableHandle);
        
        HRESULT ( STDMETHODCALLTYPE *UnsubscribeTrackingIdLost )( 
            IVisualGestureBuilderFrameSource * This,
            /* [annotation][in] */ 
            _In_  WAITABLE_HANDLE waitableHandle);
        
        HRESULT ( STDMETHODCALLTYPE *GetTrackingIdLostEventData )( 
            IVisualGestureBuilderFrameSource * This,
            /* [annotation][in] */ 
            _In_  WAITABLE_HANDLE waitableHandle,
            /* [annotation][out][retval] */ 
            _Out_  ITrackingIdLostEventArgs **eventData);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_IsActive )( 
            IVisualGestureBuilderFrameSource * This,
            /* [annotation][out][retval] */ 
            _Out_  BOOLEAN *isActive);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_TrackingId )( 
            IVisualGestureBuilderFrameSource * This,
            /* [annotation][out][retval] */ 
            _Out_  UINT64 *trackingId);
        
        /* [propput] */ HRESULT ( STDMETHODCALLTYPE *put_TrackingId )( 
            IVisualGestureBuilderFrameSource * This,
            UINT64 trackingId);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_IsTrackingIdValid )( 
            IVisualGestureBuilderFrameSource * This,
            /* [annotation][out][retval] */ 
            _Out_  BOOLEAN *isValid);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_HorizontalMirror )( 
            IVisualGestureBuilderFrameSource * This,
            /* [annotation][out][retval] */ 
            _Out_  BOOLEAN *horizontalMirror);
        
        /* [propput] */ HRESULT ( STDMETHODCALLTYPE *put_HorizontalMirror )( 
            IVisualGestureBuilderFrameSource * This,
            BOOLEAN horizontalMirror);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_GestureCount )( 
            IVisualGestureBuilderFrameSource * This,
            /* [annotation][out][retval] */ 
            _Out_  UINT *numGestures);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_Gestures )( 
            IVisualGestureBuilderFrameSource * This,
            /* [annotation][in] */ 
            _In_  UINT capacity,
            /* [annotation][size_is][out][in] */ 
            _Inout_updates_all_(capacity)  IGesture **gestures);
        
        HRESULT ( STDMETHODCALLTYPE *AddGesture )( 
            IVisualGestureBuilderFrameSource * This,
            /* [annotation][in] */ 
            _In_  IGesture *gesture);
        
        HRESULT ( STDMETHODCALLTYPE *AddGestures )( 
            IVisualGestureBuilderFrameSource * This,
            /* [annotation][in] */ 
            _In_  UINT capacity,
            /* [annotation][size_is][in] */ 
            _In_reads_(capacity)  IGesture **gestures);
        
        HRESULT ( STDMETHODCALLTYPE *RemoveGesture )( 
            IVisualGestureBuilderFrameSource * This,
            /* [annotation][in] */ 
            _In_  IGesture *gesture);
        
        HRESULT ( STDMETHODCALLTYPE *GetIsEnabled )( 
            IVisualGestureBuilderFrameSource * This,
            /* [annotation][in] */ 
            _In_  IGesture *gesture,
            /* [annotation][out] */ 
            _Out_  BOOLEAN *isEnabled);
        
        HRESULT ( STDMETHODCALLTYPE *SetIsEnabled )( 
            IVisualGestureBuilderFrameSource * This,
            /* [annotation][in] */ 
            _In_  IGesture *gesture,
            BOOLEAN isEnabled);
        
        HRESULT ( STDMETHODCALLTYPE *OpenReader )( 
            IVisualGestureBuilderFrameSource * This,
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IVisualGestureBuilderFrameReader **reader);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_KinectSensor )( 
            IVisualGestureBuilderFrameSource * This,
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IKinectSensor **sensor);
        
        END_INTERFACE
    } IVisualGestureBuilderFrameSourceVtbl;

    interface IVisualGestureBuilderFrameSource
    {
        CONST_VTBL struct IVisualGestureBuilderFrameSourceVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define IVisualGestureBuilderFrameSource_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define IVisualGestureBuilderFrameSource_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define IVisualGestureBuilderFrameSource_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define IVisualGestureBuilderFrameSource_SubscribeTrackingIdLost(This,waitableHandle)	\
    ( (This)->lpVtbl -> SubscribeTrackingIdLost(This,waitableHandle) ) 

#define IVisualGestureBuilderFrameSource_UnsubscribeTrackingIdLost(This,waitableHandle)	\
    ( (This)->lpVtbl -> UnsubscribeTrackingIdLost(This,waitableHandle) ) 

#define IVisualGestureBuilderFrameSource_GetTrackingIdLostEventData(This,waitableHandle,eventData)	\
    ( (This)->lpVtbl -> GetTrackingIdLostEventData(This,waitableHandle,eventData) ) 

#define IVisualGestureBuilderFrameSource_get_IsActive(This,isActive)	\
    ( (This)->lpVtbl -> get_IsActive(This,isActive) ) 

#define IVisualGestureBuilderFrameSource_get_TrackingId(This,trackingId)	\
    ( (This)->lpVtbl -> get_TrackingId(This,trackingId) ) 

#define IVisualGestureBuilderFrameSource_put_TrackingId(This,trackingId)	\
    ( (This)->lpVtbl -> put_TrackingId(This,trackingId) ) 

#define IVisualGestureBuilderFrameSource_get_IsTrackingIdValid(This,isValid)	\
    ( (This)->lpVtbl -> get_IsTrackingIdValid(This,isValid) ) 

#define IVisualGestureBuilderFrameSource_get_HorizontalMirror(This,horizontalMirror)	\
    ( (This)->lpVtbl -> get_HorizontalMirror(This,horizontalMirror) ) 

#define IVisualGestureBuilderFrameSource_put_HorizontalMirror(This,horizontalMirror)	\
    ( (This)->lpVtbl -> put_HorizontalMirror(This,horizontalMirror) ) 

#define IVisualGestureBuilderFrameSource_get_GestureCount(This,numGestures)	\
    ( (This)->lpVtbl -> get_GestureCount(This,numGestures) ) 

#define IVisualGestureBuilderFrameSource_get_Gestures(This,capacity,gestures)	\
    ( (This)->lpVtbl -> get_Gestures(This,capacity,gestures) ) 

#define IVisualGestureBuilderFrameSource_AddGesture(This,gesture)	\
    ( (This)->lpVtbl -> AddGesture(This,gesture) ) 

#define IVisualGestureBuilderFrameSource_AddGestures(This,capacity,gestures)	\
    ( (This)->lpVtbl -> AddGestures(This,capacity,gestures) ) 

#define IVisualGestureBuilderFrameSource_RemoveGesture(This,gesture)	\
    ( (This)->lpVtbl -> RemoveGesture(This,gesture) ) 

#define IVisualGestureBuilderFrameSource_GetIsEnabled(This,gesture,isEnabled)	\
    ( (This)->lpVtbl -> GetIsEnabled(This,gesture,isEnabled) ) 

#define IVisualGestureBuilderFrameSource_SetIsEnabled(This,gesture,isEnabled)	\
    ( (This)->lpVtbl -> SetIsEnabled(This,gesture,isEnabled) ) 

#define IVisualGestureBuilderFrameSource_OpenReader(This,reader)	\
    ( (This)->lpVtbl -> OpenReader(This,reader) ) 

#define IVisualGestureBuilderFrameSource_get_KinectSensor(This,sensor)	\
    ( (This)->lpVtbl -> get_KinectSensor(This,sensor) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __IVisualGestureBuilderFrameSource_INTERFACE_DEFINED__ */


#ifndef __IVisualGestureBuilderFrameReader_INTERFACE_DEFINED__
#define __IVisualGestureBuilderFrameReader_INTERFACE_DEFINED__

/* interface IVisualGestureBuilderFrameReader */
/* [object][local][uuid] */ 


EXTERN_C const IID IID_IVisualGestureBuilderFrameReader;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("9EC0C4CD-3CB0-4ED6-ABF9-7C7389F852F9")
    IVisualGestureBuilderFrameReader : public IUnknown
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
            _Out_  IVisualGestureBuilderFrameArrivedEventArgs **eventData) = 0;
        
        virtual HRESULT STDMETHODCALLTYPE CalculateAndAcquireLatestFrame( 
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IVisualGestureBuilderFrame **gestureFrame) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_IsPaused( 
            /* [annotation][out][retval] */ 
            _Out_  BOOLEAN *isPaused) = 0;
        
        virtual /* [propput] */ HRESULT STDMETHODCALLTYPE put_IsPaused( 
            BOOLEAN isPaused) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_VisualGestureBuilderFrameSource( 
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IVisualGestureBuilderFrameSource **userGestureFrameSource) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct IVisualGestureBuilderFrameReaderVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            IVisualGestureBuilderFrameReader * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            IVisualGestureBuilderFrameReader * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            IVisualGestureBuilderFrameReader * This);
        
        HRESULT ( STDMETHODCALLTYPE *SubscribeFrameArrived )( 
            IVisualGestureBuilderFrameReader * This,
            /* [annotation][out][retval] */ 
            _Out_  WAITABLE_HANDLE *waitableHandle);
        
        HRESULT ( STDMETHODCALLTYPE *UnsubscribeFrameArrived )( 
            IVisualGestureBuilderFrameReader * This,
            /* [annotation][in] */ 
            _In_  WAITABLE_HANDLE waitableHandle);
        
        HRESULT ( STDMETHODCALLTYPE *GetFrameArrivedEventData )( 
            IVisualGestureBuilderFrameReader * This,
            /* [annotation][in] */ 
            _In_  WAITABLE_HANDLE waitableHandle,
            /* [annotation][out][retval] */ 
            _Out_  IVisualGestureBuilderFrameArrivedEventArgs **eventData);
        
        HRESULT ( STDMETHODCALLTYPE *CalculateAndAcquireLatestFrame )( 
            IVisualGestureBuilderFrameReader * This,
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IVisualGestureBuilderFrame **gestureFrame);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_IsPaused )( 
            IVisualGestureBuilderFrameReader * This,
            /* [annotation][out][retval] */ 
            _Out_  BOOLEAN *isPaused);
        
        /* [propput] */ HRESULT ( STDMETHODCALLTYPE *put_IsPaused )( 
            IVisualGestureBuilderFrameReader * This,
            BOOLEAN isPaused);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_VisualGestureBuilderFrameSource )( 
            IVisualGestureBuilderFrameReader * This,
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IVisualGestureBuilderFrameSource **userGestureFrameSource);
        
        END_INTERFACE
    } IVisualGestureBuilderFrameReaderVtbl;

    interface IVisualGestureBuilderFrameReader
    {
        CONST_VTBL struct IVisualGestureBuilderFrameReaderVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define IVisualGestureBuilderFrameReader_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define IVisualGestureBuilderFrameReader_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define IVisualGestureBuilderFrameReader_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define IVisualGestureBuilderFrameReader_SubscribeFrameArrived(This,waitableHandle)	\
    ( (This)->lpVtbl -> SubscribeFrameArrived(This,waitableHandle) ) 

#define IVisualGestureBuilderFrameReader_UnsubscribeFrameArrived(This,waitableHandle)	\
    ( (This)->lpVtbl -> UnsubscribeFrameArrived(This,waitableHandle) ) 

#define IVisualGestureBuilderFrameReader_GetFrameArrivedEventData(This,waitableHandle,eventData)	\
    ( (This)->lpVtbl -> GetFrameArrivedEventData(This,waitableHandle,eventData) ) 

#define IVisualGestureBuilderFrameReader_CalculateAndAcquireLatestFrame(This,gestureFrame)	\
    ( (This)->lpVtbl -> CalculateAndAcquireLatestFrame(This,gestureFrame) ) 

#define IVisualGestureBuilderFrameReader_get_IsPaused(This,isPaused)	\
    ( (This)->lpVtbl -> get_IsPaused(This,isPaused) ) 

#define IVisualGestureBuilderFrameReader_put_IsPaused(This,isPaused)	\
    ( (This)->lpVtbl -> put_IsPaused(This,isPaused) ) 

#define IVisualGestureBuilderFrameReader_get_VisualGestureBuilderFrameSource(This,userGestureFrameSource)	\
    ( (This)->lpVtbl -> get_VisualGestureBuilderFrameSource(This,userGestureFrameSource) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __IVisualGestureBuilderFrameReader_INTERFACE_DEFINED__ */


#ifndef __IVisualGestureBuilderFrameReference_INTERFACE_DEFINED__
#define __IVisualGestureBuilderFrameReference_INTERFACE_DEFINED__

/* interface IVisualGestureBuilderFrameReference */
/* [object][local][uuid] */ 


EXTERN_C const IID IID_IVisualGestureBuilderFrameReference;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("2393DACC-2182-491F-A140-484F4218286D")
    IVisualGestureBuilderFrameReference : public IUnknown
    {
    public:
        virtual HRESULT STDMETHODCALLTYPE AcquireFrame( 
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IVisualGestureBuilderFrame **gestureFrame) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_RelativeTime( 
            /* [annotation][out][retval] */ 
            _Out_  TIMESPAN *relativeTime) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct IVisualGestureBuilderFrameReferenceVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            IVisualGestureBuilderFrameReference * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            IVisualGestureBuilderFrameReference * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            IVisualGestureBuilderFrameReference * This);
        
        HRESULT ( STDMETHODCALLTYPE *AcquireFrame )( 
            IVisualGestureBuilderFrameReference * This,
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IVisualGestureBuilderFrame **gestureFrame);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_RelativeTime )( 
            IVisualGestureBuilderFrameReference * This,
            /* [annotation][out][retval] */ 
            _Out_  TIMESPAN *relativeTime);
        
        END_INTERFACE
    } IVisualGestureBuilderFrameReferenceVtbl;

    interface IVisualGestureBuilderFrameReference
    {
        CONST_VTBL struct IVisualGestureBuilderFrameReferenceVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define IVisualGestureBuilderFrameReference_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define IVisualGestureBuilderFrameReference_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define IVisualGestureBuilderFrameReference_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define IVisualGestureBuilderFrameReference_AcquireFrame(This,gestureFrame)	\
    ( (This)->lpVtbl -> AcquireFrame(This,gestureFrame) ) 

#define IVisualGestureBuilderFrameReference_get_RelativeTime(This,relativeTime)	\
    ( (This)->lpVtbl -> get_RelativeTime(This,relativeTime) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __IVisualGestureBuilderFrameReference_INTERFACE_DEFINED__ */


#ifndef __IVisualGestureBuilderFrameArrivedEventArgs_INTERFACE_DEFINED__
#define __IVisualGestureBuilderFrameArrivedEventArgs_INTERFACE_DEFINED__

/* interface IVisualGestureBuilderFrameArrivedEventArgs */
/* [object][local][uuid] */ 


EXTERN_C const IID IID_IVisualGestureBuilderFrameArrivedEventArgs;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("FBF3F74C-D814-4CD4-BC2E-C7BD0A4BF362")
    IVisualGestureBuilderFrameArrivedEventArgs : public IUnknown
    {
    public:
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_FrameReference( 
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IVisualGestureBuilderFrameReference **gestureFrameReference) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct IVisualGestureBuilderFrameArrivedEventArgsVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            IVisualGestureBuilderFrameArrivedEventArgs * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            IVisualGestureBuilderFrameArrivedEventArgs * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            IVisualGestureBuilderFrameArrivedEventArgs * This);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_FrameReference )( 
            IVisualGestureBuilderFrameArrivedEventArgs * This,
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IVisualGestureBuilderFrameReference **gestureFrameReference);
        
        END_INTERFACE
    } IVisualGestureBuilderFrameArrivedEventArgsVtbl;

    interface IVisualGestureBuilderFrameArrivedEventArgs
    {
        CONST_VTBL struct IVisualGestureBuilderFrameArrivedEventArgsVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define IVisualGestureBuilderFrameArrivedEventArgs_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define IVisualGestureBuilderFrameArrivedEventArgs_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define IVisualGestureBuilderFrameArrivedEventArgs_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define IVisualGestureBuilderFrameArrivedEventArgs_get_FrameReference(This,gestureFrameReference)	\
    ( (This)->lpVtbl -> get_FrameReference(This,gestureFrameReference) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __IVisualGestureBuilderFrameArrivedEventArgs_INTERFACE_DEFINED__ */


#ifndef __IVisualGestureBuilderFrame_INTERFACE_DEFINED__
#define __IVisualGestureBuilderFrame_INTERFACE_DEFINED__

/* interface IVisualGestureBuilderFrame */
/* [object][local][uuid] */ 


EXTERN_C const IID IID_IVisualGestureBuilderFrame;

#if defined(__cplusplus) && !defined(CINTERFACE)
    
    MIDL_INTERFACE("0C00BA06-C6F2-4F75-AA12-EDDBA2018AB8")
    IVisualGestureBuilderFrame : public IUnknown
    {
    public:
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_ContinuousGestureResult( 
            /* [annotation][in] */ 
            _In_  IGesture *gesture,
            /* [annotation][out][retval] */ 
            _COM_Outptr_result_maybenull_  IContinuousGestureResult **result) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_DiscreteGestureResult( 
            /* [annotation][in] */ 
            _In_  IGesture *gesture,
            /* [annotation][out][retval] */ 
            _COM_Outptr_result_maybenull_  IDiscreteGestureResult **result) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_TrackingId( 
            /* [annotation][out][retval] */ 
            _Out_  UINT64 *trackingId) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_IsTrackingIdValid( 
            /* [annotation][out][retval] */ 
            _Out_  BOOLEAN *isTrackingIdValid) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_RelativeTime( 
            /* [annotation][out][retval] */ 
            _Out_  TIMESPAN *relativeTime) = 0;
        
        virtual /* [propget] */ HRESULT STDMETHODCALLTYPE get_VisualGestureBuilderFrameSource( 
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IVisualGestureBuilderFrameSource **userGestureFrameSource) = 0;
        
    };
    
    
#else 	/* C style interface */

    typedef struct IVisualGestureBuilderFrameVtbl
    {
        BEGIN_INTERFACE
        
        HRESULT ( STDMETHODCALLTYPE *QueryInterface )( 
            IVisualGestureBuilderFrame * This,
            /* [in] */ REFIID riid,
            /* [annotation][iid_is][out] */ 
            _COM_Outptr_  void **ppvObject);
        
        ULONG ( STDMETHODCALLTYPE *AddRef )( 
            IVisualGestureBuilderFrame * This);
        
        ULONG ( STDMETHODCALLTYPE *Release )( 
            IVisualGestureBuilderFrame * This);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_ContinuousGestureResult )( 
            IVisualGestureBuilderFrame * This,
            /* [annotation][in] */ 
            _In_  IGesture *gesture,
            /* [annotation][out][retval] */ 
            _COM_Outptr_result_maybenull_  IContinuousGestureResult **result);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_DiscreteGestureResult )( 
            IVisualGestureBuilderFrame * This,
            /* [annotation][in] */ 
            _In_  IGesture *gesture,
            /* [annotation][out][retval] */ 
            _COM_Outptr_result_maybenull_  IDiscreteGestureResult **result);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_TrackingId )( 
            IVisualGestureBuilderFrame * This,
            /* [annotation][out][retval] */ 
            _Out_  UINT64 *trackingId);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_IsTrackingIdValid )( 
            IVisualGestureBuilderFrame * This,
            /* [annotation][out][retval] */ 
            _Out_  BOOLEAN *isTrackingIdValid);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_RelativeTime )( 
            IVisualGestureBuilderFrame * This,
            /* [annotation][out][retval] */ 
            _Out_  TIMESPAN *relativeTime);
        
        /* [propget] */ HRESULT ( STDMETHODCALLTYPE *get_VisualGestureBuilderFrameSource )( 
            IVisualGestureBuilderFrame * This,
            /* [annotation][out][retval] */ 
            _COM_Outptr_  IVisualGestureBuilderFrameSource **userGestureFrameSource);
        
        END_INTERFACE
    } IVisualGestureBuilderFrameVtbl;

    interface IVisualGestureBuilderFrame
    {
        CONST_VTBL struct IVisualGestureBuilderFrameVtbl *lpVtbl;
    };

    

#ifdef COBJMACROS


#define IVisualGestureBuilderFrame_QueryInterface(This,riid,ppvObject)	\
    ( (This)->lpVtbl -> QueryInterface(This,riid,ppvObject) ) 

#define IVisualGestureBuilderFrame_AddRef(This)	\
    ( (This)->lpVtbl -> AddRef(This) ) 

#define IVisualGestureBuilderFrame_Release(This)	\
    ( (This)->lpVtbl -> Release(This) ) 


#define IVisualGestureBuilderFrame_get_ContinuousGestureResult(This,gesture,result)	\
    ( (This)->lpVtbl -> get_ContinuousGestureResult(This,gesture,result) ) 

#define IVisualGestureBuilderFrame_get_DiscreteGestureResult(This,gesture,result)	\
    ( (This)->lpVtbl -> get_DiscreteGestureResult(This,gesture,result) ) 

#define IVisualGestureBuilderFrame_get_TrackingId(This,trackingId)	\
    ( (This)->lpVtbl -> get_TrackingId(This,trackingId) ) 

#define IVisualGestureBuilderFrame_get_IsTrackingIdValid(This,isTrackingIdValid)	\
    ( (This)->lpVtbl -> get_IsTrackingIdValid(This,isTrackingIdValid) ) 

#define IVisualGestureBuilderFrame_get_RelativeTime(This,relativeTime)	\
    ( (This)->lpVtbl -> get_RelativeTime(This,relativeTime) ) 

#define IVisualGestureBuilderFrame_get_VisualGestureBuilderFrameSource(This,userGestureFrameSource)	\
    ( (This)->lpVtbl -> get_VisualGestureBuilderFrameSource(This,userGestureFrameSource) ) 

#endif /* COBJMACROS */


#endif 	/* C style interface */




#endif 	/* __IVisualGestureBuilderFrame_INTERFACE_DEFINED__ */


/* interface __MIDL_itf_Kinect2EVisualGestureBuilder_0000_0010 */
/* [local] */ 

HRESULT WINAPI CreateVisualGestureBuilderFrameSource(_In_ IKinectSensor* sensor, _In_ UINT64 initialTrackingId, _COM_Outptr_ IVisualGestureBuilderFrameSource** source);
HRESULT WINAPI CreateVisualGestureBuilderDatabaseInstanceFromFile(_In_z_ const wchar_t* name, _COM_Outptr_ IVisualGestureBuilderDatabase** instance);
HRESULT WINAPI CreateVisualGestureBuilderDatabaseInstanceFromMemory(UINT capacity, _In_reads_(capacity) BYTE* buffer, _COM_Outptr_ IVisualGestureBuilderDatabase** instance);


extern RPC_IF_HANDLE __MIDL_itf_Kinect2EVisualGestureBuilder_0000_0010_v0_0_c_ifspec;
extern RPC_IF_HANDLE __MIDL_itf_Kinect2EVisualGestureBuilder_0000_0010_v0_0_s_ifspec;

/* Additional Prototypes for ALL interfaces */

/* end of Additional Prototypes */

#ifdef __cplusplus
}
#endif

#endif


