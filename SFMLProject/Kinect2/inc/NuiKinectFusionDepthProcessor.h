///------------------------------------------------------------------------------
/// <copyright file="NuiKinectFusionDepthProcessor.h" company="Microsoft">
///    Copyright (c) Microsoft Corporation. All rights reserved.
/// </copyright>
///------------------------------------------------------------------------------
/// This module defines the APIs for Kinect Fusion depth processing.

#pragma once

#ifndef FUSIONAPI
#error "You must include NuiKinectFusionApi.h rather than including NuiKinectFusionDepthProcessor.h directly"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include "NuiKinectFusionVolume.h"

// Include the KinectSDK structure definitions
#include <Kinect.h>

#define NUI_DEPTH_RAW_WIDTH             512
#define NUI_DEPTH_RAW_HEIGHT            424

// Default Kinect intrinsic camera parameters, normalized by image size, 
// based on Kinect for Windows SDK values
// @TODO: [evgenysa] convert all defines as consts
#define NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_X (0.72113f)
#define NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_Y (0.870799f)
#define NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_X (0.50602675f)
#define NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_Y (0.499133f)


// Default processing parameters
#define NUI_FUSION_DEFAULT_MINIMUM_DEPTH (0.5f)
#define NUI_FUSION_DEFAULT_MAXIMUM_DEPTH (8.0f)

#define NUI_FUSION_DEFAULT_SMOOTHING_KERNEL_WIDTH (2)
#define NUI_FUSION_DEFAULT_SMOOTHING_DISTANCE_THRESHOLD (0.04f)

#define NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT (20)

#define NUI_FUSION_DEFAULT_INTEGRATION_WEIGHT (200)

#define NUI_FUSION_DEFAULT_COLOR_INTEGRATION_OF_ALL_ANGLES (180.0f) // Integrate +/- 180 degrees

/// <summary>
/// Create an image frame for use with Kinect Fusion with a specified data type and resolution.
/// Note that image must be a minimum of 32 pixels in both width and height, and for camera tracking 
/// Align functions and volume Integration, use of the default camera parameters is only supported
/// with 4:3 pixel aspect ratio images such as uniformly scaled versions of the source Kinect image  
/// (e.g. 160x120,320x240,640x480 etc.). To crop to smaller non 4:3 ratios and still use the default
/// camera parameters set unwanted pixels to 0 depth, which will be ignored in processing, or 
/// alternately, the user can supply their own calibration with an arbitrary sized image. For example,
/// a user supplied set of parameters can be used when calling CalculatePointCloud to calculate a 
/// large image of the reconstruction at the UI window resolution (perhaps with a virtual viewpoint 
/// different to the Kinect camera or a non 4:3 aspect image ratio) by then subsequently calling 
/// ShadePointCloud and rendering the resulting images on screen.
/// </summary>
/// <param name="frameType">The frame type to create.</param>
/// <param name="width">The frame width to create, in pixels. Minimum width is 32 pixels.</param>
/// <param name="height">The frame height to create, in pixels. Minimum height is 32 pixels.</param>
/// <param name="pCameraParameters">Optionally, a pointer to a pre-allocated set of camera 
/// parameters with values normalized by the camera image size in their respective axis. 
/// If set to NULL the standard Kinect camera parameters are used.</param>
/// <param name="ppImageFrame">A pointer that receives a reference to the created image frame.</param>
/// <returns>
/// Returns S_OK if successful; otherwise, returns one of the following failure codes:
/// <list type="table">
///    <listheader>
///       <term>Error code</term>
///       <description>Description</description>
///    </listheader>
///    <item>
///       <term>E_INVALIDARG</term>
///       <description>The <paramref name="frameType"/> parameter is invalid,
///       the <paramref name="width"/> or <paramref name="height"/> parameter is less than 32,
///       or the <paramref name="pCameraParameters"/> parameter's <c>focalLengthX"</c> or 
///       <c>focalLengthX"</c> member is 0 or negative.
///       </description>
///    </item>
///    <item>
///       <term>E_POINTER</term>
///       <description>The <paramref name="ppImageFrame"/> parameter is NULL.</description>
///    </item>
///    <item>
///       <term>E_OUTOFMEMORY</term>
///       <description>The <paramref name="frameType"/> and <paramref name="frameResolution"/> 
///       parameter combination specifies a frame too big, memory allocation failed.</description>
///    </item>
///    <item>
///       <term>E_FAIL</term>
///       <description>The call failed for an unknown reason.</description>
///    </item>
/// </list>
/// </returns>
_Check_return_ HRESULT FUSIONAPI NuiFusionCreateImageFrame(
    _In_ NUI_FUSION_IMAGE_TYPE frameType,
    _In_ UINT width,
    _In_ UINT height,
    _In_opt_ const NUI_FUSION_CAMERA_PARAMETERS *pCameraParameters,
    _Deref_out_ NUI_FUSION_IMAGE_FRAME **ppImageFrame
    );

/// <summary>
/// Releases the specified frame of data.
/// </summary>
/// <param name="pImageFrame">A pointer to the frame to release.</param>
/// <returns>
/// Returns S_OK if successful; otherwise, returns one of the following failure codes:
/// <list type="table">
///    <listheader>
///       <term>Error code</term>
///       <description>Description</description>
///    </listheader>
///    <item>
///       <term>E_INVALIDARG</term>
///       <description>The <paramref name="pImageFrame"/> parameter is NULL.</description>
///    </item>
///    <item>
///       <term>E_NOINTERFACE</term>
///       <description>The <paramref name="pImageFrame"/> parameter's <c>pFrameBuffer</c> member
///       is NULL.</description>
///    </item>
///    <item>
///       <term>E_FAIL</term>
///       <description>The call failed for an unknown reason.</description>
///    </item>
/// </list>
/// </returns>
_Check_return_ HRESULT FUSIONAPI NuiFusionReleaseImageFrame(
    _In_ NUI_FUSION_IMAGE_FRAME *pImageFrame
    );

/// <summary>
/// Converts Kinect depth frames in unsigned short format to depth frames in float format 
/// representing distance from the camera in meters (parallel to the optical center axis).
/// Note: <paramref name="pDepthImageData"/> and <paramref name="pDepthFloatFrame"/> must
/// be the same pixel resolution and equal to <paramref name="depthImageDataWidth"/> by
/// <paramref name="depthImageDataHeight"/>.
/// </summary>
/// <param name="pDepthImageData">A pointer to the extended-depth texture of a NUI_IMAGE_FRAME 
/// depth image from the Kinect camera.</param>
/// <param name="depthImageDataWidth">Width of the depth image data.</param>
/// <param name="depthImageDataHeight">Height of the depth image data.</param>
/// <param name="pDepthFloatFrame">A pointer to a pre-allocated depth float type frame.</param> 
/// <param name="minDepthClip">Minimum depth distance threshold in meters. Depth pixels below this
//// value will be returned as invalid (0). Minimum depth must be 0 or greater.</param>
/// <param name="maxDepthClip">Maximum depth distance threshold in meters. Depth pixels above this 
/// value will be returned as invalid (1000). Maximum depth must be greater than 0.</param>
/// <param name="mirrorDepth">A boolean parameter specifying whether to horizontally mirror the 
/// input depth image.</param> 
/// <returns>
/// Returns S_OK if successful; otherwise, returns one of the following failure codes:
/// <list type="table">
///    <listheader>
///       <term>Error code</term>
///       <description>Description</description>
///    </listheader>
///    <item>
///       <term>E_INVALIDARG</term>
///       <description>The <paramref name="pDepthImageData"/> or <paramref name="pDepthFloatFrame"/>
///       parameter is NULL, 
///       the <paramref name="minDepthClip"/> parameter or <paramref name="maxDepthClip"/> is less 
///       than zero,</description>
///       or the <paramref name="pDepthFloatFrame"/> parameter is an incorrect image type,
///       or the <paramref name="depthImageDataWidth"/> parameter and pDepthFloatFrame's 
///       <c>width</c> member is not equal,	
///       or the <paramref name="depthImageDataHeight"/> parameter and pDepthFloatFrame's 
///       <c>height</c> member is not equal.	
///       </description>
///    </item>
///    </item>
///       <term>E_NOINTERFACE</term>
///       <description>The <paramref name="pDepthFloatFrame"/> parameter's <c>pFrameBuffer</c> 
///       member is NULL.</description>
///    </item>
///    <item>
///       <term>E_OUTOFMEMORY</term>
///       <description>A CPU memory allocation failed.</description>
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
/// The min and max depth clip values enable clipping of the input data, for example, to help
/// isolate particular objects or surfaces to be reconstructed. Note that the thresholds return 
/// different values when a depth pixel is outside the threshold - pixels inside minDepthClip will
/// will be returned as 0 and ignored in processing, whereas pixels beyond maxDepthClip will be set
/// to 1000 to signify a valid depth ray with depth beyond the set threshold. Setting this far-
/// distance flag is important for reconstruction integration in situations where the camera is
/// static or does not move significantly, as it enables any voxels closer to the camera
/// along this ray to be culled instead of persisting (as would happen if the pixels were simply 
/// set to 0 and ignored in processing). Note that when reconstructing large real-world size volumes,
/// be sure to set large maxDepthClip distances, as when the camera moves around, any voxels in view
/// which go beyond this threshold distance from the camera will be removed.
/// </remarks>
_Check_return_ HRESULT FUSIONAPI NuiFusionDepthToDepthFloatFrame(
    _In_ const UINT16 *pDepthImageData,
    _In_ UINT depthImageDataWidth,
    _In_ UINT depthImageDataHeight,
    _In_ const NUI_FUSION_IMAGE_FRAME *pDepthFloatFrame,
    _In_ FLOAT minDepthClip,
    _In_ FLOAT maxDepthClip,
    _In_ BOOL mirrorDepth
    );

/// <summary>
/// Construct an oriented point cloud in the local camera frame of reference, from a depth float
/// image frame. Here we calculate the 3D position of each depth float pixel with the optical
/// center of the camera as the origin. We use a right-hand coordinate system, and (in common with
/// bitmap images with top left origin) +X is to the right, +Y down, and +Z is now forward from the
/// Kinect camera into the scene, as though looking into the scene from behind the Kinect camera.
/// Both image frames must be the same size and have the same camera parameters.
/// </summary>
/// <param name="pDepthFloatFrame">A pointer to a pre-allocated depth float frame.</param>
/// <param name="pPointCloudFrame">A pointer to a pre-allocated point cloud frame, to be filled
/// with 3D points and normals.</param>
/// <returns>
/// Returns S_OK if successful; otherwise, returns one of the following failure codes:
/// <list type="table">
///    <listheader>
///       <term>Error code</term>
///       <description>Description</description>
///    </listheader>
///    <item>
///       <term>E_INVALIDARG</term>
///       <description>The <paramref name="pPointCloudFrame"/> or 
///       <paramref name="pDepthFloatFrame"/> parameter is NULL,
///       the <paramref name="pDepthFloatFrame"/> or <paramref name="pPointCloudFrame"/> 
///       parameters are different image sizes,
///       or the <paramref name="pDepthFloatFrame"/> or <paramref name="pPointCloudFrame"/> 
///       parameters are an incorrect image type, 
///       or <paramref name="pDepthFloatFrame"/> or <paramref name="pPointCloudFrame"/> 
///       parameters have different <c>pCameraParameter</c> members.
///       </description>
///    </item>
///    </item>
///       <term>E_NOINTERFACE</term>
///       <description>The <paramref name="pDepthFloatFrame"/> or 
///       <paramref name="pPointCloudFrame"/> parameter's <c>pFrameBuffer</c> member is NULL.
///       </description>
///    </item>
///    <item>
///       <term>E_OUTOFMEMORY</term>
///       <description>A CPU memory allocation failed.</description>
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
_Check_return_ HRESULT FUSIONAPI NuiFusionDepthFloatFrameToPointCloud(
    _In_ const NUI_FUSION_IMAGE_FRAME *pDepthFloatFrame,
    _In_ const NUI_FUSION_IMAGE_FRAME *pPointCloudFrame
    );

/// <summary>
/// Create visible color shaded images of a point cloud and its normals. 
/// All image frames must be the same size and have the same camera parameters.
/// </summary>
/// <param name="pPointCloudFrame">A pointer to a pre-allocated point cloud frame.</param>
/// <param name="pWorldToCameraTransform">A pointer to a pre-allocated Matrix4, filled with the 
/// world to camera transform (camera pose) to be used in lighting calculations. Note that the
/// output from the CalculatePointCloud function raycast is in world coordinates hence the camera
/// transfrom used when calling that function should typically also be passed here. In contrast, if
/// the point cloud originated from NuiFusionDepthFloatFrameToPointCloud it is in the camera local 
/// coordinate system, hence identity should typically be passed here.</param>
/// <param name="pWorldToBGRTransform">Optionally, a pointer to a pre-allocated Matrix4, which contains
/// a mapping from world co-ordinates to an RGB color space. If non-NULL, this transform is used to
/// color surfaces according to their world positions. RGB values are clamped to the range [0, 1].
/// If this parameter is NULL, then a simpler grayscale L.N shading routine is used.</param>
/// <param name="pShadedSurfaceFrame">Optionally, a pointer to a pre-allocated color image frame,
/// to be filled with a color shaded surface image. Pass NULL to skip this image.</param>
/// <param name="pShadedSurfaceNormalsFrame">Optionally, a pointer to a pre-allocated color image
/// frame, to be filled with the color shaded normals image with color indicating orientation. 
/// Pass NULL to skip this image.</param>
/// <returns>
/// Returns S_OK if successful; otherwise, returns one of the following failure codes:
/// <list type="table">
///    <listheader>
///       <term>Error code</term>
///       <description>Description</description>
///    </listheader>
///    <item>
///       <term>E_INVALIDARG</term>
///       <description>The <paramref name="pPointCloudFrame"/> or 
///       <paramref name="pWorldToCameraTransform"/> parameter is NULL,
///       the <paramref name="pPointCloudFrame"/> or <paramref name="pShadedSurfaceFrame"/> 
///       or <paramref name="pShadedSurfaceNormalsFrame"/> parameters are different image sizes,
///       or the <paramref name="pPointCloudFrame"/> or <paramref name="pShadedSurfaceFrame"/>
///       or <paramref name="pShadedSurfaceNormalsFrame"/> parameters are an incorrect image type,
///       or <paramref name="pPointCloudFrame"/> or <paramref name="pShadedSurfaceFrame"/> or
///       <paramref name="pShadedSurfaceNormalsFrame"/> parameters have different 
///       <c>pCameraParameter</c> members.
///       </description>
///    </item>
///    </item>
///       <term>E_NOINTERFACE</term>
///       <description>The <paramref name="pPointCloudFrame"/> or 
///       <paramref name="pShadedSurfaceFrame"/> or <paramref name="pShadedSurfaceNormalsFrame"/>
///       parameter's <c>pFrameBuffer</c> member is NULL.</description>
///    </item>
///    <item>
///       <term>E_OUTOFMEMORY</term>
///       <description>A CPU memory allocation failed.</description>
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
_Check_return_ HRESULT FUSIONAPI NuiFusionShadePointCloud(
    _In_ const NUI_FUSION_IMAGE_FRAME *pPointCloudFrame,
    _In_ const Matrix4 *pWorldToCameraTransform,
    _In_opt_ const Matrix4 *pWorldToBGRTransform,
    _In_opt_ const NUI_FUSION_IMAGE_FRAME *pShadedSurfaceFrame,  
    _In_opt_ const NUI_FUSION_IMAGE_FRAME *pShadedSurfaceNormalsFrame
    );

/// <summary>
/// The AlignPointClouds function uses an iterative algorithm to align two sets of oriented point 
/// clouds and calculate the camera's relative pose. This is a generic function which can be used
/// independently of a Reconstruction Volume with sets of overlapping point clouds.
/// All images must be the same size and have the same camera parameters.
/// </summary>
/// <param name="pReferencePointCloudFrame">A pointer to a pre-allocated point cloud frame of the
/// reference camera, or the previous Kinect point cloud frame.</param>
/// <param name="pObservedPointCloudFrame">A pointer to a pre-allocated point cloud frame of the 
/// observed camera, or the current Kinect frame.</param>
/// <param name="maxAlignIterationCount">The maximium number of iterations of the algorithm to run. 
/// The minimum value is 1. Using only a small number of iterations will have a faster runtime, 
/// however, the algorithm may not converge to the correct transformation.</param>
/// <param name="pDeltaFromReferenceFrame">Optionally, a pointer to a pre-allocated color image 
/// frame, to be filled with argb color-coded data from the camera tracking. This may be used as
/// input to additional vision algorithms such as object segmentation. Values vary depending on
/// whether the pixel was a valid pixel used in tracking (inlier) or failed in different tests (outlier).
/// 0xff000000 indicates an invalid input vertex (e.g. from 0 input depth), or one where no 
/// correspondences occur between point cloud images. Outlier vertices rejected due to too large a
/// distance between vertices are coded as 0xff008000. Outlier vertices rejected due to to large a
/// difference in normal angle between point clouds are coded as 0xff800000. Inliers are color shaded
/// depending on the residual energy at that point, with more saturated colors indicating more
/// discrepancy between vertices and less saturated colors (i.e. more white) representing less 
/// discrepancy, or less information at that pixel. Pass NULL if this image is not required.</param>
/// <param name="pReferenceToObservedTransform">A pointer to a pre-allocated transformation matrix.
/// At entry to the function this should be filled with the best guess for the reference to
/// observed transform (usually the last frame's calculated pose). At exit this is filled with the
/// calculated pose or identity if the calculation failed.</param>
/// <returns>
/// Returns S_OK if successful; otherwise, returns one of the following failure codes:
/// <list type="table">
///    <listheader>
///       <term>Error code</term>
///       <description>Description</description>
///    </listheader>
///    <item>
///       <term>E_INVALIDARG</term>
///       <description>The <paramref name="pReferencePointCloudFrame"/> or 
///       <paramref name="pObservedPointCloudFrame"/> or <paramref name="pReferenceToObservedTransform" />
///       parameter is NULL, the <paramref name="maxAlignIterationCount"/> is 0,
///       the <paramref name="pReferencePointCloudFrame"/> or 
///       <paramref name="pObservedPointCloudFrame"/> or <paramref name="pDeltaFromReferenceFrame"/>
///       parameters are different image resolutions,
///       the <paramref name="pReferencePointCloudFrame"/> or 
///       <paramref name="pObservedPointCloudFrame"/> or <paramref name="pDeltaFromReferenceFrame"/>
///       parameters have different <c>pCameraParameter</c> members,
///       or the <paramref name="pReferencePointCloudFrame"/> or 
///       <paramref name="pObservedPointCloudFrame"/> or <paramref name="pDeltaFromReferenceFrame"/>
///       parameter is an incorrect image type.
///       </description>
///    </item>
///    </item>
///       <term>E_NOINTERFACE</term>
///       <description>The <paramref name="pReferencePointCloudFrame"/> or 
///       <paramref name="pObservedPointCloudFrame"/> or 
///       <paramref name="pDeltaFromReferenceFrame"/> parameter's <c>pFrameBuffer</c> 
///       member is NULL.</description>
///    </item>
///    <item>
///       <term>E_NUI_FUSION_TRACKING_ERROR</term>
///       <description>The algorithm encountered a problem aligning the input point clouds and 
///       could not calculate a valid transformation. The
///       <paramref name="pReferenceToObservedTransform"/> parameter is set to identity.</description>
///    </item>
///    <item>
///       <term>E_OUTOFMEMORY</term>
///       <description>A CPU memory allocation failed.</description>
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
/// To find the frame-to-frame relative transformation between two sets of point clouds in the 
/// camera local frame of reference (e.g. both created by NuiFusionDepthFloatFrameToPointCloud),
/// set the pReferenceToObservedTransform to NULL or identity.
/// To calculate the frame-to-model pose transformation between point clouds calculated from new depth
/// frames with NuiFusionDepthFloatFrameToPointCloud and point clouds calculated from an existing 
/// Reconstruction volume with CalculatePointCloud (e.g. from the previous frame), pass the 
/// CalculatePointCloud image as the reference frame, and the current depth frame point cloud from 
/// NuiFusionDepthFloatFrameToPointCloud as the observed frame. Set the pReferenceToObservedTransform 
/// to the previous frames calculated camera pose that was used in the CalculatePointCloud call.
/// Note that here the current depth frame point cloud will be in the camera local frame of reference,
/// whereas the raycast points and normals will be in the global/world coordinate system.
/// By passing the pReferenceToObservedTransform you make the algorithm aware of the transformation
/// between the two coordinate systems.
/// The pReferenceToObservedTransform pose supplied can also take into account information you may
/// have from other sensors or sensing mechanisms to aid the tracking. To do this multiply the 
/// relative frame to frame delta transformation from the other sensing system with the previous 
/// frame's pose before passing to this function. Note that any delta transform used should be in
/// the same coordinate system as that returned by the NuiFusionDepthFloatFrameToPointCloud 
/// calculation.
/// </remarks>
_Check_return_ HRESULT FUSIONAPI NuiFusionAlignPointClouds(
    _In_ const NUI_FUSION_IMAGE_FRAME *pReferencePointCloudFrame,
    _In_ const NUI_FUSION_IMAGE_FRAME *pObservedPointCloudFrame,
    _In_ USHORT maxAlignIterationCount,
    _In_opt_ const NUI_FUSION_IMAGE_FRAME *pDeltaFromReferenceFrame,
    _Inout_ Matrix4 *pReferenceToObservedTransform
    );

#ifdef __cplusplus
} //close extern "C"
#endif