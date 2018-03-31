#pragma once
#include "stdafx.h"


#include <algorithm>
#include "Kinect.h"
#include "SmoothingParameters.h"

#define max(a,b)            (((a) > (b)) ? (a) : (b))
//Copyright credited to Microsoft, and source code found at https://github.com/zwang87/MixedRealitywithLaserWhiteboard/blob/master/Assets/Scripts/KinectScripts/Filters/BoneOrientationsFilter.cs
// Converted into C++

class DoubleExpBoneOrientationsFilter
{
private:

    // Historical Filter Data.  
    struct FilterDoubleExponentialData
    {
        // Gets or sets Historical Position.  
        Vector4 RawBoneOrientation = { 0,0,0,0 };

        // Gets or sets Historical Filtered Position.  
        Vector4 FilteredBoneOrientation = { 0,0,0,0 };

        // Gets or sets Historical Trend.  
        Vector4 Trend = { 0,0,0,0 };

        // Gets or sets Historical FrameCount.  
        int FrameCount = 0;
    };

    // The previous filtered orientation data.
    FilterDoubleExponentialData history[JointType_Count];


    // The transform smoothing parameters for this filter.

    SmoothingParameters smoothParameters;

    // True when the filter parameters are initialized.
    bool init;
public:
    // Initializes a new instance of the class.
    DoubleExpBoneOrientationsFilter()
    {
        init = false;
        Init();
    }
    inline const Vector4* GetFilteredJoints() const { return &filteredOrientations[0]; }
    // Initialize the filter with a default set of TransformSmoothParameters.
    void Init()
    {
        // Set some reasonable defaults
        Init(0.5f, 0.8f, 0.75f, 0.1f, 0.1f);
    }

    void Init(float smoothingValue, float correctionValue, float predictionValue, float jitterRadiusValue, float maxDeviationRadiusValue)
    {
        smoothParameters = getDefaultSmoothingParams();

        smoothParameters.maxDeviationRadius = maxDeviationRadiusValue; // Size of the max prediction radius Can snap back to noisy data when too high
        smoothParameters.smoothing = smoothingValue;                   // How much soothing will occur.  Will lag when too high
        smoothParameters.correction = correctionValue;                 // How much to correct back from prediction.  Can make things springy
        smoothParameters.prediction = predictionValue;                 // Amount of prediction into the future to use. Can over shoot when too high
        smoothParameters.jitterRadius = jitterRadiusValue;             // Size of the radius where jitter is removed. Can do too much smoothing when too high

        Reset();
        init = true;
    }

    // Initialize the filter with a set of TransformSmoothParameters.
    void Init(SmoothingParameters smoothingParameters)
    {
        smoothParameters = smoothingParameters;

        Reset();
        init = true;
    }

    /// Resets the filter to default values.
    void Reset()
    {
        for (int i = 0; i < JointType_Count; ++i)
            history[i] = FilterDoubleExponentialData();
    }

    // Implements a double exponential smoothing filter on the skeleton bone orientation quaternions.
    void UpdateFilter(IBody* const pBody, JointOrientation* jointsOrientations)
    {
        //        if (null == skeleton)
        //        {
        //            return;
        //        }

        //        if (skeleton.eTrackingState != KinectWrapper.NuiSkeletonTrackingState.SkeletonTracked)
        //        {
        //            return;
        //        }

        if (init == false)
        {
            Init(); // initialize with default parameters                
        }

        SmoothingParameters tempSmoothingParams = getDefaultSmoothingParams();

        // Check for divide by zero. Use an epsilon of a 10th of a millimeter
        smoothParameters.jitterRadius = max(0.0001f, smoothParameters.jitterRadius);

        tempSmoothingParams.smoothing = smoothParameters.smoothing;
        tempSmoothingParams.correction = smoothParameters.correction;
        tempSmoothingParams.prediction = smoothParameters.prediction;

        for (int jointIndex = 0; jointIndex < JointType_Count; jointIndex++)
        {
            //KinectWrapper.NuiSkeletonPositionIndex jt = (KinectWrapper.NuiSkeletonPositionIndex)jointIndex;

            // If not tracked, we smooth a bit more by using a bigger jitter radius
            // Always filter feet highly as they are so noisy
            Joint joints[JointType_Count];
            pBody->GetJoints(JointType_Count, joints);
            if (joints[jointIndex].TrackingState == TrackingState::TrackingState_Inferred ||
                jointIndex == JointType_FootLeft || jointIndex == JointType_FootRight)
            {
                tempSmoothingParams.jitterRadius = smoothParameters.jitterRadius * 2.0f;
                tempSmoothingParams.maxDeviationRadius = smoothParameters.maxDeviationRadius * 2.0f;
            }
            else
            {
                tempSmoothingParams.jitterRadius = smoothParameters.jitterRadius;
                tempSmoothingParams.maxDeviationRadius = smoothParameters.maxDeviationRadius;
            }

            FilterJoint(joints, jointIndex, tempSmoothingParams, jointsOrientations);
        }
    }
protected:
    // Update the filter for one joint.  
    bool jointPositionIsValid(sf::Vector3f vJointPosition)
    {
        return (vJointPosition.x != 0.0f ||
            vJointPosition.y != 0.0f ||
            vJointPosition.z != 0.0f);
    }
    sf::Vector3f normalise(const sf::Vector3f & v) {
        float  length = v.x*v.x + v.y*v.y + v.z*v.z;
        if (length == 0)
            return v;
        length = 1.0 / sqrt(length);

        return {
            v.x * length,
            v.y * length,
            v.z * length };
    }
    float length(Vector4 v) {
        return sqrt(v.w*v.w + v.x *v.x + v.y*v.y + v.z*v.z);
    }
    Vector4 normalisedQ(Vector4 a) {
        Vector4 v = a;
        float magnitude = pow(length(v), 2);
        v.w /= magnitude;
        v.x /= magnitude;
        v.y /= magnitude;
        v.z /= magnitude;
        return v;
    }
    Vector4 fromToRotation(const sf::Vector3f &from, const sf::Vector3f &to) {
        sf::Vector3f v0 = from;
        sf::Vector3f v1 = to;
        v0 = normalise(v0);
        v1 = normalise(v1);

        float d = KMath::dot(v0, v1);
        if (d >= 1.0f) {
            Vector4 r;
            r.w = 1;
            return r;
        }
        else if (d <= -1.0f) //Exactly opposite
        {
            sf::Vector3f axis(1.f, 0.f, 0.f);
            axis = KMath::cross(axis, v0);
            if (KMath::length(axis) == 0) {
                axis = { 0.f, 1.f, 0.f };
                axis = KMath::cross(axis, v0);
            }
            return normalisedQ({ axis.x, axis.y, axis.z, 0 });
        }

        const float s = sqrtf((1 + d) * 2);
        const float invs = 1.f / s;
        const sf::Vector3f c = KMath::cross(v0, v1)*invs;
        return normalisedQ({ c.x, c.y, c.z, s * .5f });
    }
    Vector4 lerp(const Vector4& a, const Vector4& b, const float t)
    {
        Vector4 r;
        float t_ = 1 - t;
        r.x = t_ * a.x + t * b.x;
        r.y = t_ * a.y + t * b.y;
        r.z = t_ * a.z + t * b.z;
        r.w = t_ * a.w + t * b.w;
        r = normalisedQ(r);
        return r;
    }
    Vector4 divide(const Vector4 x, const Vector4 y) {
        return product(x, inverse(y));
    }
    Vector4 divide(const Vector4& x, float k) {
        Vector4 q;
        q.w = x.w /k;
        q.x = x.x / k;
        q.y = x.y / k;
        q.z = x.z / k;
        return q;
    }
    Vector4 conj(const Vector4 x) {
        return { x.w, -x.x, -x.y, -x.z };
    }
    Vector4 inverse(const Vector4 x) {   // Might need to take in reference
        return divide(conj(x), norm_squared(x));
    }

    float norm_squared(const Vector4 x) const {
        return  x.w * x.w + x.x * x.x + x.y * x.y + x.z * x.z;
    }
    Vector4 RotationBetweenQuaternions(Vector4 quaternionA, Vector4 quaternionB)
    {
        Vector4 modifiedB = EnsureQuaternionNeighborhood(quaternionA, quaternionB);
        return product(inverse(quaternionA), modifiedB);
    }
    float dot(Vector4 q1, Vector4 q2) {
        return q1.x * q2.x + q1.y * q2.y + q1.z *q2.z + q1.w * q2.w;
    }
    Vector4 EnsureQuaternionNeighborhood(Vector4 quaternionA, Vector4 quaternionB)
    {
        if (dot(quaternionA, quaternionB) < 0)
        {
            // Negate the second quaternion, to place it in the opposite 3D sphere.
            //return -quaternionB;
            return Vector4{ -quaternionB.x, -quaternionB.y, -quaternionB.z, -quaternionB.w };
        }

        return quaternionB;
    }
    float QuaternionAngle(Vector4 rotation)
    {
        //rotation.Normalize();
        float angle = 2.0f * acos(rotation.w);
        return angle;
    }

    bool isTrackedOrInferred(Joint joints[], int index) {
        return (joints[index].TrackingState == TrackingState_Inferred || joints[index].TrackingState == TrackingState_Tracked);
    }
    bool rotationIsValid(Vector4 q) {
        if (q.w == 0 && q.x == 0 && q.y == 0 && q.z == 0)
            return false;
        return true;
    }
    void FilterJoint(Joint joints[], int jointIndex, SmoothingParameters & params, JointOrientation *jointOrientations)
    {
        //        if (null == skeleton)
        //        {
        //            return;
        //        }

        //        int jointIndex = (int)jt;

        Vector4 filteredOrientation;
        Vector4 trend;

        sf::Vector3f fwdVector = { 0, 0 , 1 }; // LOOK INTO THIS VALUE!!!
        sf::Vector3f upAxis = { 0,1,0 };
        if (fwdVector == sf::Vector3f{0, 0, 0})
            return;
        //Vector4 rawOrientation = fromToRotation(fwdVector, jointOrientations[jointIndex].Orientation);
        Vector4 rawOrientation = jointOrientations[jointIndex].Orientation;
        Vector4 prevFilteredOrientation = history[jointIndex].FilteredBoneOrientation;
        Vector4 prevTrend = history[jointIndex].Trend;
        sf::Vector3f rawPosition = { joints[jointIndex].Position.X, joints[jointIndex].Position.Y, joints[jointIndex].Position .Z};
        bool orientationIsValid = jointPositionIsValid(rawPosition) && isTrackedOrInferred(joints, jointIndex) && rotationIsValid(rawOrientation);

        if (!orientationIsValid)
        {
            if (history[jointIndex].FrameCount > 0)
            {
                rawOrientation = history[jointIndex].FilteredBoneOrientation;
                history[jointIndex].FrameCount = 0;
            }
        }

        // Initial start values or reset values
        if (history[jointIndex].FrameCount == 0)
        {
            // Use raw position and zero trend for first value
            filteredOrientation = rawOrientation;
            trend = Vector4{ 0,0,0,0 };
        }
        else if (history[jointIndex].FrameCount == 1)
        {
            // Use average of two positions and calculate proper trend for end value
            Vector4 prevRawOrientation = history[jointIndex].RawBoneOrientation;
            filteredOrientation = lerp(prevRawOrientation, rawOrientation, 0.5f);

            Vector4 diffStarted = lerp(filteredOrientation, prevFilteredOrientation, 0); // Might not be 0!!!
            trend = lerp(prevTrend, diffStarted, params.correction);
        }
        else
        {
            // First apply a jitter filter
            Vector4 diffJitter = RotationBetweenQuaternions(rawOrientation, prevFilteredOrientation);
            float diffValJitter = abs(QuaternionAngle(diffJitter));

            if (diffValJitter <= params.jitterRadius)
            {
                filteredOrientation = lerp(prevFilteredOrientation, rawOrientation, diffValJitter / params.jitterRadius);
            }
            else
            {
                filteredOrientation = rawOrientation;
            }

            // Now the double exponential smoothing filter
            filteredOrientation = lerp(filteredOrientation, product(prevFilteredOrientation, prevTrend), params.smoothing);

            diffJitter = RotationBetweenQuaternions(filteredOrientation, prevFilteredOrientation);
            trend = lerp(prevTrend, diffJitter, params.correction);
        }

        // Use the trend and predict into the future to reduce latency
        Vector4 predictedOrientation = product(filteredOrientation,lerp(Vector4{ 0,0,0,0 }, trend, params.prediction));

        // Check that we are not too far away from raw data
        Vector4 diff = RotationBetweenQuaternions(predictedOrientation, filteredOrientation);
        float diffVal = abs(QuaternionAngle(diff));

        if (diffVal > params.maxDeviationRadius)
        {
            predictedOrientation = lerp(filteredOrientation, predictedOrientation, params.maxDeviationRadius / diffVal);
        }

        //        predictedOrientation.Normalize();
        //        filteredOrientation.Normalize();
        //        trend.Normalize();

        // Save the data from this frame
        history[jointIndex].RawBoneOrientation = rawOrientation;
        history[jointIndex].FilteredBoneOrientation = filteredOrientation;
        history[jointIndex].Trend = trend;
        history[jointIndex].FrameCount++;

        // Set the filtered and predicted data back into the bone orientation
        if (rotationIsValid(predictedOrientation))
        {
            filteredOrientations[jointIndex] = predictedOrientation; //Was previously a TRS matrix!!!
        }
    }
    Vector4 product(const Vector4 &lhs, const Vector4 &rhs) {
        return {
            (lhs.w * rhs.x) + (lhs.x * rhs.w) + (lhs.y * rhs.z) - (lhs.z * rhs.y),
            (lhs.w * rhs.y) + (lhs.y * rhs.w) + (lhs.z * rhs.x) - (lhs.x * rhs.z),
            (lhs.w * rhs.z) + (lhs.z * rhs.w) + (lhs.x * rhs.y) - (lhs.y * rhs.x),
            (lhs.w * rhs.w) - (lhs.x * rhs.x) - (lhs.y * rhs.y) - (lhs.z * rhs.z)
        };
    }
    private:
        Vector4 filteredOrientations[JointType_Count];
};