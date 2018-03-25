#include "stdafx.h"
#include "KinectJointFilter.h"
#include "VectorMath.h"
inline float lerp(float f1, float f2, float fBlend)
{
    return f1 + (f2 - f1) * fBlend;
}

//--------------------------------------------------------------------------------------
// if joint is 0 it is not valid.
//--------------------------------------------------------------------------------------
inline bool jointPositionIsValid(sf::Vector3f vJointPosition)
{
    return (vJointPosition.x != 0.0f ||
        vJointPosition.y != 0.0f ||
        vJointPosition.z != 0.0f);
}

//--------------------------------------------------------------------------------------
// Implementation of a Holt Double Exponential Smoothing filter. The double exponential
// smooths the curve and predicts.  There is also noise jitter removal. And maximum
// prediction bounds.  The paramaters are commented in the init function.
//--------------------------------------------------------------------------------------
void DoubleExponentialFilter::update(IBody* const pBody)
{
    assert(pBody);

    // Check for divide by zero. Use an epsilon of a 10th of a millimeter
    m_fJitterRadius = max(0.0001f, m_fJitterRadius);

    SmoothingParameters SmoothingParams;

    int jointCapacity = 0;
    Joint joints[JointType_Count];

    pBody->GetJoints(jointCapacity, joints);
    for (int i = 0; i < JointType_Count; i++)
    {
        SmoothingParams.smoothing = m_fSmoothing;
        SmoothingParams.correction = m_fCorrection;
        SmoothingParams.prediction = m_fPrediction;
        SmoothingParams.jitterRadius = m_fJitterRadius;
        SmoothingParams.maxDeviationRadius = m_fMaxDeviationRadius;

        // If inferred, we smooth a bit more by using a bigger jitter radius
        Joint joint = joints[i];
        if (joint.TrackingState == TrackingState::TrackingState_Inferred)
        {
            SmoothingParams.jitterRadius *= 2.0f;
            SmoothingParams.maxDeviationRadius *= 2.0f;
        }

        update(joints, i, SmoothingParams);
    }
}

void DoubleExponentialFilter::update(Joint joints[])
{
    // Check for divide by zero. Use an epsilon of a 10th of a millimeter
    m_fJitterRadius = max(0.0001f, m_fJitterRadius);

    SmoothingParameters SmoothingParams;
    for (INT i = 0; i < JointType_Count; i++)
    {
        SmoothingParams.smoothing = m_fSmoothing;
        SmoothingParams.correction = m_fCorrection;
        SmoothingParams.prediction = m_fPrediction;
        SmoothingParams.jitterRadius = m_fJitterRadius;
        SmoothingParams.maxDeviationRadius = m_fMaxDeviationRadius;

        // If inferred, we smooth a bit more by using a bigger jitter radius
        Joint joint = joints[i];
        if (joint.TrackingState == TrackingState::TrackingState_Inferred)
        {
            SmoothingParams.jitterRadius *= 2.0f;
            SmoothingParams.maxDeviationRadius *= 2.0f;
        }

        update(joints, i, SmoothingParams);
    }

}

void DoubleExponentialFilter::update(Joint joints[], UINT JointID, SmoothingParameters smoothingParams)
{
    sf::Vector3f vPrevRawPosition;
    sf::Vector3f vPrevFilteredPosition;
    sf::Vector3f vPrevTrend;
    sf::Vector3f vRawPosition;
    sf::Vector3f vFilteredPosition;
    sf::Vector3f vPredictedPosition;
    sf::Vector3f vDiff;
    sf::Vector3f vTrend;

    float vectorLength;
    float fDiff;
    bool bJointIsValid;

    const Joint joint = joints[JointID];

    vRawPosition = sf::Vector3f(joint.Position.X, joint.Position.Y, joint.Position.Z);
    vPrevFilteredPosition = pointHistory[JointID].filteredPosition;
    vPrevTrend = pointHistory[JointID].trend;
    vPrevRawPosition = pointHistory[JointID].rawPosition;
    bJointIsValid = jointPositionIsValid(vRawPosition);

    // If joint is invalid, reset the filter
    if (!bJointIsValid)
    {
        pointHistory[JointID].frameCount = 0;
    }

    // Initial start values
    if (pointHistory[JointID].frameCount == 0)
    {
        vFilteredPosition = vRawPosition;
        vTrend = { 0,0,0 };
        pointHistory[JointID].frameCount++;
    }
    else if (pointHistory[JointID].frameCount == 1)
    {
        vFilteredPosition = (vRawPosition + vPrevRawPosition) * 0.5f;
        vDiff = vFilteredPosition - vPrevFilteredPosition;
        vTrend = (vDiff * smoothingParams.correction) 
            + (vPrevTrend * (1.0f - smoothingParams.correction));
        pointHistory[JointID].frameCount++;
    }
    else
    {
        // First apply jitter filter
        vDiff = vRawPosition - vPrevFilteredPosition;
        vectorLength = KMath::length(vDiff);
        fDiff = fabs(vectorLength);

        if (fDiff <= smoothingParams.jitterRadius)
        {
            vFilteredPosition = 
                vRawPosition 
                    * (fDiff / smoothingParams.jitterRadius)
                + vPrevFilteredPosition 
                    *( 1.0f - fDiff / smoothingParams.jitterRadius);
        }
        else
        {
            vFilteredPosition = vRawPosition;
        }

        // Now the double exponential smoothing filter
        vFilteredPosition = 
            vFilteredPosition * ( 1.0f - smoothingParams.smoothing)
            +
            (vPrevFilteredPosition + vPrevTrend) * smoothingParams.smoothing;


        vDiff = vFilteredPosition - vPrevFilteredPosition;
        vTrend = vDiff * smoothingParams.correction +
            vPrevTrend *( 1.0f - smoothingParams.correction);
    }

    // Predict into the future to reduce latency
    vPredictedPosition = vFilteredPosition + vTrend * smoothingParams.prediction;

    // Check that we are not too far away from raw data
    vDiff = vPredictedPosition - vRawPosition;
    vectorLength = KMath::length(vDiff);
    fDiff = fabs(vectorLength);

    if (fDiff > smoothingParams.maxDeviationRadius)
    {
        vPredictedPosition = 
            vPredictedPosition * (smoothingParams.maxDeviationRadius / fDiff)
            +
            vRawPosition * ( 1.0f - (smoothingParams.maxDeviationRadius / fDiff));
    }

    // Save the data from this frame
    pointHistory[JointID].rawPosition = vRawPosition;
    pointHistory[JointID].filteredPosition = vFilteredPosition;
    pointHistory[JointID].trend = vTrend;

    // Output the data
    filteredJointPoints[JointID] = vPredictedPosition;
}