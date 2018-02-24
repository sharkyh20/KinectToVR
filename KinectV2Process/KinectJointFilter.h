#pragma once
#include "stdafx.h"
#include <queue>
#include <assert.h>

#include <SFML\System\Vector3.hpp>
#include "Kinect.h"


// Joint Filter 
// Courtesy of https://social.msdn.microsoft.com/Forums/en-US/045b058a-ae3a-4d01-beb6-b756631b4b42/joint-smoothing-code?forum=kinectv2sdk
struct SmoothingParameters {
    float smoothing;    // [0..1], lower values closer to raw data
    float correction;   // [0..1], lower values slower to correct towards the raw data
    float prediction;   // [0..n], the number of frames to predict into the future
    float jitterRadius; // The radius in meters for jitter reduction
    float maxDeviationRadius; // The maximum radius in meters that filtered positions are allowed to deviate from raw data
};

// A holt double exponential smoothing filter
class DoubleExponentialFilterData {
public:
    sf::Vector3f rawPosition;
    sf::Vector3f filteredPosition;
    sf::Vector3f trend;
    uint32_t frameCount;
};

class DoubleExponentialFilter {
public:
    DoubleExponentialFilter() { init(); }
    ~DoubleExponentialFilter() { shutdown(); }
    void init(float fSmoothing = 0.25f, float fCorrection = 0.25f, float fPrediction = 0.25f, float fJitterRadius = 0.03f, float fMaxDeviationRadius = 0.05f)
    {
        Reset(fSmoothing, fCorrection, fPrediction, fJitterRadius, fMaxDeviationRadius);
    }

    void shutdown()
    {
    }

    void Reset(float fSmoothing = 0.25f, float fCorrection = 0.25f, float fPrediction = 0.25f, float fJitterRadius = 0.03f, float fMaxDeviationRadius = 0.05f)
    {
        assert(m_pFilteredJoints);
        assert(m_pHistory);

        m_fMaxDeviationRadius = fMaxDeviationRadius; // Size of the max prediction radius Can snap back to noisy data when too high
        m_fSmoothing = fSmoothing;                   // How much smothing will occur.  Will lag when too high
        m_fCorrection = fCorrection;                 // How much to correct back from prediction.  Can make things springy
        m_fPrediction = fPrediction;                 // Amount of prediction into the future to use. Can over shoot when too high
        m_fJitterRadius = fJitterRadius;             // Size of the radius where jitter is removed. Can do too much smoothing when too high

        memset(filteredJointPoints, 0, sizeof(sf::Vector3f) * JointType_Count);
        memset(pointHistory, 0, sizeof(DoubleExponentialFilterData) * JointType_Count);
    }

    void update(IBody* const pBody);
    void update(Joint joints[]);

    inline const sf::Vector3f* GetFilteredJoints() const { return &filteredJointPoints[0]; }

private:
    sf::Vector3f filteredJointPoints[JointType_Count];
    DoubleExponentialFilterData pointHistory[JointType_Count];
    float m_fSmoothing;
    float m_fCorrection;
    float m_fPrediction;
    float m_fJitterRadius;
    float m_fMaxDeviationRadius;

    void update(Joint joints[], UINT JointID, SmoothingParameters smoothingParams);
};
