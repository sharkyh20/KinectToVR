#pragma once
#include "stdafx.h"
#include <queue>
#include <assert.h>

#include <SFML\System\Vector3.hpp>
#include <KinectSettings.h>
#include "Kinect.h"
#include <openvr_math.h>
#include "VectorMath.h"

#include "SmoothingParameters.h"

// Joint Filter 
// Courtesy of https://social.msdn.microsoft.com/Forums/en-US/045b058a-ae3a-4d01-beb6-b756631b4b42/joint-smoothing-code?forum=kinectv2sdk

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
    DoubleExponentialFilter() { init(getAggressiveSmoothingParams()); }
    ~DoubleExponentialFilter() { shutdown(); }
    void init(SmoothingParameters p) {
        Reset(p.smoothing, p.correction, p.prediction, p.jitterRadius, p.maxDeviationRadius);
    }
    void init(float fSmoothing = 0.25f, float fCorrection = 0.25f, float fPrediction = 0.25f, float fJitterRadius = 0.03f, float fMaxDeviationRadius = 0.05f)
    {
        Reset(fSmoothing, fCorrection, fPrediction, fJitterRadius, fMaxDeviationRadius);
    }

    void shutdown()
    {
    }

    void Reset(float fSmoothing = 0.25f, float fCorrection = 0.25f, float fPrediction = 0.25f, float fJitterRadius = 0.03f, float fMaxDeviationRadius = 0.05f)
    {
        assert(filteredJointPoints);
        assert(pointHistory);

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

