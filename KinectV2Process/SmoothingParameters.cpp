#include "stdafx.h"
#include "SmoothingParameters.h"


//m_fSmoothing How much soothing will occur.  Will lag when too high
//m_fCorrection How much to correct back from prediction.  Can make things springy
//m_fMaxDeviationRadius Size of the max prediction radius Can snap back to noisy data when too high
//m_fJitterRadius Size of the radius where jitter is removed. Can do too much smoothing when too high
//m_fPrediction Amount of prediction into the future to use. Can over shoot when too high

SmoothingParameters getDefaultSmoothingParams() {
    SmoothingParameters params;
    params.smoothing = .25f;
    params.correction = .25f;
    params.maxDeviationRadius = .05f;
    params.jitterRadius = 0.03f;
    params.prediction = .25f;
    return params;
}
SmoothingParameters getAggressiveSmoothingParams() {
    SmoothingParameters params;
    params.smoothing = .25f;
    params.correction = .25f;
    params.maxDeviationRadius = .25f;
    params.jitterRadius = 0.4f;
    params.prediction = .25f;
    return params;
}
SmoothingParameters getRotationSmoothingParams() {
    SmoothingParameters params;
    params.smoothing = .6f;
    params.correction = .4f;
    params.maxDeviationRadius = .3f;
    params.jitterRadius = 0.5f;
    params.prediction = .25f;
    return params;
}