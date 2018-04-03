#include "stdafx.h"
#include "SmoothingParameters.h"
struct SmoothingParameters {
    float smoothing = .25f;    // [0..1], lower values closer to raw data
    float correction = .25f;   // [0..1], lower values slower to correct towards the raw data
    float prediction = .05f;   // [0..n], the number of frames to predict into the future
    float jitterRadius = 0.03f; // The radius in meters for jitter reduction
    float maxDeviationRadius = .25f; // The maximum radius in meters that filtered positions are allowed to deviate from raw data
};
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
    params.smoothing = .5f;
    params.correction = .25f;
    params.maxDeviationRadius = .25f;
    params.jitterRadius = 0.4f;
    params.prediction = .25f;
    return params;
}