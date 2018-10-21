#pragma once
#ifndef TYPES_H
#define TYPES_H
#include "stdafx.h"
#include "Kinect.h"

struct SmoothingParameters {
    float smoothing;    // [0..1], lower values closer to raw data
    float correction;   // [0..1], lower values slower to correct towards the raw data
    float prediction;   // [0..n], the number of frames to predict into the future
    float jitterRadius; // The radius in meters for jitter reduction
    float maxDeviationRadius; // The maximum radius in meters that filtered positions are allowed to deviate from raw data
};
SmoothingParameters getDefaultSmoothingParams();
SmoothingParameters getAggressiveSmoothingParams();
SmoothingParameters getRotationSmoothingParams();
#endif