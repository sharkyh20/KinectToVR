#pragma once
#ifndef TYPES_H
#define TYPES_H
#include "stdafx.h"
#include "Kinect.h"

struct SmoothingParameters;
SmoothingParameters getDefaultSmoothingParams();
SmoothingParameters getAggressiveSmoothingParams();
#endif