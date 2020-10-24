#pragma once
#include <iostream>

#define _USE_MATH_DEFINES
#include <math.h>

class LowPassFilter
{
public:

	LowPassFilter::LowPassFilter() :
		output(0),
		ePow(0)
	{
	}

	LowPassFilter::LowPassFilter(float iCutOffFrequency, float iDeltaTime) :
		output(0),
		ePow(1 - exp(-iDeltaTime * 2 * M_PI * iCutOffFrequency))
	{
		if (iDeltaTime <= 0)
		{
			ePow = 0;
		}
		if (iCutOffFrequency <= 0)
		{
			ePow = 0;
		}
	}

	float LowPassFilter::update(float input)
	{
		return output += (input - output) * ePow;
	}

	float LowPassFilter::update(float input, float deltaTime, float cutoffFrequency)
	{
		reconfigureFilter(deltaTime, cutoffFrequency); //Changes ePow accordingly.
		return output += (input - output) * ePow;
	}

	void LowPassFilter::reconfigureFilter(float deltaTime, float cutoffFrequency)
	{
		if (deltaTime <= 0)
		{
			ePow = 0;
		}
		if (cutoffFrequency <= 0)
		{
			ePow = 0;
		}
		ePow = 1 - exp(-deltaTime * 2 * M_PI * cutoffFrequency);
	}

private:
	float output;
	float ePow;
};
