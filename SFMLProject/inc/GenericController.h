#pragma once
#include "stdafx.h"
#include "SFML/System/Vector2.hpp"


class GenericController
{
public:
	GenericController()
	{
	}

	~GenericController()
	{
	}

	sf::Vector2f leftStickValue()
	{
	}

	sf::Vector2f rightStickValue()
	{
	}

	bool leftTriggerPress()
	{
	}

	bool rightTriggerPress()
	{
	}

	void sendHaptic(HapticPulse haptic)
	{
	}
};
