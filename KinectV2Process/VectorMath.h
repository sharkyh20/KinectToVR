#pragma once
#include "stdafx.h"
#include <SFML\System\Vector3.hpp>
#include <math.h>
float length(sf::Vector3f vector) {
    return
        sqrt(pow(vector.x, 2) + pow(vector.y, 2) + pow(vector.z, 2));
}
