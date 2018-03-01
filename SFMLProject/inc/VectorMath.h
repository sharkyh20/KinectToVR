#pragma once
#include "stdafx.h"
#include <SFML\System\Vector3.hpp>
#include <string>
float length(sf::Vector3f vector);
std::string to_string(sf::Vector3f v);

sf::Vector3f cross(sf::Vector3f v1, sf::Vector3f v2);
float dot(sf::Vector3f v1, sf::Vector3f v2);
sf::Vector3f rotate(const sf::Vector3f& v, const sf::Vector3f& k, double theta);