#pragma once
#include "stdafx.h"
#include <SFML\System\Vector3.hpp>
#include <glm/glm.hpp>
#include <string>
namespace KMath {
#define PI 3.14159265359
    float length(sf::Vector3f vector);
    std::string to_string(sf::Vector3f v);

    glm::vec3 defaultUpVector();
    glm::vec3 defaultForwardVector();

    sf::Vector3f cross(sf::Vector3f v1, sf::Vector3f v2);
    float dot(sf::Vector3f v1, sf::Vector3f v2);
    sf::Vector3f rotate(const sf::Vector3f& v, const sf::Vector3f& k, double theta);
}