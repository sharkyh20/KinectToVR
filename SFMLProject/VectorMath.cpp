#include "stdafx.h"
#include "VectorMath.h"
#include <SFML\System\Vector3.hpp>
#include <math.h>
#include <string>
namespace KMath {
    float length(sf::Vector3f vector) {
        return
            sqrt(pow(vector.x, 2) + pow(vector.y, 2) + pow(vector.z, 2));
    }
    std::string to_string(sf::Vector3f v) {
        return std::to_string(v.x) + ", " + std::to_string(v.y) + ", " + std::to_string(v.z);
    }

    sf::Vector3f cross(sf::Vector3f v1, sf::Vector3f v2) {
        float x = (v1.y * v2.z) - (v1.z*v2.y);
        float y = -((v1.x*v2.z) - (v1.z*v2.x));
        float z = (v1.x*v2.y) - (v1.y*v2.x);
        return { x,y,z };
    }
    float dot(sf::Vector3f v1, sf::Vector3f v2) {
        return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
    }
    //https://stackoverflow.com/questions/42421611/3d-vector-rotation-in-c
    // v: a vector in 3D space
    // k: a unit vector describing the axis of rotation
    // theta: the angle (in radians) that v rotates around k
    sf::Vector3f rotate(const sf::Vector3f& v, const sf::Vector3f& k, double theta) {
        /* //DEBUG
        std::cout << "Rotating " << to_string(v) << " "
        << theta << " radians around "
        << to_string(k) << "..." << std::endl;
        */
        float cos_theta = cos(theta);
        float sin_theta = sin(theta);

        sf::Vector3f rotated = (v * cos_theta) + (cross(k, v) * sin_theta) + (k * dot(k, v)) * (1 - cos_theta);

        //std::cout << "Rotated: " << to_string(rotated) << std::endl;  //DEBUG

        return rotated;
    }

};