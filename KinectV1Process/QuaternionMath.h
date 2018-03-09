#pragma once
#include "stdafx.h"
#include <SFML\System\Vector3.hpp>
#include <openvr.h>
#include <openvr_math.h>
#include "VectorMath.h"

#include "KinectV1Includes.h"

namespace KMath {
    bool equal(const Vector4& lhs, const Vector4& rhs);
    Vector4 vrToKinectQuat(vr::HmdQuaternion_t vrQuaternion);
    vr::HmdQuaternion_t kinectToVRQuat(Vector4 kQuaternion);
    
    float dot(Vector4 q1, Vector4 q2);
    float length(Vector4 v);
    Vector4 normalisedQ(Vector4 a);
    Vector4 lerp(const Vector4& a, const Vector4& b, const float t);
    Vector4 fromToRotation(const sf::Vector3f &from, const sf::Vector3f &to);
    Vector4 product(const Vector4 &lhs, const Vector4 &rhs);
    Vector4 NormalizeQuaternion(Vector4 quaternion);
};
