#pragma once
#include "stdafx.h"
#include <queue>
#include <math.h>
#include <iostream>
#include <algorithm>
#include <assert.h>

#include <SFML\System\Vector3.hpp>
#include <KinectSettings.h>
#include "KinectV1Includes.h"
#include <openvr_math.h>
#include <VectorMath.h>


#define PI 3.14159265359

//Credit to https://social.msdn.microsoft.com/Forums/en-US/eb647eeb-26ef-45d6-ba73-ac26b8b46925/joint-orientation-smoothing-unity-c?forum=kinectv2sdk

/* Kinect Vector 4 structure: x,y,z,w
*  OpenVR structure: w,x,y,z
*/

class RotationalSmoothingFilter {
public:
    RotationalSmoothingFilter() { init(); }
    ~RotationalSmoothingFilter() {}
    NUI_SKELETON_POSITION_INDEX jointType;
    int queueSize = 5;

    void init() {
        rotations = std::deque<Vector4>();
        /*
        for (int i = 0; i < queueSize; ++i) {
            rotations.push_back({ 0,0,0,0 }); //MAY NEED TO CHANGE THIS TO SOME OTHER ROT
        }
        */
        for (int i = 0; i < NUI_SKELETON_POSITION_COUNT; ++i) {
            filteredJointOrientations[i] = { 0,0,0,0 };
        }
    }
    void update(NUI_SKELETON_BONE_ORIENTATION joints[]) {
        ApplyJointRotation(joints);
    }
    inline const Vector4* GetFilteredJoints() const { return &filteredJointOrientations[0]; }
private:
    std::deque<Vector4> rotations;
    Vector4 filteredJointOrientations[NUI_SKELETON_POSITION_COUNT];


    Vector4 vrToKinectQuat(vr::HmdQuaternion_t vrQuaternion) {
        Vector4 temp;
        temp.w = vrQuaternion.w;
        temp.x = vrQuaternion.x;
        temp.y = vrQuaternion.y;
        temp.z = vrQuaternion.z;
        return temp;
    }
    vr::HmdQuaternion_t kinectToVRQuat(Vector4 kQuaternion) {
        vr::HmdQuaternion_t temp;
        temp.w = kQuaternion.w;
        temp.x = kQuaternion.x;
        temp.y = kQuaternion.y;
        temp.z = kQuaternion.z;
        return temp;
    }
    bool equal(const Vector4& lhs, const Vector4& rhs ) {
        return 
            lhs.w == rhs.w
            && lhs.x == rhs.x
            && lhs.y == rhs.y
            && lhs.z == rhs.z
        ;
    }
    sf::Vector3f normalise(const sf::Vector3f & v) {
        float  vecLength = v.x*v.x + v.y*v.y + v.z*v.z;
        if (vecLength == 0)
            return v;
        vecLength = 1.0 / sqrt(vecLength);

        return { 
            v.x * vecLength,
            v.y * vecLength,
            v.z * vecLength };
    }
    Vector4 fromToRotation(const sf::Vector3f &from, const sf::Vector3f &to) {
        sf::Vector3f v0 = from;
        sf::Vector3f v1 = to;
        v0 = normalise(v0);
        v1 = normalise(v1);

        float d = KMath::dot(v0, v1);
        if (d >= 1.0f) {
            Vector4 r;
            r.w = 1;
            return r;
        }
        else if (d <= -1.0f) //Exactly opposite
        {
            sf::Vector3f axis(1.f, 0.f, 0.f);
            axis = KMath::cross(axis, v0);
            if (KMath::length(axis) == 0) {
                axis = { 0.f, 1.f, 0.f };
                axis = KMath::cross(axis, v0);
            }
            return normalisedQ({ axis.x, axis.y, axis.z, 0 });
        }

        const float s = sqrtf((1 + d) * 2);
        const float invs = 1.f / s;
        const sf::Vector3f c = KMath::cross(v0, v1)*invs;
        return normalisedQ({ c.x, c.y, c.z, s * .5f });
    }
    Vector4 product(const Vector4 &lhs, const Vector4 &rhs) {
        return {
            (lhs.w * rhs.x) + (lhs.x * rhs.w) + (lhs.y * rhs.z) - (lhs.z * rhs.y),
            (lhs.w * rhs.y) + (lhs.y * rhs.w) + (lhs.z * rhs.x) - (lhs.x * rhs.z),
            (lhs.w * rhs.z) + (lhs.z * rhs.w) + (lhs.x * rhs.y) - (lhs.y * rhs.x),
            (lhs.w * rhs.w) - (lhs.x * rhs.x) - (lhs.y * rhs.y) - (lhs.z * rhs.z)
        };
    }
    void ApplyJointRotation(NUI_SKELETON_BONE_ORIENTATION joints[])
    {
        const Vector4 fromTo = fromToRotation(upVRAxis(), forwardVRAxis());
        for (int i = 0; i < NUI_SKELETON_POSITION_COUNT; ++i) {
            Vector4 lastRotation = product(
                joints[i].absoluteRotation.rotationQuaternion,
                fromTo);
            //std::cerr << "Joint lastRot" << i << ": " << lastRotation.w << ", " << lastRotation.x << ", " << lastRotation.y << ", " << lastRotation.z << '\n';
            rotations.push_back(lastRotation);
            filteredJointOrientations[i] = SmoothFilter(rotations, filteredJointOrientations[i]);

            rotations.pop_front();
        }
    }
    sf::Vector3f upVRAxis() { return { 0,1,0 }; }
    sf::Vector3f forwardVRAxis() { return { 0,0,1 }; }

    float dot(Vector4 q1, Vector4 q2) {
        return q1.x * q2.x + q1.y * q2.y + q1.z *q2.z + q1.w * q2.w;
    }
    float length(Vector4 v) {
        return sqrt(v.w*v.w + v.x *v.x + v.y*v.y + v.z*v.z);
    }
    Vector4 normalisedQ(Vector4 a) {
        Vector4 v = a;
        float magnitude = pow(length(v), 2);
        v.w /= magnitude;
        v.x /= magnitude;
        v.y /= magnitude;
        v.z /= magnitude;
        return v;
    }
    Vector4 lerp(const Vector4& a, const Vector4& b, const float t)
    {
        Vector4 r;
        float t_ = 1 - t;
        r.x = t_ * a.x + t * b.x;
        r.y = t_ * a.y + t * b.y;
        r.z = t_ * a.z + t * b.z;
        r.w = t_ * a.w + t * b.w;
        r = normalisedQ(r);
        return r;
    }
    
    Vector4 SmoothFilter(std::deque<Vector4> quaternions, Vector4 lastMedian)
    {
        Vector4 median = Vector4{ .0f, .0f, .0f, .0f };
        for (Vector4 quaternion : quaternions)
        {
            /*
            if (equal(quaternion, { .0f, .0f, .0f, .0f })) {
                
            }
            */
            // else {}
                float weight = 1.0f - (dot(lastMedian, quaternion) / (PI / 2.0f)); // 0 degrees of difference => weight 1. 180 degrees of difference => weight 0.
                Vector4 weightedQuaternion = lerp(lastMedian, quaternion, weight);
                
                //std::cerr << "weight: " <<  weight << "\n";
                median.x += weightedQuaternion.x;
                median.y += weightedQuaternion.y;
                median.z += weightedQuaternion.z;
                median.w += weightedQuaternion.w;
            
        }

        median.x /= quaternions.size();
        median.y /= quaternions.size();
        median.z /= quaternions.size();
        median.w /= quaternions.size();

        return NormalizeQuaternion(median);
    }

    Vector4 NormalizeQuaternion(Vector4 quaternion)
    {
        if (equal(quaternion, { 0.0f,0.0f,0.0f,0.0f })) 
        {
            return { 0.0f,0.0f,0.0f,0.0f };
        }
        else {
            float x = quaternion.x;
            float y = quaternion.y;
            float z = quaternion.z;
            float w = quaternion.w;
            //This is an issue with the original code, it didn't sqrt the length, leading to spazzing rotations occasionally
            //float length = 1.0f / (w * w + x * x + y * y + z * z); 
            float len = 1.0f / length(quaternion);
            Vector4 v;
            v.x = x * len;
            v.y = y * len;
            v.z = z * len;
            v.w = w * len;
            return v;
        }
    }
};