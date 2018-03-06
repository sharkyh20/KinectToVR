#pragma once
#include "stdafx.h"
#include <queue>
#include <math.h>
#include <iostream>
#include <algorithm>
#include <assert.h>

#include <SFML\System\Vector3.hpp>
#include <KinectSettings.h>
#include "Kinect.h"
#include <openvr_math.h>


#define PI 3.14159265359

//Credit to https://social.msdn.microsoft.com/Forums/en-US/eb647eeb-26ef-45d6-ba73-ac26b8b46925/joint-orientation-smoothing-unity-c?forum=kinectv2sdk

class RotationalSmoothingFilter {
public:
    RotationalSmoothingFilter() { init(); }
    ~RotationalSmoothingFilter() {}
    JointType jointType;
    int queueSize = 5;

    void init() {
        rotations = std::deque<Vector4>();
        /*
        for (int i = 0; i < queueSize; ++i) {
            rotations.push_back({ 0,0,0,0 }); //MAY NEED TO CHANGE THIS TO SOME OTHER ROT
        }
        */
        for (int i = 0; i < JointType_Count; ++i) {
            filteredJointOrientations[i] = { 0,0,0,0 };
        }
    }
    void update(JointOrientation joints[]) {
        ApplyJointRotation(joints);
    }
    inline const Vector4* GetFilteredJoints() const { return &filteredJointOrientations[0]; }
private:
    std::deque<Vector4> rotations;
    Vector4 filteredJointOrientations[JointType_Count];


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
    void ApplyJointRotation(JointOrientation joints[])
    {
        for (int i = 0; i < JointType_Count; ++i) {
            if (i == 14) {
                std::cerr << "";
            }
            Vector4 lastRotation = vrToKinectQuat(kinectToVRQuat(joints[i].Orientation) * KinectSettings::kinectRepRotation);
            //std::cerr << "Joint lastRot" << i << ": " << lastRotation.w << ", " << lastRotation.x << ", " << lastRotation.y << ", " << lastRotation.z << '\n';
            rotations.push_back(lastRotation);
            filteredJointOrientations[i] = SmoothFilter(rotations, filteredJointOrientations[i]);

            rotations.pop_front();
        }
    }
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
    float clip(float n, float lower, float upper) {
        return max(lower, min(n, upper));
    }
    Vector4 clamp(Vector4 v) {
        Vector4 temp = v;
        temp.x = clip(temp.x, -1.0f, 1.0f);
        temp.y = clip(temp.y, -1.0f, 1.0f);
        temp.z = clip(temp.z, -1.0f, 1.0f);
        temp.w = clip(temp.w, -1.0f, 1.0f);
        return temp;
    }
    
    Vector4 SmoothFilter(std::deque<Vector4> quaternions, Vector4 lastMedian)
    {
        Vector4 median = Vector4{ .0f, .0f, .0f, .0f };
        for (Vector4 quaternion : quaternions)
        {
            if (equal(quaternion, { .0f, .0f, .0f, .0f })) {
                
            }
            else {
                float weight = 1 - (dot(lastMedian, quaternion) / (PI / 2.0f)); // 0 degrees of difference => weight 1. 180 degrees of difference => weight 0.
                Vector4 weightedQuaternion = lerp(lastMedian, quaternion, weight);
                
                //weightedQuaternion = clamp(weightedQuaternion);
                //std::cerr << "weight: " <<  weight << "\n";
                median.x += weightedQuaternion.x;
                median.y += weightedQuaternion.y;
                median.z += weightedQuaternion.z;
                median.w += weightedQuaternion.w;
            }
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