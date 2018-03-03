#pragma once
#include "stdafx.h"
#include <queue>
#include <iostream>
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
    int queueSize = 15;

    void init() {
        rotations = std::deque<Vector4>();
        for (int i = 0; i < queueSize; ++i) {
            rotations.push_back({ 0,0,0,0 }); //MAY NEED TO CHANGE THIS TO SOME OTHER ROT
        }
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
    void ApplyJointRotation(JointOrientation joints[])
    {
        for (int i = 0; i < JointType_Count; ++i) {
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
    Vector4 SmoothFilter(std::deque<Vector4> quaternions, Vector4 lastMedian)
    {
        Vector4 median = Vector4{ .0f, .0f, .0f, .0f };
        for (Vector4 quaternion : quaternions)
        {
            float weight = 1 - (dot(lastMedian, quaternion) / (PI / 2)); // 0 degrees of difference => weight 1. 180 degrees of difference => weight 0.
            Vector4 weightedQuaternion = lerp(lastMedian, quaternion, weight);
            std::cerr << "weight: " <<  weight << "\n";
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
        float x = quaternion.x;
        float y = quaternion.y;
        float z = quaternion.z;
        float w = quaternion.w;
        float length = 1.0f / (w * w + x * x + y * y + z * z);
        Vector4 v;
        v.x = x * length;
        v.y = y * length;
        v.z = z * length;
        v.w = w * length;
        return v;
    }
};