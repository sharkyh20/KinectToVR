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
#include <VectorMath.h>
#include "QuaternionMath.h"


#define PI 3.14159265359

//Credit to https://social.msdn.microsoft.com/Forums/en-US/eb647eeb-26ef-45d6-ba73-ac26b8b46925/joint-orientation-smoothing-unity-c?forum=kinectv2sdk

/* Kinect Vector 4 structure: x,y,z,w
*  OpenVR structure: w,x,y,z
*/

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


    
    
    void ApplyJointRotation(JointOrientation joints[])
    {
        const Vector4 fromTo = KMath::fromToRotation(upVRAxis(), forwardVRAxis());
        for (int i = 0; i < JointType_Count; ++i) {
            if (i == 14) {
                std::cerr << "";
            }
            Vector4 lastRotation = KMath::product(
                joints[i].Orientation,
                fromTo);
            //std::cerr << "Joint lastRot" << i << ": " << lastRotation.w << ", " << lastRotation.x << ", " << lastRotation.y << ", " << lastRotation.z << '\n';
            rotations.push_back(lastRotation);
            filteredJointOrientations[i] = SmoothFilter(rotations, filteredJointOrientations[i]);

            rotations.pop_front();
        }
    }
    sf::Vector3f upVRAxis() { return { 0,1,0 }; }
    sf::Vector3f forwardVRAxis() { return { 0,0,1 }; }

    
    
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
                float weight = 1.0f - (KMath::dot(lastMedian, quaternion) / (PI / 2.0f)); // 0 degrees of difference => weight 1. 180 degrees of difference => weight 0.
                Vector4 weightedQuaternion = KMath::lerp(lastMedian, quaternion, weight);
                
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

        return KMath::NormalizeQuaternion(median);
    }

    
};