#ifndef MTOOLS_H
#define MTOOLS_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include <algorithm>
#include <string>
#include <time.h>
#include <vector>
#include <cmath> //Para lenguaje C++

namespace Mtools
{
    struct ContactPoint
    {
        Eigen::Vector3f p;
        Eigen::Vector3f n;
        unsigned int id;
    };

    struct Quaternion
    {
        Quaternion()
        {
            x = y = z = 0.0f;
            w = 1.0f;
        }

        float x,y,z,w;
    };

    struct TriangleFace6D
    {
        int id[6];
        ContactPoint normal;
        std::vector<ContactPoint> verts;
        float distNormZero;
        float distNormCenter;
        float distPlaneZero;
        float distPlaneCenter;
        float offset;
    };

    struct ConvexHUll6D
    {
        std::vector<ContactPoint> vertices;
        std::vector<TriangleFace6D> faces;
        float volume;
        ContactPoint center;
    };

    Quaternion getRot(const Eigen::Vector3f& from, const Eigen::Vector3f& to);
    float getAng(const Quaternion& q);
    Quaternion eigen4f2quat(const Eigen::Matrix4f& m);
    Eigen::Matrix4f quat2eigen4f(float x, float y, float z, float w);
    Eigen::Matrix4f quat2eigen4f(const Quaternion q);
    Eigen::Vector3f TPosition(const Eigen::Vector3f& pos, const Eigen::Matrix4f& m);
    float calculateQuality(float GWSOffset, float OWSOffset);
    
}

#endif
