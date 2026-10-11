/**
 * @file ManifoldReduction.cpp
 * @brief Implementation of planar 4-point manifold reduction using area maximization.
 */

#include "RT-PhysicsCore/physics/collision/ManifoldReduction.h"
#include <algorithm>
#include <limits>

namespace RT_PhysicsCore
{
    int ManifoldReduction::Reduce(const ContactPoint* inPoints, int inCount, const Vec3& normal, ContactPoint* outPoints)
    {
        if (inCount <= kMaxContactPoints)
        {
            for (int i = 0; i < inCount; ++i)
            {
                outPoints[i] = inPoints[i];
            }
            return inCount;
        }

        // 1. Deepest point (minimum signed distance)
        int idx0 = 0;
        Real minDistance = inPoints[0].signedDistance;
        for (int i = 1; i < inCount; ++i)
        {
            if (inPoints[i].signedDistance < minDistance)
            {
                minDistance = inPoints[i].signedDistance;
                idx0 = i;
            }
        }
        outPoints[0] = inPoints[idx0];

        // 2. Point farthest from Point 0
        int idx1 = -1;
        Real maxDistSq = -1.0;
        for (int i = 0; i < inCount; ++i)
        {
            if (i == idx0)
            {
                continue;
            }
            Vec3 delta = inPoints[i].position - outPoints[0].position;
            Real distSq = glm::dot(delta, delta);
            if (distSq > maxDistSq)
            {
                maxDistSq = distSq;
                idx1 = i;
            }
        }

        if (idx1 == -1)
        {
            return 1;
        }
        outPoints[1] = inPoints[idx1];

        // 3. Point maximizing triangle area with (p0, p1)
        Vec3 lineDir = outPoints[1].position - outPoints[0].position;
        int idx2 = -1;
        Real maxTriangleArea = -1.0;

        for (int i = 0; i < inCount; ++i)
        {
            if (i == idx0 || i == idx1)
            {
                continue;
            }
            Vec3 delta = inPoints[i].position - outPoints[0].position;
            Vec3 crossProduct = glm::cross(lineDir, delta);
            Real area = glm::length(crossProduct);
            if (area > maxTriangleArea)
            {
                maxTriangleArea = area;
                idx2 = i;
            }
        }

        if (idx2 == -1 || maxTriangleArea < 1e-12)
        {
            return 2;
        }
        outPoints[2] = inPoints[idx2];

        // 4. Point maximizing quad area on opposite side of baseline (p0, p1)
        Vec3 triNormal = glm::cross(lineDir, outPoints[2].position - outPoints[0].position);
        int idx3 = -1;
        Real maxQuadArea = -1.0;

        for (int i = 0; i < inCount; ++i)
        {
            if (i == idx0 || i == idx1 || i == idx2)
            {
                continue;
            }

            Vec3 delta = inPoints[i].position - outPoints[0].position;
            Vec3 crossProduct = glm::cross(lineDir, delta);
            Real sign = glm::dot(crossProduct, triNormal);

            if (sign < 0.0)
            {
                Real area = glm::length(crossProduct);
                if (area > maxQuadArea)
                {
                    maxQuadArea = area;
                    idx3 = i;
                }
            }
        }

        if (idx3 == -1)
        {
            for (int i = 0; i < inCount; ++i)
            {
                if (i == idx0 || i == idx1 || i == idx2)
                {
                    continue;
                }
                Vec3 d0 = inPoints[i].position - outPoints[0].position;
                Real area = glm::length(glm::cross(lineDir, d0));
                if (area > maxQuadArea)
                {
                    maxQuadArea = area;
                    idx3 = i;
                }
            }
        }

        if (idx3 == -1)
        {
            return 3;
        }

        outPoints[3] = inPoints[idx3];
        return 4;
    }
}