/**
 * @file AABB.cpp
 * @brief Implementation of world-space AABB calculation routines and overlap tests.
 */

#include "RT-PhysicsCore/physics/collision/AABB.h"
#include <algorithm>
#include <cmath>

namespace RT_PhysicsCore
{
    namespace
    {
        AABB BoxAABB(const Vec3& halfExtents, const Vec3& position, const Mat3& rotation)
        {
            Vec3 extent = glm::abs(rotation[0]) * halfExtents.x
                + glm::abs(rotation[1]) * halfExtents.y
                + glm::abs(rotation[2]) * halfExtents.z;
            return { position - extent, position + extent };
        }

        AABB SphereAABB(Real radius, const Vec3& position)
        {
            Vec3 r(radius);
            return { position - r, position + r };
        }

        AABB CapsuleAABB(Real radius, Real halfLength, const Vec3& position, const Mat3& rotation)
        {
            Vec3 axis = rotation[1] * halfLength;
            Vec3 a = position - axis;
            Vec3 b = position + axis;

            Vec3 r(radius);
            return { glm::min(a, b) - r, glm::max(a, b) + r };
        }
    }

    AABB ComputeWorldAABB(const ColliderComponent& collider, const Vec3& position, const Quat& rotation)
    {
        Mat3 r = glm::mat3_cast(rotation);
        Vec3 center = position + r * collider.offset;

        switch (collider.shape)
        {
        case ColliderShape::Box:     return BoxAABB(collider.size, center, r);
        case ColliderShape::Sphere:  return SphereAABB(collider.size.x, center);
        case ColliderShape::Capsule: return CapsuleAABB(collider.size.x, collider.size.y, center, r);
        }
        return { center, center };
    }

    AABB InflateAABB(const AABB& aabb, Real margin)
    {
        Vec3 m(margin);
        return { aabb.min - m, aabb.max + m };
    }

    AABB ComputeSweptAABB(const AABB& aabb, const Vec3& displacement)
    {
        AABB swept;
        swept.min = glm::min(aabb.min, aabb.min + displacement);
        swept.max = glm::max(aabb.max, aabb.max + displacement);
        return swept;
    }

    bool Overlaps(const AABB& a, const AABB& b)
    {
        return (a.min.x <= b.max.x && a.max.x >= b.min.x)
            && (a.min.y <= b.max.y && a.max.y >= b.min.y)
            && (a.min.z <= b.max.z && a.max.z >= b.min.z);
    }
}