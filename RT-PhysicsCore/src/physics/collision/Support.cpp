/**
 * @file Support.cpp
 * @brief Implementation of extremal support mappings and bounding radius calculations.
 */

#include "RT-PhysicsCore/physics/collision/Support.h"
#include <algorithm>
#include <cmath>

namespace RT_PhysicsCore
{
    namespace
    {
        constexpr Real kSupportEpsilon = 1e-12;
    }

    Vec3 Support::ComputeSupport(const ConvexCore& core, const Vec3& direction)
    {
        switch (core.shape)
        {
        case ColliderShape::Sphere:
        {
            // The core of a sphere is its center point. The radius is handled by coreRadius in GJK.
            return core.position;
        }

        case ColliderShape::Capsule:
        {
            // The core of a capsule is the central line segment spanning along the local +Y axis.
            // Half-length of the cylindrical section is size.y; coreRadius is size.x (capsule radius).
            Vec3 localAxis = core.rotation[1] * core.size.y;
            Vec3 p0 = core.position - localAxis;
            Vec3 p1 = core.position + localAxis;

            // Pick whichever segment endpoint projects farthest along the search direction
            return (glm::dot(direction, localAxis) >= 0.0) ? p1 : p0;
        }

        case ColliderShape::Box:
        {
            // For an oriented box with half-extents H, the support point is the corner where
            // each local axis sign matches the sign of the directional projection along that axis.
            // S_box(d) = center + sum_i(sign(d dot u_i) * H_i * u_i)
            Vec3 result = core.position;
            for (int i = 0; i < 3; ++i)
            {
                Vec3 axis = core.rotation[i];
                Real projection = glm::dot(direction, axis);
                Real sign = (projection >= 0.0) ? 1.0 : -1.0;
                result += axis * (sign * core.size[i]);
            }
            return result;
        }
        }

        return core.position;
    }

    Real Support::ComputeBoundingRadius(const ColliderComponent& collider)
    {
        switch (collider.shape)
        {
        case ColliderShape::Sphere:
        {
            return collider.size.x;
        }

        case ColliderShape::Capsule:
        {
            // Maximum distance from center to capsule tip: half-height plus hemisphere radius
            return collider.size.y + collider.size.x;
        }

        case ColliderShape::Box:
        {
            // Diagonal distance to the farthest corner
            return glm::length(collider.size);
        }
        }

        return 1.0;
    }
}