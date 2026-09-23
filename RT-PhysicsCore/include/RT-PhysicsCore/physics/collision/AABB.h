#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include "RT-PhysicsCore/physics/components/ColliderComponent.h"

namespace RT_PhysicsCore
{
    struct AABB
    {
        glm::vec3 min{0.0f};
        glm::vec3 max{0.0f};
    };

    // World-space AABB tightly enclosing a collider at the given pose.
    // Boxes use the rotate-then-reproject formula (so a rotated box still
    // gets a tight, not-too-small AABB); spheres are rotation-invariant;
    // capsules are the union of their two cap-sphere AABBs.
    AABB ComputeWorldAABB(const ColliderComponent& collider, const glm::vec3& position, const glm::quat& rotation);

    bool Overlaps(const AABB& a, const AABB& b);
}
