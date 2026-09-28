#pragma once

#include <glm/glm.hpp>

namespace RT_PhysicsCore
{
    enum class ColliderShape
    {
        Box,
        Sphere,
        Capsule
    };

    // size's meaning depends on shape:
    //   Box     - half-extents (x, y, z)
    //   Sphere  - radius = size.x
    //   Capsule - radius = size.x, half-length of the cylindrical section = size.y, axis = local +Y
    struct ColliderComponent
    {
        ColliderShape shape{ ColliderShape::Box };
        glm::vec3 size{ 1.0f, 1.0f, 1.0f };

        // Collider center, in the entity's local space, relative to
        // TransformComponent::position - 0 means centered on the entity
        // like before. Lets a collision volume differ from what's
        // rendered - e.g. a thin visual ground plane backed by a thicker
        // slab extending downward, so its top surface still lines up with
        // the visible surface instead of floating above it.
        glm::vec3 offset{ 0.0f, 0.0f, 0.0f };
    };
}