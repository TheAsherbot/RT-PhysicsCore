#pragma once

#include <glm/glm.hpp>

namespace RT_PhysicsCore
{
    // Placeholder shape enum - swap this out for your engine's existing
    // collision shape type/enum if one already exists.
    enum class ColliderShape
    {
        Box,
        Sphere,
        Capsule
    };

    struct ColliderComponent
    {
        ColliderShape shape{ColliderShape::Box};
        glm::vec3 size{1.0f, 1.0f, 1.0f}; // half-extents (Box), size.x = radius (Sphere/Capsule)
    };
}
