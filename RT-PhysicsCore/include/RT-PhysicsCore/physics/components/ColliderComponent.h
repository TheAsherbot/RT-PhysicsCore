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
    };
}
