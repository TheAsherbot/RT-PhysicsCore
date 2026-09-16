#pragma once

#include <glm/glm.hpp>

namespace RT_PhysicsCore
{
    // Deliberately its own enum, not ColliderShape: what an entity looks
    // like and what it collides as are different concerns. They happen to
    // overlap today (Box/Sphere) but shouldn't be coupled - a visual-only
    // "Plane" ground mesh has no physical equivalent here, and a future
    // physics-only convex hull collider wouldn't need a matching visual.
    enum class PrimitiveShape
    {
        Cube,
        Sphere,
        Plane
    };

    struct MeshComponent
    {
        PrimitiveShape shape{PrimitiveShape::Cube};
        glm::vec3 color{1.0f, 1.0f, 1.0f};
        // Sizing comes from TransformComponent::scale, not a separate field
        // here - one source of truth for "how big is this entity."
    };
}
