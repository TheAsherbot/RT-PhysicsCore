#pragma once

#include <glm/glm.hpp>
#include "RT-PhysicsCore/physics/collision/Contact.h"
#include "RT-PhysicsCore/physics/components/ColliderComponent.h"

namespace RT_PhysicsCore
{
    // World-space shape + pose, bundled for narrow-phase tests.
    struct ColliderPose
    {
        ColliderShape shape{ColliderShape::Box};
        glm::vec3 size{1.0f};
        glm::vec3 position{0.0f};
        glm::mat3 rotation{1.0f};
    };

    // Fills outContact and returns true if a and b overlap. normal points
    // from a toward b. Dispatches to the matching shape-pair test.
    bool TestCollision(const ColliderPose& a, const ColliderPose& b, Contact& outContact);
}
