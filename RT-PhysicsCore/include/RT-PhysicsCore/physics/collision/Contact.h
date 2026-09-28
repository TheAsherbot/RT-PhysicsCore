#pragma once

#include <glm/glm.hpp>
#include "RT-PhysicsCore/core/ecs/core/Entity.h"

namespace RT_PhysicsCore
{
    constexpr int kMaxContactPoints = 8; // box-box face clipping can produce up to this many

    // normal points from a toward b. Multiple points (from box-box face
    // clipping) are what let a box rest flat without rocking - a single
    // point can't resist torque. Each point has its own penetration depth,
    // since incident and reference faces aren't always exactly parallel.
    struct Contact
    {
        Entity a{};
        Entity b{};

        glm::vec3 normal{ 0.0f, 1.0f, 0.0f };
        glm::vec3 points[kMaxContactPoints]{};
        float penetrations[kMaxContactPoints]{};
        int pointCount{ 0 };

        // Combined pair values (see PhysicsMaterial.h) - computed once
        // when CollisionSystem builds this contact, not re-derived by
        // ResolutionSystem every iteration.
        float restitution{ 0.0f };
        float staticFriction{ 0.0f };
        float kineticFriction{ 0.0f };
    };
}