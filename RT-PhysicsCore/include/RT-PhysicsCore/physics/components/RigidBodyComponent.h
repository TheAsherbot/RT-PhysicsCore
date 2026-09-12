#pragma once

#include <glm/glm.hpp>

namespace RT_PhysicsCore
{
    struct RigidBodyComponent
    {
        float mass{1.0f};
        glm::vec3 velocity{0.0f, 0.0f, 0.0f};
        glm::vec3 forceAccum{ 0.0f, 0.0f, 0.0f }; // cleared to zero every PhysicsSystem::FixedUpdate
    };
}
