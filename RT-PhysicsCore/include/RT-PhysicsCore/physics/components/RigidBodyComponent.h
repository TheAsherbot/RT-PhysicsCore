#pragma once

#include <glm/glm.hpp>

namespace RT_PhysicsCore
{
    // Full 6-DOF rigid body dynamics state. Position/orientation stay on
    // TransformComponent - this only holds what that component doesn't:
    // mass properties, momentum, and the per-step force/torque accumulators.
    //
    // Angular state is ANGULAR MOMENTUM, not angular velocity - world-space
    // inertia rotates with the body, so momentum is what's actually
    // conserved when torque is zero (angular velocity isn't, for any
    // non-spherically-symmetric body - this is what makes asymmetric
    // tumbling look physically correct). PhysicsSystem derives angular
    // velocity from this each step.
    //
    // Plain data, matching every other component - build one via
    // MassProperties.h's factory functions rather than by hand.
    struct RigidBodyComponent
    {
        float mass{ 1.0f };
        float invMass{ 1.0f };             // 0 = infinite mass (static/kinematic)

        glm::mat3 inertiaBody{ 1.0f };      // body-space, about center of mass
        glm::mat3 invInertiaBody{ 1.0f };

        glm::vec3 velocity{ 0.0f, 0.0f, 0.0f };
        glm::vec3 forceAccum{ 0.0f, 0.0f, 0.0f };    // cleared every FixedUpdate

        glm::vec3 angularMomentum{ 0.0f, 0.0f, 0.0f };
        glm::vec3 torqueAccum{ 0.0f, 0.0f, 0.0f };   // cleared every FixedUpdate
    };
}
