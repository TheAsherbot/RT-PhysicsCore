/**
 * @file RigidBodyComponent.h
 * @brief Full 6-DOF rigid body dynamics state component.
 *
 * Position and orientation remain on TransformComponent; this component holds
 * mass properties, linear/angular momentum, and per-step accumulators.
 */

#pragma once

#include <glm/glm.hpp>

namespace RT_PhysicsCore
{
    /**
     * @struct RigidBodyComponent
     * @brief 6-DOF physical state and momentum storage.
     *
     * Angular state is tracked via ANGULAR MOMENTUM rather than angular velocity.
     * Because world-space inertia rotates with the body, momentum is what remains
     * conserved under zero torque (producing physically correct intermediate-axis tumbling).
     */
    struct RigidBodyComponent
    {
        float mass{ 1.0f };                     ///< Total mass in kilograms.
        float invMass{ 1.0f };                  ///< Inverse mass (0.0 = static/immovable).

        glm::mat3 inertiaBody{ 1.0f };          ///< Principal body-space inertia tensor about center of mass.
        glm::mat3 invInertiaBody{ 1.0f };       ///< Inverse body-space inertia tensor.

        glm::vec3 velocity{ 0.0f, 0.0f, 0.0f };     ///< Linear velocity in world space (m/s).
        glm::vec3 forceAccum{ 0.0f, 0.0f, 0.0f };   ///< Accumulated world forces for current step (cleared each tick).

        glm::vec3 angularMomentum{ 0.0f, 0.0f, 0.0f }; ///< Angular momentum in world space (conserved quantity).
        glm::vec3 torqueAccum{ 0.0f, 0.0f, 0.0f };     ///< Accumulated world torques for current step (cleared each tick).

        // Cached quantities derived every FixedUpdate:
        glm::vec3 angularVelocity{ 0.0f, 0.0f, 0.0f }; ///< Derived instantaneous angular velocity (rad/s).
        glm::mat3 invInertiaWorld{ 0.0f };             ///< Derived world-space inverse inertia tensor (R * I^-1 * R^T).
    };
}