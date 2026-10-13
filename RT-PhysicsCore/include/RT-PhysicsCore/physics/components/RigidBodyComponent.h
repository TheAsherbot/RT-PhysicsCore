/**
 * @file RigidBodyComponent.h
 * @brief High-precision 6-DOF rigid body dynamics state component.
 *
 * Position and orientation remain on TransformComponent; this component holds
 * mass properties, linear/angular momentum, and per-step accumulators.
 */

#pragma once

#include "RT-PhysicsCore/core/math/MathTypes.h"

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
        Real mass{ 1.0f };                     ///< Total mass in kilograms.
        Real invMass{ 1.0f };                  ///< Inverse mass (0.0 = static/immovable).

        Mat3 inertiaBody{ 1.0f };              ///< Principal body-space inertia tensor about center of mass.
        Mat3 invInertiaBody{ 1.0f };           ///< Inverse body-space inertia tensor.

        Vec3 velocity{ 0.0f, 0.0f, 0.0f };     ///< Linear velocity in world space (m/s).
        Vec3 forceAccum{ 0.0f, 0.0f, 0.0f };   ///< Accumulated world forces for current step (cleared each tick).

        Vec3 angularMomentum{ 0.0f, 0.0f, 0.0f }; ///< Angular momentum in world space (conserved quantity).
        Vec3 torqueAccum{ 0.0f, 0.0f, 0.0f };     ///< Accumulated world torques for current step (cleared each tick).

        // Cached quantities derived every FixedUpdate:
        Vec3 angularVelocity{ 0.0f, 0.0f, 0.0f }; ///< Derived instantaneous angular velocity (rad/s).
        Mat3 invInertiaWorld{ 1.0f };             ///< Derived world-space inverse inertia tensor (R * I^-1 * R^T).

        Vec3 pseudoVelocity{ 0.0, 0.0, 0.0 };       ///< Split-impulse linear pseudo-velocity for penetration recovery.
        Vec3 pseudoAngularVelocity{ 0.0, 0.0, 0.0 };///< Split-impulse angular pseudo-velocity for penetration recovery.

        Real motionFraction{ 1.0 };                 ///< Continuous collision motion fraction [0, 1] for current substep.
    };
}