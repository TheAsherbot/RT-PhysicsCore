/**
 * @file ContactConstraint.h
 * @brief Jacobian rows, effective mass matrices, and impulse channels for contact points.
 */

#pragma once

#include "RT-PhysicsCore/core/math/MathTypes.h"
#include "RT-PhysicsCore/physics/collision/Contact.h"
#include "RT-PhysicsCore/physics/components/RigidBodyComponent.h"
#include <vector>

namespace RT_PhysicsCore
{
    /**
     * @struct ConstraintPoint
     * @brief Precomputed effective mass and momentum tracking data for a single contact constraint point.
     */
    struct ConstraintPoint
    {
        Vec3 rA{ 0.0, 0.0, 0.0 };               ///< Lever arm vector from entity A center of mass to contact point.
        Vec3 rB{ 0.0, 0.0, 0.0 };               ///< Lever arm vector from entity B center of mass to contact point.

        Real normalEffectiveMass{ 0.0 };        ///< Inverse effective mass along normal axis (1 / (J M^-1 J^T)).
        Real tangentMass11{ 0.0 };              ///< Diagonal effective mass for tangent axis 1.
        Real tangentMass22{ 0.0 };              ///< Diagonal effective mass for tangent axis 2.
        Real tangentMass12{ 0.0 };              ///< Cross-coupling inertial mass between tangent axes.

        Real preSolveRelVelN{ 0.0 };            ///< Pre-impact approach velocity along normal direction.
        Real speculativeBias{ 0.0 };            ///< Velocity bias offsetting positive separation gaps.
        Real pseudoVelocityTarget{ 0.0 };       ///< Velocity bias targeted during split-impulse penetration recovery.

        Real normalImpulse{ 0.0 };              ///< Converged normal impulse.
        Real tangentImpulse1{ 0.0 };            ///< Converged tangent impulse along tangent 1.
        Real tangentImpulse2{ 0.0 };            ///< Converged tangent impulse along tangent 2.
        Real pseudoNormalImpulse{ 0.0 };        ///< Accumulated pseudo-impulse for penetration recovery.

        Real restitution{ 0.0 };                ///< Effective restitution coefficient.
        Real staticFriction{ 0.0 };             ///< Effective static friction coefficient.
        Real kineticFriction{ 0.0 };            ///< Effective kinetic friction coefficient.
        bool isStuck{ false };                  ///< Whether contact regime is static sticking or dynamic sliding.
    };

    /**
     * @struct ContactConstraint
     * @brief Precomputed contact constraint container bound to a pair of interacting bodies.
     */
    struct ContactConstraint
    {
        RigidBodyComponent* bodyA{ nullptr };   ///< Pointer to RigidBodyComponent of body A.
        RigidBodyComponent* bodyB{ nullptr };   ///< Pointer to RigidBodyComponent of body B.
        Contact* originalContact{ nullptr };    ///< Source contact manifold reference for write-back.

        Vec3 normal{ 0.0, 1.0, 0.0 };           ///< Shared unit normal directed from body A to body B.
        Vec3 tangent1{ 1.0, 0.0, 0.0 };         ///< Primary tangent vector orthogonal to normal.
        Vec3 tangent2{ 0.0, 0.0, 1.0 };         ///< Secondary tangent vector orthogonal to normal.

        std::vector<ConstraintPoint> points;    ///< Precomputed constraint points.
    };
}