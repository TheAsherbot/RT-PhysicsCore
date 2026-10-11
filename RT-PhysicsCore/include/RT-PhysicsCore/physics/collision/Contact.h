/**
 * @file Contact.h
 * @brief Collision manifold and high-precision contact point definitions.
 */

#pragma once

#include "RT-PhysicsCore/core/ecs/core/Entity.h"
#include "RT-PhysicsCore/core/math/MathTypes.h"

namespace RT_PhysicsCore
{
    /**
     * @brief Maximum number of active contact points retained per manifold.
     */
    constexpr int kMaxContactPoints = 4;

    /**
     * @struct ContactPoint
     * @brief Represents an individual contact point within a manifold.
     */
    struct ContactPoint
    {
        Vec3 position{ 0.0, 0.0, 0.0 };         ///< World-space contact position.
        Vec3 localAnchorA{ 0.0, 0.0, 0.0 };     ///< Local position relative to entity A center of mass.
        Vec3 localAnchorB{ 0.0, 0.0, 0.0 };     ///< Local position relative to entity B center of mass.
        Real signedDistance{ 0.0 };             ///< Signed separation: <0 is penetration, >0 is speculative gap.
        Real normalImpulse{ 0.0 };              ///< Accumulated normal constraint impulse.
        Real tangentImpulse1{ 0.0 };            ///< Accumulated tangent impulse along tangent direction 1.
        Real tangentImpulse2{ 0.0 };            ///< Accumulated tangent impulse along tangent direction 2.
        bool isStuck{ false };                  ///< Flag indicating whether the contact is in static friction regime.
    };

    /**
     * @struct Contact
     * @brief Manifold representing persistent contact between two entities.
     */
    struct Contact
    {
        Entity a{};                             ///< First entity in the interaction.
        Entity b{};                             ///< Second entity in the interaction.

        Vec3 normal{ 0.0, 1.0, 0.0 };           ///< Unit contact normal pointing from entity A toward entity B.
        Vec3 tangent1{ 1.0, 0.0, 0.0 };         ///< Primary orthonormal tangent vector.
        Vec3 tangent2{ 0.0, 0.0, 1.0 };         ///< Secondary orthonormal tangent vector.

        ContactPoint points[kMaxContactPoints]{}; ///< Array of reduced contact points.
        int pointCount{ 0 };                    ///< Number of valid contact points in manifold [0, 4].

        Real restitution{ 0.0 };                ///< Combined coefficient of restitution.
        Real staticFriction{ 0.0 };             ///< Combined coefficient of static friction.
        Real kineticFriction{ 0.0 };            ///< Combined coefficient of kinetic friction.
    };
}