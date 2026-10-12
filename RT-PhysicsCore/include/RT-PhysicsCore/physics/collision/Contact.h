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

        /**
         * @brief Implicit conversion operator to high-precision position vector.
         * @return Reference to world-space contact position.
         */
        operator const Vec3& () const;

        /**
         * @brief Implicit conversion operator to high-precision position vector.
         * @return Reference to world-space contact position.
         */
        operator Vec3& ();

        /**
         * @brief Implicit conversion operator to single-precision position vector for rendering.
         * @return World-space contact position in single precision.
         */
        operator glm::vec3() const;

        /**
         * @brief Implicit conversion operator to double-precision position vector.
         * @return World-space contact position in double precision.
         */
        operator glm::dvec3() const;
    };

    /**
     * @brief Exact addition operator between a ContactPoint and a high-precision offset vector.
     * @param cp Contact point source.
     * @param v High-precision vector offset.
     * @return Displaced world position.
     */
    Vec3 operator+(const ContactPoint& cp, const Vec3& v);

    /**
     * @brief Exact addition operator between a high-precision offset vector and a ContactPoint.
     * @param v High-precision vector offset.
     * @param cp Contact point source.
     * @return Displaced world position.
     */
    Vec3 operator+(const Vec3& v, const ContactPoint& cp);

    /**
     * @brief Exact subtraction operator between a ContactPoint and a high-precision offset vector.
     * @param cp Contact point source.
     * @param v High-precision vector offset.
     * @return Displaced world position.
     */
    Vec3 operator-(const ContactPoint& cp, const Vec3& v);

    /**
     * @brief Exact subtraction operator between a high-precision offset vector and a ContactPoint.
     * @param v High-precision vector offset.
     * @param cp Contact point source.
     * @return Displaced world position.
     */
    Vec3 operator-(const Vec3& v, const ContactPoint& cp);

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