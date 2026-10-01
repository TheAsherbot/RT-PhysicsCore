/**
 * @file Contact.h
 * @brief Collision manifold and contact point structures.
 *
 * Encapsulates contact manifold geometry between two colliding entities, including
 * normal direction, manifold points, penetration depths, and cached interaction parameters.
 */

#pragma once

#include <glm/glm.hpp>
#include "RT-PhysicsCore/core/ecs/core/Entity.h"

namespace RT_PhysicsCore
{
    /**
     * @brief Maximum number of contact points supported per collision manifold.
     * @note Polygon face clipping during box-box collisions can produce up to 8 vertices.
     */
    constexpr int kMaxContactPoints = 8;

    /**
     * @struct Contact
     * @brief Manifold representing physical contact between two colliding entities.
     *
     * The normal points directed from entity `a` toward entity `b`. Multiple points
     * allow planar resting contacts (such as a box on a plane) to resist rotational torques.
     */
    struct Contact
    {
        Entity a{}; ///< First entity involved in the collision.
        Entity b{}; ///< Second entity involved in the collision.

        glm::vec3 normal{ 0.0f, 1.0f, 0.0f };               ///< Unit normal pointing from entity a to entity b.
        glm::vec3 points[kMaxContactPoints]{};               ///< World-space contact positions.
        float penetrations[kMaxContactPoints]{};            ///< Penetration depth at each contact point (positive).
        int pointCount{ 0 };                                ///< Number of active contact points in the manifold [0, 8].

        // Combined material properties pre-calculated once during collision detection
        float restitution{ 0.0f };                          ///< Effective coefficient of restitution for this contact.
        float staticFriction{ 0.0f };                       ///< Effective coefficient of static friction.
        float kineticFriction{ 0.0f };                      ///< Effective coefficient of kinetic friction.
    };
}