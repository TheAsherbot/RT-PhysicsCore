/**
 * @file NarrowPhase.h
 * @brief Exact pairwise collision detection tests between geometric primitives.
 *
 * Dispatches shape-pair queries (Sphere, Box, Capsule) using exact Separating
 * Axis Theorem (SAT), Sutherland-Hodgman polygon clipping, and alternating projections.
 */

#pragma once

#include <glm/glm.hpp>
#include "RT-PhysicsCore/physics/collision/Contact.h"
#include "RT-PhysicsCore/physics/components/ColliderComponent.h"

namespace RT_PhysicsCore
{
    /**
     * @struct ColliderPose
     * @brief Bundles a shape descriptor with its world-space position and rotation matrix.
     */
    struct ColliderPose
    {
        ColliderShape shape{ ColliderShape::Box }; ///< The geometric primitive shape.
        glm::vec3 size{ 1.0f };                    ///< Dimensions / extents.
        glm::vec3 position{ 0.0f };                ///< World position of collider origin.
        glm::mat3 rotation{ 1.0f };                ///< World orientation matrix.
    };

    /**
     * @brief Tests for intersection between two oriented geometric colliders.
     * @param a World pose and geometry of first collider.
     * @param b World pose and geometry of second collider.
     * @param[out] outContact Populated collision manifold (normal points from a to b).
     * @return True if shapes overlap and generate contacts; false otherwise.
     */
    bool TestCollision(const ColliderPose& a, const ColliderPose& b, Contact& outContact);
}