/**
 * @file NarrowPhase.h
 * @brief Exact pairwise collision detection tests between geometric primitives.
 *
 * Dispatches shape-pair queries (Sphere, Box, Capsule) using exact Separating Axis Theorem (SAT),
 * Sutherland-Hodgman polygon clipping, GJK convex distance queries, and extremal manifold reduction
 * down to at most 4 stable support points.
 */

#pragma once

#include "RT-PhysicsCore/core/math/MathTypes.h"
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
        Vec3 size{ 1.0, 1.0, 1.0 };                ///< Dimensions / extents.
        Vec3 position{ 0.0, 0.0, 0.0 };            ///< World position of collider origin.
        Mat3 rotation{ 1.0 };                      ///< World orientation matrix.
    };

    /**
     * @brief Tests for intersection or speculative proximity between two oriented geometric colliders.
     *
     * Returns true if bodies are overlapping (signedDistance < 0) or within the speculative margin
     * (0 <= signedDistance <= speculativeMargin). Automatically reduces generated manifolds down
     * to at most 4 extremal points.
     *
     * @param a World pose and geometry of first collider.
     * @param b World pose and geometry of second collider.
     * @param[out] outContact Populated collision manifold (normal points from a to b).
     * @param speculativeMargin Maximum positive gap distance within which speculative contacts are formed.
     * @return True if shapes generate contacts within the threshold; false otherwise.
     */
    bool TestCollision(const ColliderPose& a, const ColliderPose& b, Contact& outContact, Real speculativeMargin = 0.0);
}