/**
 * @file Support.h
 * @brief Extremal support mapping functions for convex geometric cores.
 */

#pragma once

#include "RT-PhysicsCore/core/math/MathTypes.h"
#include "RT-PhysicsCore/physics/components/ColliderComponent.h"

namespace RT_PhysicsCore
{
    /**
     * @struct ConvexCore
     * @brief Encapsulates a convex shape's core geometry and its Minkowski spherical expansion radius.
     */
    struct ConvexCore
    {
        ColliderShape shape{ ColliderShape::Box };  ///< Primitive shape type.
        Vec3 size{ 1.0, 1.0, 1.0 };                 ///< Dimensional parameters (half-extents, radii).
        Vec3 position{ 0.0, 0.0, 0.0 };             ///< World-space center position of the collider.
        Mat3 rotation{ 1.0 };                       ///< World-space 3x3 orthonormal orientation matrix.
        Real coreRadius{ 0.0 };                     ///< Spherical expansion radius added to the core (Minkowski sum radius).
    };

    /**
     * @class Support
     * @brief Evaluates extremal support mappings and bounding limits for convex shapes.
     */
    class Support
    {
    public:
        /**
         * @brief Evaluates the extremal support point on a convex core in a given world direction.
         *
         * Computes the point S_A(d) on the shape that maximizes the directional projection (p dot d).
         * For composite shapes (e.g., spheres and capsules), this evaluates the support on the
         * underlying degenerate core (point or line segment), leaving the spherical rounding radius
         * to be handled analytically by GJK.
         *
         * @param core Convex shape core specification and pose.
         * @param direction World-space directional search vector.
         * @return Extremal support point in world-space coordinates.
         */
        static Vec3 ComputeSupport(const ConvexCore& core, const Vec3& direction);

        /**
         * @brief Computes the maximum bounding radius from the body's center of mass to its farthest surface point.
         *
         * Used to compute conservative advancement angular closing speed bounds:
         * bound = ||v_rel dot n|| + ||omega_A|| * rA + ||omega_B|| * rB.
         *
         * @param collider Collider geometry specifications.
         * @return Maximum Euclidean radius from the collider origin in meters.
         */
        static Real ComputeBoundingRadius(const ColliderComponent& collider);
    };
}