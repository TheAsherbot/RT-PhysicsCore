/**
 * @file AABB.h
 * @brief Axis-Aligned Bounding Box (AABB) computation and intersection tests.
 *
 * Encloses oriented collider shapes in tight world-space bounding boxes for broad-phase
 * collision rejection.
 */

#pragma once

#include "RT-PhysicsCore/core/math/MathTypes.h"
#include "RT-PhysicsCore/physics/components/ColliderComponent.h"

namespace RT_PhysicsCore
{
    /**
     * @struct AABB
     * @brief 3D axis-aligned bounding volume defined by minimal and maximal corner coordinates.
     */
    struct AABB
    {
        Vec3 min{ 0.0, 0.0, 0.0 }; ///< Minimum extent corner (lower bounds on X, Y, Z).
        Vec3 max{ 0.0, 0.0, 0.0 }; ///< Maximum extent corner (upper bounds on X, Y, Z).
    };

    /**
     * @brief Computes a tight world-space AABB enclosing a collider at the specified pose.
     *
     * Boxes use the rotate-and-reproject projection formula; spheres are rotation-invariant;
     * capsules calculate the bounding envelope of their two hemispherical cap spheres.
     *
     * @param collider Geometry configuration and shape of the collider.
     * @param position World-space center position of the parent entity.
     * @param rotation World-space orientation of the parent entity.
     * @return World-space AABB bounding volume.
     */
    AABB ComputeWorldAABB(const ColliderComponent& collider, const Vec3& position, const Quat& rotation);

    /**
     * @brief Expands an AABB symmetrically along all three axes by a uniform margin distance.
     * @param aabb Bounding box to inflate.
     * @param margin Margin expansion distance in meters.
     * @return Inflated AABB bounding volume.
     */
    AABB InflateAABB(const AABB& aabb, Real margin);

    /**
     * @brief Computes a swept bounding box enclosing both the initial AABB and its translated end position.
     * @param aabb Initial static bounding box.
     * @param displacement Total linear displacement vector across the substep.
     * @return Swept enclosing AABB.
     */
    AABB ComputeSweptAABB(const AABB& aabb, const Vec3& displacement);

    /**
     * @brief Tests whether two Axis-Aligned Bounding Boxes overlap along all three axes.
     * @param a First bounding box.
     * @param b Second bounding box.
     * @return True if the boxes intersect or touch; false if separated along any axis.
     */
    bool Overlaps(const AABB& a, const AABB& b);
}