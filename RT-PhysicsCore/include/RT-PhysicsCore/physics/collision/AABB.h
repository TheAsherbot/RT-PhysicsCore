/**
 * @file AABB.h
 * @brief Axis-Aligned Bounding Box (AABB) computation and intersection tests.
 *
 * Encloses oriented collider shapes in tight world-space bounding boxes for broad-phase
 * collision rejection.
 */

#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include "RT-PhysicsCore/physics/components/ColliderComponent.h"

namespace RT_PhysicsCore
{
    /**
     * @struct AABB
     * @brief 3D axis-aligned bounding volume defined by minimal and maximal corner coordinates.
     */
    struct AABB
    {
        glm::vec3 min{ 0.0f }; ///< Minimum extent corner (lower bounds on X, Y, Z).
        glm::vec3 max{ 0.0f }; ///< Maximum extent corner (upper bounds on X, Y, Z).
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
    AABB ComputeWorldAABB(const ColliderComponent& collider, const glm::vec3& position, const glm::quat& rotation);

    /**
     * @brief Tests whether two Axis-Aligned Bounding Boxes overlap along all three axes.
     * @param a First bounding box.
     * @param b Second bounding box.
     * @return True if the boxes intersect or touch; false if separated along any axis.
     */
    bool Overlaps(const AABB& a, const AABB& b);
}