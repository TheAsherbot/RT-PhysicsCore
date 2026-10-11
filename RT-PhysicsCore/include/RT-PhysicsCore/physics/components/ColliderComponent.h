/**
 * @file ColliderComponent.h
 * @brief High-precision geometric collision volume profile and local offset component.
 * Defines the geometric bounding representation used by broad-phase AABB generation
 * and narrow-phase shape intersection tests.
 */

#pragma once

#include "RT-PhysicsCore/core/math/MathTypes.h"

namespace RT_PhysicsCore
{
    /**
     * @enum ColliderShape
     * @brief Supported geometric primitive shapes for discrete and continuous collision detection.
     */
    enum class ColliderShape
    {
        Box,     ///< 3D oriented bounding box defined by half-extents.
        Sphere,  ///< Sphere defined by radius.
        Capsule  ///< Cylinder capped with two hemispheres along the local +Y axis.
    };

    /**
     * @struct ColliderComponent
     * @brief Geometric collision profile attached to an entity in the scene.
     *
     * The interpretation of the `size` vector depends strictly on `shape`:
     * - Box: half-extents (half-width, half-height, half-depth) along local X, Y, Z.
     * - Sphere: radius = size.x (size.y and size.z are unused).
     * - Capsule: radius = size.x; half-length of cylindrical section = size.y; aligned with local +Y.
     */
    struct ColliderComponent
    {
        ColliderShape shape{ ColliderShape::Box };      ///< The geometric primitive shape type.
        Vec3 size{ 1.0, 1.0, 1.0 };                     ///< High-precision shape dimension parameters.

        /**
         * @brief Collider center in entity local space, relative to TransformComponent::position.
         *
         * Allows the physical collision volume to differ from the visual mesh (e.g., adding
         * thickness below a visual plane or offsetting an asymmetric collider volume from the origin).
         */
        Vec3 offset{ 0.0, 0.0, 0.0 };
    };
}