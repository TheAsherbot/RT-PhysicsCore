/**
 * @file ColliderComponent.h
 * @brief Collision volume component defining shape geometry and local offset.
 *
 * Defines the geometric bounding representation used by broad-phase AABB generation
 * and narrow-phase shape intersection tests.
 */

#pragma once

#include <glm/glm.hpp>

namespace RT_PhysicsCore
{
    /**
     * @enum ColliderShape
     * @brief Supported geometric primitive shapes for collision detection.
     */
    enum class ColliderShape
    {
        Box,     ///< 3D oriented box defined by half-extents.
        Sphere,  ///< Sphere defined by radius.
        Capsule  ///< Cylinder capped with two hemispheres along the local +Y axis.
    };

    /**
     * @struct ColliderComponent
     * @brief Geometric collision profile attached to an entity.
     *
     * The interpretation of the `size` vector depends on `shape`:
     * - Box: half-extents (half-width, half-height, half-depth) along local X, Y, Z.
     * - Sphere: radius = size.x.
     * - Capsule: radius = size.x; half-length of cylindrical section = size.y; aligned with local +Y.
     */
    struct ColliderComponent
    {
        ColliderShape shape{ ColliderShape::Box };      ///< The geometric primitive shape type.
        glm::vec3 size{ 1.0f, 1.0f, 1.0f };             ///< Shape dimension parameters.

        /**
         * @brief Collider center in entity local space, relative to TransformComponent::position.
         *
         * Allows the physical collision volume to differ from the visual mesh (e.g., adding
         * thickness below a thin visual ground plane to prevent tunneling while preserving surface alignment).
         */
        glm::vec3 offset{ 0.0f, 0.0f, 0.0f };
    };
}