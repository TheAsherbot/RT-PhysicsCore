/**
 * @file MeshComponent.h
 * @brief Visual mesh primitive representation and rendering properties.
 *
 * Defines the geometric appearance and base surface color of an entity.
 */

#pragma once

#include <glm/glm.hpp>

namespace RT_PhysicsCore
{
    /**
     * @enum PrimitiveShape
     * @brief Visual primitive shapes supported by the built-in Renderer.
     *
     * Kept separate from ColliderShape to avoid coupling rendering representation
     * with physical collision volumes (e.g., visual-only ground planes).
     */
    enum class PrimitiveShape
    {
        Cube,   ///< Unit cube centered at origin.
        Sphere, ///< UV sphere centered at origin.
        Plane   ///< Unit quad on the XZ plane facing +Y.
    };

    /**
     * @struct MeshComponent
     * @brief Visual mesh descriptor attached to renderable entities.
     *
     * Dimensions and extents are driven directly by TransformComponent::scale.
     */
    struct MeshComponent
    {
        PrimitiveShape shape{ PrimitiveShape::Cube }; ///< The geometric primitive to draw.
        glm::vec3 color{ 1.0f, 1.0f, 1.0f };          ///< Diffuse surface color multiplier (RGB).
    };
}