/**
 * @file TransformComponent.h
 * @brief Local and world-space spatial transform representations.
 *
 * TransformComponent defines local pose relative to parent, while
 * WorldTransformComponent stores absolute space coordinates calculated
 * by TransformPropagationSystem.
 */

#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace RT_PhysicsCore
{
    /**
     * @struct TransformComponent
     * @brief Local-space translation, rotation, and scaling relative to parent entity.
     */
    struct TransformComponent
    {
        glm::vec3 position{ 0.0f, 0.0f, 0.0f };      ///< Local position offset.
        glm::quat rotation{ 1.0f, 0.0f, 0.0f, 0.0f }; ///< Local rotation quaternion (w, x, y, z).
        glm::vec3 scale{ 1.0f, 1.0f, 1.0f };         ///< Local scale factors.
    };

    /**
     * @struct WorldTransformComponent
     * @brief Computed global-space transformation, updated by TransformPropagationSystem.
     */
    struct WorldTransformComponent
    {
        glm::vec3 worldPosition{ 0.0f, 0.0f, 0.0f };      ///< World-space origin position.
        glm::quat worldRotation{ 1.0f, 0.0f, 0.0f, 0.0f }; ///< World-space absolute orientation.
        glm::vec3 worldScale{ 1.0f, 1.0f, 1.0f };         ///< World-space compound scale.
    };
}