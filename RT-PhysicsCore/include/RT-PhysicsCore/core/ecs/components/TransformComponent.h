/**
 * @file TransformComponent.h
 * @brief High-precision spatial transform representations for local and world coordinates.
 */

#pragma once

#include "RT-PhysicsCore/core/math/MathTypes.h"

namespace RT_PhysicsCore
{
    /**
     * @struct TransformComponent
     * @brief Local-space translation, rotation, and scaling relative to parent entity.
     */
    struct TransformComponent
    {
        Vec3 position{ 0.0, 0.0, 0.0 };         ///< Local position offset in meters.
        Quat rotation{ 1.0, 0.0, 0.0, 0.0 };    ///< Local orientation quaternion (w, x, y, z).
        Vec3 scale{ 1.0, 1.0, 1.0 };            ///< Local scaling factors along local axes.
    };

    /**
     * @struct WorldTransformComponent
     * @brief Computed global-space transformation, updated by TransformPropagationSystem.
     */
    struct WorldTransformComponent
    {
        Vec3 worldPosition{ 0.0, 0.0, 0.0 };         ///< World-space origin position in meters.
        Quat worldRotation{ 1.0, 0.0, 0.0, 0.0 };    ///< World-space absolute orientation quaternion.
        Vec3 worldScale{ 1.0, 1.0, 1.0 };            ///< World-space compound scale factors.
    };
}