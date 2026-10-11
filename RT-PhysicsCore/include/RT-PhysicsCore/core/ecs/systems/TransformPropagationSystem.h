/**
 * @file TransformPropagationSystem.h
 * @brief System that calculates WorldTransformComponents from root entities down child trees.
 */

#pragma once

#include "RT-PhysicsCore/core/math/MathTypes.h"
#include "RT-PhysicsCore/core/ecs/core/System.h"
#include "RT-PhysicsCore/core/ecs/core/Entity.h"

namespace RT_PhysicsCore
{
    /**
     * @class TransformPropagationSystem
     * @brief Traverses scene hierarchies and updates WorldTransformComponent for all entities.
     */
    class TransformPropagationSystem : public ISystem
    {
    public:
        /**
         * @brief Constructs the transform propagation system.
         * @param scene The Scene whose transform hierarchy will be evaluated.
         */
        explicit TransformPropagationSystem(Scene& scene);

        /**
         * @brief Traverses all root entities and recursively updates world transforms.
         */
        void Update() override;

    private:
        /**
         * @brief Recursively propagates world-space transforms down entity hierarchy trees.
         * @param entity Target entity to update.
         * @param parentWorldPos Accumulated parent world position.
         * @param parentWorldRot Accumulated parent world rotation.
         * @param parentWorldScale Accumulated parent world scale.
         */
        void Propagate(Entity entity,
            const Vec3& parentWorldPos,
            const Quat& parentWorldRot,
            const Vec3& parentWorldScale);
    };
}