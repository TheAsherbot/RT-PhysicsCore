/**
 * @file TransformPropagationSystem.cpp
 * @brief Implementation of recursive world transform hierarchy propagation.
 */

#include "RT-PhysicsCore/core/ecs/systems/TransformPropagationSystem.h"
#include "RT-PhysicsCore/core/ecs/core/Scene.h"
#include "RT-PhysicsCore/core/ecs/components/TransformComponent.h"
#include "RT-PhysicsCore/core/ecs/components/HierarchyComponent.h"

namespace RT_PhysicsCore
{
    TransformPropagationSystem::TransformPropagationSystem(Scene& scene)
        : ISystem(scene)
    {}

    void TransformPropagationSystem::Update()
    {
        auto entitiesWithTransform = scene.Query<TransformComponent>();
        for (Entity e : entitiesWithTransform)
        {
            auto* hierarchy = scene.GetComponent<HierarchyComponent>(e);
            if (!hierarchy || hierarchy->parent == invalidEntity)
            {
                Propagate(e,
                    Vec3(0.0f),
                    Quat(1.0f, 0.0f, 0.0f, 0.0f),
                    Vec3(1.0f));
            }
        }
    }

    void TransformPropagationSystem::Propagate(Entity entity,
        const Vec3& parentWorldPos,
        const Quat& parentWorldRot,
        const Vec3& parentWorldScale)
    {
        auto* local = scene.GetComponent<TransformComponent>(entity);
        if (!local)
        {
            return;
        }

        // worldTransform(child) = worldTransform(parent) * localTransform(child)
        Vec3 worldPos = parentWorldPos + parentWorldRot * (parentWorldScale * local->position);
        Quat worldRot = parentWorldRot * local->rotation;
        Vec3 worldScale = parentWorldScale * local->scale;

        WorldTransformComponent world;
        world.worldPosition = worldPos;
        world.worldRotation = worldRot;
        world.worldScale = worldScale;
        scene.AddComponent<WorldTransformComponent>(entity, world);

        auto* hierarchy = scene.GetComponent<HierarchyComponent>(entity);
        if (!hierarchy)
        {
            return;
        }

        for (Entity child : hierarchy->children)
        {
            Propagate(child, worldPos, worldRot, worldScale);
        }
    }
}