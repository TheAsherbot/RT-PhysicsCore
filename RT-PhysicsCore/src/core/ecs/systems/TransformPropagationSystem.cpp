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
                          glm::vec3(0.0f),
                          glm::quat(1.0f, 0.0f, 0.0f, 0.0f),
                          glm::vec3(1.0f));
            }
        }
    }

    void TransformPropagationSystem::Propagate(Entity entity,
                                                const glm::vec3& parentWorldPos,
                                                const glm::quat& parentWorldRot,
                                                const glm::vec3& parentWorldScale)
    {
        auto* local = scene.GetComponent<TransformComponent>(entity);
        if (!local)
            return;

        // worldTransform(child) = worldTransform(parent) * localTransform(child)
        glm::vec3 worldPos = parentWorldPos + parentWorldRot * (parentWorldScale * local->position);
        glm::quat worldRot = parentWorldRot * local->rotation;
        glm::vec3 worldScale = parentWorldScale * local->scale;

        WorldTransformComponent world;
        world.worldPosition = worldPos;
        world.worldRotation = worldRot;
        world.worldScale = worldScale;
        scene.AddComponent<WorldTransformComponent>(entity, world);

        auto* hierarchy = scene.GetComponent<HierarchyComponent>(entity);
        if (!hierarchy)
            return;

        for (Entity child : hierarchy->children)
        {
            Propagate(child, worldPos, worldRot, worldScale);
        }
    }
}
