#include "RT-PhysicsCore/rendering/systems/RenderSystem.h"
#include "RT-PhysicsCore/rendering/Renderer.h"
#include "RT-PhysicsCore/rendering/components/MeshComponent.h"
#include "RT-PhysicsCore/core/ecs/core/Scene.h"
#include "RT-PhysicsCore/core/ecs/components/TransformComponent.h"

namespace RT_PhysicsCore
{
    RenderSystem::RenderSystem(Scene& scene, Renderer& renderer)
        : ISystem(scene), renderer(renderer)
    {}

    void RenderSystem::RenderUpdate()
    {
        renderer.BeginFrame();

        auto entities = scene.Query<MeshComponent, WorldTransformComponent>();
        for (Entity e : entities)
        {
            auto* mesh = scene.GetComponent<MeshComponent>(e);
            auto* worldTransform = scene.GetComponent<WorldTransformComponent>(e);
            if (!mesh || !worldTransform)
                continue;

            renderer.DrawMesh(*mesh, *worldTransform);
        }

        renderer.FlushDebugDraw();
        renderer.EndFrame();
    }
}
