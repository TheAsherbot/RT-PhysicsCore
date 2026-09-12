#include <iostream>

#include "RT-PhysicsCore/rendering/systems/RenderSystem.h"
#include "RT-PhysicsCore/core/ecs/core/Scene.h"
#include "RT-PhysicsCore/core/ecs/components/TransformComponent.h"

#include "RT-PhysicsCore/Utils/Log.h"

namespace RT_PhysicsCore
{
    RenderSystem::RenderSystem(Scene& scene)
        : ISystem(scene)
    {}

    void RenderSystem::RenderUpdate()
    {
        // WorldTransformComponent, not TransformComponent: a child entity's
        // local position is relative to its parent, so printing it directly
        // would report the wrong location for anything that's parented.
        // WorldTransformComponent is kept up to date by
        // TransformPropagationSystem, which must run (during UpdateSystems())
        // before this system's RenderUpdate() each frame.
        auto entities = scene.Query<WorldTransformComponent>();
        for (Entity e : entities)
        {
            auto* world = scene.GetComponent<WorldTransformComponent>(e);
            if (!world)
                continue;

			RT_LOG_INFO("Entity " << e << " position: "
				<< world->worldPosition.x << ", "
				<< world->worldPosition.y << ", "
				<< world->worldPosition.z);
        }
    }
}
