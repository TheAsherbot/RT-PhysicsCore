#include "RT-PhysicsCore/core/ecs/systems/PhysicsSystem.h"
#include "RT-PhysicsCore/core/ecs/core/Scene.h"
#include "RT-PhysicsCore/core/ecs/components/TransformComponent.h"
#include "RT-PhysicsCore/core/ecs/components/RigidBodyComponent.h"

namespace RT_PhysicsCore
{
    PhysicsSystem::PhysicsSystem(Scene& scene)
        : ISystem(scene)
    {}

    void PhysicsSystem::FixedUpdate(double dt)
    {
        auto entities = scene.Query<TransformComponent, RigidBodyComponent>();
        for (Entity e : entities)
        {
            auto* transform = scene.GetComponent<TransformComponent>(e);
            auto* body = scene.GetComponent<RigidBodyComponent>(e);
            if (!transform || !body)
                continue;

            if (body->mass <= 0.0f)
            {
                // Zero (or negative) mass is treated as infinite mass / static,
                // the usual physics-engine convention - dividing by it would
                // otherwise produce inf/NaN. Drop any accumulated force so it
                // doesn't leak into a later frame if the mass changes.
                body->forceAccum = glm::vec3(0.0f);
                continue;
            }

            glm::vec3 acceleration = body->forceAccum / body->mass;
            body->velocity += acceleration * static_cast<float>(dt);
            transform->position += body->velocity * static_cast<float>(dt);
            body->forceAccum = glm::vec3(0.0f);
        }
    }
}
