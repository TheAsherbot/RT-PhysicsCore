#include "RT-PhysicsCore/physics/systems/PhysicsSystem.h"
#include "RT-PhysicsCore/core/ecs/core/Scene.h"
#include "RT-PhysicsCore/core/ecs/components/TransformComponent.h"
#include "RT-PhysicsCore/physics/components/RigidBodyComponent.h"

#include <glm/gtc/quaternion.hpp>

namespace RT_PhysicsCore
{
    PhysicsSystem::PhysicsSystem(Scene& scene)
        : ISystem(scene)
    {}

    void PhysicsSystem::SetGravity(const glm::vec3& g) { gravity = g; }
    const glm::vec3& PhysicsSystem::GetGravity() const { return gravity; }

    void PhysicsSystem::FixedUpdate(double dt)
    {
        const float dtf = static_cast<float>(dt);

        auto entities = scene.Query<TransformComponent, RigidBodyComponent>();
        for (Entity e : entities)
        {
            auto* transform = scene.GetComponent<TransformComponent>(e);
            auto* body = scene.GetComponent<RigidBodyComponent>(e);
            if (!transform || !body)
                continue;

            if (body->invMass <= 0.0f)
            {
                body->forceAccum = glm::vec3(0.0f);
                body->torqueAccum = glm::vec3(0.0f);
                continue;
            }

            // Force, not a direct velocity change - dividing by mass below
            // is what makes every body fall at the same rate under gravity.
            // No torque contribution: gravity acts uniformly through the volume.
            body->forceAccum += body->mass * gravity;

            // Linear - semi-implicit Euler
            glm::vec3 linearAcceleration = body->forceAccum * body->invMass;
            body->velocity += linearAcceleration * dtf;
            transform->position += body->velocity * dtf; // uses the just-updated velocity
            body->forceAccum = glm::vec3(0.0f);

            // Angular - semi-implicit Euler. World-space inertia rebuilt from
            // the current orientation every step; see RigidBodyComponent.h
            // for why momentum (not angular velocity) is what's integrated.
            glm::mat3 rotation = glm::mat3_cast(transform->rotation);
            glm::mat3 invInertiaWorld = rotation * body->invInertiaBody * glm::transpose(rotation);

            body->angularMomentum += body->torqueAccum * dtf;
            glm::vec3 angularVelocity = invInertiaWorld * body->angularMomentum;

            glm::quat omegaQuat(0.0f, angularVelocity.x, angularVelocity.y, angularVelocity.z);
            glm::quat deltaRotation = 0.5f * dtf * (omegaQuat * transform->rotation);
            transform->rotation = glm::normalize(transform->rotation + deltaRotation);

            body->torqueAccum = glm::vec3(0.0f);
        }
    }
}
