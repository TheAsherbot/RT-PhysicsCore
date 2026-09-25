#include "RT-PhysicsCore/physics/systems/CollisionSystem.h"
#include "RT-PhysicsCore/core/ecs/core/Scene.h"
#include "RT-PhysicsCore/core/ecs/components/TransformComponent.h"
#include "RT-PhysicsCore/physics/components/ColliderComponent.h"
#include "RT-PhysicsCore/physics/collision/AABB.h"
#include "RT-PhysicsCore/physics/collision/NarrowPhase.h"

#include <glm/gtc/quaternion.hpp>

namespace RT_PhysicsCore
{
    namespace
    {
        struct Entry
        {
            Entity entity;
            AABB aabb;
            ColliderPose pose;
        };
    }

    CollisionSystem::CollisionSystem(Scene& scene)
        : ISystem(scene)
    {}

    const std::vector<Contact>& CollisionSystem::GetContacts() const
    {
        return contacts;
    }

    void CollisionSystem::FixedUpdate(double /*dt*/)
    {
        contacts.clear();

        auto entities = scene.Query<TransformComponent, ColliderComponent>();

        std::vector<Entry> entries;
        entries.reserve(entities.size());

        for (Entity e : entities)
        {
            auto* transform = scene.GetComponent<TransformComponent>(e);
            auto* collider = scene.GetComponent<ColliderComponent>(e);
            if (!transform || !collider)
                continue;

            ColliderPose pose;
            pose.shape = collider->shape;
            pose.size = collider->size;
            pose.position = transform->position;
            pose.rotation = glm::mat3_cast(transform->rotation);

            entries.push_back({ e, ComputeWorldAABB(*collider, transform->position, transform->rotation), pose });
        }

        // Naive O(n^2) broad phase - fine at current body counts. Swap in
        // sweep-and-prune later if it becomes the bottleneck; it wouldn't
        // change which pairs end up colliding, only how fast non-colliding
        // ones get rejected.
        for (size_t i = 0; i < entries.size(); ++i)
        {
            for (size_t j = i + 1; j < entries.size(); ++j)
            {
                if (!Overlaps(entries[i].aabb, entries[j].aabb))
                    continue;

                Contact contact;
                if (TestCollision(entries[i].pose, entries[j].pose, contact))
                {
                    contact.a = entries[i].entity;
                    contact.b = entries[j].entity;
                    contacts.push_back(contact);
                }
            }
        }
    }
}
