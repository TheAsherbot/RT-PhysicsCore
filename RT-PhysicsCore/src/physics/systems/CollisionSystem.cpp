/**
 * @file CollisionSystem.cpp
 * @brief Implementation of broad-phase swept AABB filtering and narrow-phase signed collision pipeline.
 */

#include "RT-PhysicsCore/physics/systems/CollisionSystem.h"
#include "RT-PhysicsCore/core/ecs/core/Scene.h"
#include "RT-PhysicsCore/core/ecs/components/TransformComponent.h"
#include "RT-PhysicsCore/physics/components/ColliderComponent.h"
#include "RT-PhysicsCore/physics/components/PhysicsMaterialComponent.h"
#include "RT-PhysicsCore/physics/components/RigidBodyComponent.h"
#include "RT-PhysicsCore/physics/collision/AABB.h"
#include "RT-PhysicsCore/physics/collision/NarrowPhase.h"
#include "RT-PhysicsCore/physics/PhysicsMaterial.h"

namespace RT_PhysicsCore
{
    namespace
    {
        struct Entry
        {
            Entity entity;
            AABB sweptAabb;
            ColliderPose pose;
            MaterialId material;
        };
    }

    CollisionSystem::CollisionSystem(Scene& scene)
        : ISystem(scene)
    {}

    const std::vector<Contact>& CollisionSystem::GetContacts() const
    {
        return contacts;
    }

    std::vector<Contact>& CollisionSystem::GetContacts()
    {
        return contacts;
    }

    ContactCache& CollisionSystem::GetContactCache()
    {
        return contactCache;
    }

    void CollisionSystem::FixedUpdate(double dt)
    {
        Detect(dt, 0.002);
    }

    void CollisionSystem::Detect(double h, Real speculativeMargin)
    {
        contacts.clear();
        contactCache.BeginSubstep();

        auto entities = scene.Query<TransformComponent, ColliderComponent>();

        std::vector<Entry> entries;
        entries.reserve(entities.size());

        for (Entity e : entities)
        {
            auto* transform = scene.GetComponent<TransformComponent>(e);
            auto* collider = scene.GetComponent<ColliderComponent>(e);
            if (!transform || !collider)
            {
                continue;
            }

            ColliderPose pose;
            pose.shape = collider->shape;
            pose.size = collider->size;
            pose.position = transform->position + (transform->rotation * collider->offset);
            pose.rotation = glm::mat3_cast(transform->rotation);

            auto* material = scene.GetComponent<PhysicsMaterialComponent>(e);
            MaterialId materialId = material ? material->material : MaterialId::Default;

            AABB baseAabb = ComputeWorldAABB(*collider, transform->position, transform->rotation);
            baseAabb = InflateAABB(baseAabb, speculativeMargin);

            auto* body = scene.GetComponent<RigidBodyComponent>(e);
            Vec3 displacement = (body && body->invMass > 0.0) ? (body->velocity * h) : Vec3(0.0);
            AABB sweptAabb = ComputeSweptAABB(baseAabb, displacement);

            entries.push_back({ e, sweptAabb, pose, materialId });
        }

        // Broad-phase swept AABB overlap testing
        for (size_t i = 0; i < entries.size(); ++i)
        {
            auto* bodyI = scene.GetComponent<RigidBodyComponent>(entries[i].entity);
            bool dynamicI = (bodyI && bodyI->invMass > 0.0);

            for (size_t j = i + 1; j < entries.size(); ++j)
            {
                auto* bodyJ = scene.GetComponent<RigidBodyComponent>(entries[j].entity);
                bool dynamicJ = (bodyJ && bodyJ->invMass > 0.0);

                // Skip static-static candidate pairs
                if (!dynamicI && !dynamicJ)
                {
                    continue;
                }

                if (!Overlaps(entries[i].sweptAabb, entries[j].sweptAabb))
                {
                    continue;
                }

                Contact contact;
                if (TestCollision(entries[i].pose, entries[j].pose, contact, speculativeMargin))
                {
                    contact.a = entries[i].entity;
                    contact.b = entries[j].entity;

                    MaterialPairProperties mat = GetPairProperties(entries[i].material, entries[j].material);
                    contact.restitution = mat.restitution;
                    contact.staticFriction = mat.staticFriction;
                    contact.kineticFriction = mat.kineticFriction;

                    // Correlate against persistent cache for warm starting
                    contactCache.MatchAndWarmStart(contact);
                    contacts.push_back(contact);
                }
            }
        }

        contactCache.EndSubstep();
    }
}