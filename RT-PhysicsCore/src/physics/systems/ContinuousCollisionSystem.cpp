/**
 * @file ContinuousCollisionSystem.cpp
 * @brief Implementation of continuous collision shape casting and time-stealing motion clamping.
 */

#include "RT-PhysicsCore/physics/systems/ContinuousCollisionSystem.h"
#include "RT-PhysicsCore/core/ecs/core/Scene.h"
#include "RT-PhysicsCore/core/ecs/components/TransformComponent.h"
#include "RT-PhysicsCore/physics/components/ColliderComponent.h"
#include "RT-PhysicsCore/physics/components/RigidBodyComponent.h"
#include "RT-PhysicsCore/physics/collision/ShapeCast.h"
#include <algorithm>
#include <cmath>
#include <vector>

namespace RT_PhysicsCore
{
    ContinuousCollisionSystem::ContinuousCollisionSystem(Scene& scene)
        : ISystem(scene)
    {}

    ContinuousCollisionSystem::~ContinuousCollisionSystem()
    {}

    void ContinuousCollisionSystem::SetMaxCastIterations(int iterations)
    {
        maxCastIterations = iterations;
    }

    void ContinuousCollisionSystem::SetTargetSkinGap(Real gap)
    {
        targetSkinGap = gap;
    }

    void ContinuousCollisionSystem::ExecuteCCD(double h, Real speculativeMargin)
    {
        auto entities = scene.Query<TransformComponent, ColliderComponent>();

        struct BodyRecord
        {
            Entity entity;
            TransformComponent* transform;
            RigidBodyComponent* body;
            ColliderComponent* collider;
            ConvexCore core;
            Real boundRadius;
            Vec3 linDisp;
        };

        std::vector<BodyRecord> fastBodies;
        std::vector<BodyRecord> allColliders;

        for (Entity e : entities)
        {
            auto* transform = scene.GetComponent<TransformComponent>(e);
            auto* collider = scene.GetComponent<ColliderComponent>(e);
            auto* body = scene.GetComponent<RigidBodyComponent>(e);

            if (!transform || !collider)
            {
                continue;
            }

            ConvexCore core;
            core.shape = collider->shape;
            core.size = collider->size;
            core.position = transform->position + (transform->rotation * collider->offset);
            core.rotation = glm::mat3_cast(transform->rotation);
            core.coreRadius = (collider->shape == ColliderShape::Sphere || collider->shape == ColliderShape::Capsule) ?
                collider->size.x : 0.0;

            Real boundRadius = Support::ComputeBoundingRadius(*collider);
            Vec3 linDisp = (body && body->invMass > 0.0) ? (body->velocity * h) : Vec3(0.0);

            BodyRecord record{ e, transform, body, collider, core, boundRadius, linDisp };
            allColliders.push_back(record);

            if (body && body->invMass > 0.0)
            {
                // Measure total potential motion displacement over the substep
                Real linearSpeed = glm::length(body->velocity);
                Real angularSpeed = glm::length(body->angularVelocity);
                Real totalMotionBound = (linearSpeed + angularSpeed * boundRadius) * h;

                // Trigger bilateral shape casting if motion exceeds speculative margin
                if (totalMotionBound > speculativeMargin)
                {
                    fastBodies.push_back(record);
                }
            }
        }

        if (fastBodies.empty())
        {
            return;
        }

        // Execute bilateral shape casting for all fast bodies against candidate scene obstacles
        for (auto& fast : fastBodies)
        {
            for (auto& target : allColliders)
            {
                if (fast.entity == target.entity)
                {
                    continue;
                }

                // Skip static-static interactions
                if ((!fast.body || fast.body->invMass <= 0.0) && (!target.body || target.body->invMass <= 0.0))
                {
                    continue;
                }

                Vec3 targetAngVel = (target.body && target.body->invMass > 0.0) ? target.body->angularVelocity : Vec3(0.0);

                ShapeCastResult castRes = ShapeCast::Cast(
                    fast.core, fast.linDisp, fast.body->angularVelocity, fast.boundRadius,
                    target.core, target.linDisp, targetAngVel, target.boundRadius,
                    h, targetSkinGap, maxCastIterations);

                if (castRes.hasHit && castRes.timeOfImpact > 1e-6 && castRes.timeOfImpact < fast.body->motionFraction)
                {
                    // Clamp body motion fraction (time stealing)
                    fast.body->motionFraction = castRes.timeOfImpact;

                    // If target is also dynamic, mirror the motion clamp to maintain relative alignment
                    if (target.body && target.body->invMass > 0.0)
                    {
                        target.body->motionFraction = std::min(target.body->motionFraction, castRes.timeOfImpact);
                    }
                }
            }
        }
    }
}