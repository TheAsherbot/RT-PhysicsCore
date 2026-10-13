/**
 * @file PhysicsSystem.cpp
 * @brief Implementation of split velocity and position integration with DLM symplectic rotational splitting.
 */

#include "RT-PhysicsCore/physics/systems/PhysicsSystem.h"
#include "RT-PhysicsCore/core/ecs/core/Scene.h"
#include "RT-PhysicsCore/core/ecs/components/TransformComponent.h"
#include "RT-PhysicsCore/physics/components/RigidBodyComponent.h"
#include <cmath>

namespace RT_PhysicsCore
{
    PhysicsSystem::PhysicsSystem(Scene& scene)
        : ISystem(scene)
    {}

    void PhysicsSystem::SetGravity(const Vec3& g)
    {
        gravity = g;
    }

    const Vec3& PhysicsSystem::GetGravity() const
    {
        return gravity;
    }

    void PhysicsSystem::FixedUpdate(double dt)
    {
        IntegrateVelocities(dt);
        IntegratePositions(dt);
    }

    void PhysicsSystem::IntegrateVelocities(double h)
    {
        auto entities = scene.Query<TransformComponent, RigidBodyComponent>();
        for (Entity e : entities)
        {
            auto* transform = scene.GetComponent<TransformComponent>(e);
            auto* body = scene.GetComponent<RigidBodyComponent>(e);
            if (!body || body->invMass <= 0.0)
            {
                if (body)
                {
                    body->forceAccum = Vec3(0.0);
                    body->torqueAccum = Vec3(0.0);
                }
                continue;
            }

            // Maintain up-to-date world-space inverse inertia tensor before computing angular velocity
            if (transform)
            {
                Mat3 rotation = glm::mat3_cast(transform->rotation);
                body->invInertiaWorld = rotation * body->invInertiaBody * glm::transpose(rotation);
            }

            // Force accumulation: gravity acts uniformly through volume without torque contribution
            body->forceAccum += body->mass * gravity;

            // Linear acceleration step
            Vec3 linearAcceleration = body->forceAccum * body->invMass;
            body->velocity += linearAcceleration * h;
            body->forceAccum = Vec3(0.0);

            // Angular momentum integration
            body->angularMomentum += body->torqueAccum * h;
            body->torqueAccum = Vec3(0.0);

            body->angularVelocity = body->invInertiaWorld * body->angularMomentum;
        }
    }

    void PhysicsSystem::IntegratePositions(double h)
    {
        auto entities = scene.Query<TransformComponent, RigidBodyComponent>();
        for (Entity e : entities)
        {
            auto* transform = scene.GetComponent<TransformComponent>(e);
            auto* body = scene.GetComponent<RigidBodyComponent>(e);
            if (!transform || !body || body->invMass <= 0.0)
            {
                continue;
            }

            Real fraction = glm::clamp(body->motionFraction, 0.0, 1.0);
            Real effectiveH = h * fraction;

            // 1. Advance linear position via physical velocity and split-impulse pseudo-velocity
            transform->position += body->velocity * effectiveH + body->pseudoVelocity * h;

            // 2. Angular integration via DLM Symplectic Splitting (Dullweber, Leimkuhler, McLachlan 1997)
            Real magL = glm::length(body->angularMomentum);
            if (magL > 1e-12)
            {
                Mat3 rotation = glm::mat3_cast(transform->rotation);
                Vec3 bodyMomentum = glm::transpose(rotation) * body->angularMomentum;

                // Strang splitting: R1(h/2) R2(h/2) R3(h) R2(h/2) R1(h/2)
                static constexpr struct
                {
                    int axis;
                    Real tauFrac;
                } kSplittingSteps[5] = {
                    { 0, 0.5 },
                    { 1, 0.5 },
                    { 2, 1.0 },
                    { 1, 0.5 },
                    { 0, 0.5 }
                };

                for (const auto& step : kSplittingSteps)
                {
                    int k = step.axis;
                    Real tau = step.tauFrac * effectiveH;
                    Real theta = tau * bodyMomentum[k] * body->invInertiaBody[k][k];

                    Real halfTheta = theta * 0.5;
                    Real cosHalf = std::cos(halfTheta);
                    Real sinHalf = std::sin(halfTheta);

                    Quat qk(1.0, 0.0, 0.0, 0.0);
                    if (k == 0) qk = Quat(cosHalf, sinHalf, 0.0, 0.0);
                    else if (k == 1) qk = Quat(cosHalf, 0.0, sinHalf, 0.0);
                    else qk = Quat(cosHalf, 0.0, 0.0, sinHalf);

                    transform->rotation = glm::normalize(transform->rotation * qk);

                    Real c = std::cos(-theta);
                    Real s = std::sin(-theta);
                    static constexpr int aIdx[3] = { 1, 2, 0 };
                    static constexpr int bIdx[3] = { 2, 0, 1 };
                    int a = aIdx[k];
                    int b = bIdx[k];

                    Real va = bodyMomentum[a];
                    Real vb = bodyMomentum[b];
                    bodyMomentum[a] = c * va - s * vb;
                    bodyMomentum[b] = s * va + c * vb;
                }
            }

            // 3. Apply split-impulse pseudo-angular rotation without altering conserved angular momentum
            Real pseudoMag = glm::length(body->pseudoAngularVelocity);
            if (pseudoMag > 1e-12)
            {
                Vec3 pseudoAxis = body->pseudoAngularVelocity / pseudoMag;
                Quat pseudoDq = glm::angleAxis(pseudoMag * h, pseudoAxis);
                transform->rotation = glm::normalize(pseudoDq * transform->rotation);
            }

            // 4. Update cached world-space inertia tensor and angular velocity
            Mat3 finalRotation = glm::mat3_cast(transform->rotation);
            body->invInertiaWorld = finalRotation * body->invInertiaBody * glm::transpose(finalRotation);
            body->angularVelocity = body->invInertiaWorld * body->angularMomentum;

            // 5. Reset per-substep transient channels
            body->pseudoVelocity = Vec3(0.0);
            body->pseudoAngularVelocity = Vec3(0.0);
            body->motionFraction = 1.0;
        }
    }
}