/**
 * @file PhysicsSystem.cpp
 * @brief Implementation of rigid body dynamics integration and force accumulation.
 */

#include "RT-PhysicsCore/physics/systems/PhysicsSystem.h"
#include "RT-PhysicsCore/core/ecs/core/Scene.h"
#include "RT-PhysicsCore/core/ecs/components/TransformComponent.h"
#include "RT-PhysicsCore/physics/components/RigidBodyComponent.h"

#include <cmath>
#include <glm/gtc/quaternion.hpp>

namespace RT_PhysicsCore
{
    PhysicsSystem::PhysicsSystem(Scene& scene)
        : ISystem(scene)
    {}

    void PhysicsSystem::SetGravity(const glm::vec3& g)
    {
        gravity = g;
    }

    const glm::vec3& PhysicsSystem::GetGravity() const
    {
        return gravity;
    }

    void PhysicsSystem::FixedUpdate(double dt)
    {
        const float dtf = static_cast<float>(dt);

        auto entities = scene.Query<TransformComponent, RigidBodyComponent>();
        for (Entity e : entities)
        {
            auto* transform = scene.GetComponent<TransformComponent>(e);
            auto* body = scene.GetComponent<RigidBodyComponent>(e);
            if (!transform || !body)
            {
                continue;
            }

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
            transform->position += body->velocity * dtf;
            body->forceAccum = glm::vec3(0.0f);

            // Angular - DLM Symplectic Splitting (Dullweber, Leimkuhler, McLachlan 1997)
            body->angularMomentum += body->torqueAccum * dtf;
            body->torqueAccum = glm::vec3(0.0f);

            float magL = glm::length(body->angularMomentum);
            if (magL > 1e-8f)
            {
                glm::mat3 rotation = glm::mat3_cast(transform->rotation);
                glm::vec3 bodyMomentum = glm::transpose(rotation) * body->angularMomentum;

                // Strang splitting: R1(h/2) R2(h/2) R3(h) R2(h/2) R1(h/2)
                static constexpr struct
                {
                    int axis;
                    float tauFrac;
                } kSplittingSteps[5] = {
                    { 0, 0.5f },
                    { 1, 0.5f },
                    { 2, 1.0f },
                    { 1, 0.5f },
                    { 0, 0.5f }
                };

                for (const auto& step : kSplittingSteps)
                {
                    int k = step.axis;
                    float tau = step.tauFrac * dtf;
                    float theta = tau * bodyMomentum[k] * body->invInertiaBody[k][k];

                    float halfTheta = theta * 0.5f;
                    float cosHalf = std::cos(halfTheta);
                    float sinHalf = std::sin(halfTheta);

                    glm::quat qk(1.0f, 0.0f, 0.0f, 0.0f);
                    if (k == 0)
                    {
                        qk = glm::quat(cosHalf, sinHalf, 0.0f, 0.0f);
                    }
                    else if (k == 1)
                    {
                        qk = glm::quat(cosHalf, 0.0f, sinHalf, 0.0f);
                    }
                    else
                    {
                        qk = glm::quat(cosHalf, 0.0f, 0.0f, sinHalf);
                    }

                    transform->rotation = glm::normalize(transform->rotation * qk);

                    // Rotate body-frame momentum vector by -theta about axis k
                    float c = std::cos(-theta);
                    float s = std::sin(-theta);
                    static constexpr int aIdx[3] = { 1, 2, 0 };
                    static constexpr int bIdx[3] = { 2, 0, 1 };
                    int a = aIdx[k];
                    int b = bIdx[k];

                    float va = bodyMomentum[a];
                    float vb = bodyMomentum[b];
                    bodyMomentum[a] = c * va - s * vb;
                    bodyMomentum[b] = s * va + c * vb;
                }
            }

            // Update cached world-space inertia tensor and angular velocity
            // using the final orientation of the step for downstream systems.
            glm::mat3 finalRotation = glm::mat3_cast(transform->rotation);
            body->invInertiaWorld = finalRotation * body->invInertiaBody * glm::transpose(finalRotation);
            body->angularVelocity = body->invInertiaWorld * body->angularMomentum;
        }
    }
}