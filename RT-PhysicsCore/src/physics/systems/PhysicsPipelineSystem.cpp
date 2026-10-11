/**
 * @file PhysicsPipelineSystem.cpp
 * @brief Implementation of the master substepping physics pipeline loop.
 */

#include "RT-PhysicsCore/physics/systems/PhysicsPipelineSystem.h"
#include <algorithm>

namespace RT_PhysicsCore
{
    PhysicsPipelineSystem::PhysicsPipelineSystem(Scene& scene, ResolutionSystem::SolverMode solverMode, int substeps)
        : ISystem(scene),
        substepCount(std::max(1, substeps)),
        physicsSystem(scene),
        collisionSystem(scene),
        resolutionSystem(scene, collisionSystem, solverMode),
        continuousCollisionSystem(scene)
    {}

    PhysicsPipelineSystem::~PhysicsPipelineSystem()
    {}

    void PhysicsPipelineSystem::SetSubsteps(int substeps)
    {
        substepCount = std::max(1, substeps);
    }

    int PhysicsPipelineSystem::GetSubsteps() const
    {
        return substepCount;
    }

    void PhysicsPipelineSystem::SetSpeculativeMargin(Real margin)
    {
        speculativeMargin = margin;
    }

    PhysicsSystem& PhysicsPipelineSystem::GetPhysicsSystem()
    {
        return physicsSystem;
    }

    CollisionSystem& PhysicsPipelineSystem::GetCollisionSystem()
    {
        return collisionSystem;
    }

    ResolutionSystem& PhysicsPipelineSystem::GetResolutionSystem()
    {
        return resolutionSystem;
    }

    ContinuousCollisionSystem& PhysicsPipelineSystem::GetContinuousCollisionSystem()
    {
        return continuousCollisionSystem;
    }

    void PhysicsPipelineSystem::FixedUpdate(double dt)
    {
        double h = dt / static_cast<double>(substepCount);

        for (int step = 0; step < substepCount; ++step)
        {
            // 1. Integrate forces and torques into velocities without moving positions
            physicsSystem.IntegrateVelocities(h);

            // 2. Detect collisions with swept AABB broad phase and speculative narrow phase
            collisionSystem.Detect(h, speculativeMargin);

            // 3. Resolve constraints (velocity, restitution, split-impulse penetration)
            resolutionSystem.Resolve(h, collisionSystem.GetContactCache());

            // 4. Continuous collision detection: bilateral shape casting and time-stealing clamps
            continuousCollisionSystem.ExecuteCCD(h, speculativeMargin);

            // 5. Integrate positions and DLM rotations using solved motion fractions and pseudo-velocities
            physicsSystem.IntegratePositions(h);
        }
    }
}