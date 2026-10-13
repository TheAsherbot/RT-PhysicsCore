/**
 * @file ResolutionSystem.cpp
 * @brief Implementation of island constraint resolution coordinator and solver dispatch.
 */

#include "RT-PhysicsCore/physics/systems/ResolutionSystem.h"
#include "RT-PhysicsCore/physics/systems/CollisionSystem.h"
#include "RT-PhysicsCore/core/ecs/core/Scene.h"
#include "RT-PhysicsCore/core/ecs/components/TransformComponent.h"
#include "RT-PhysicsCore/physics/components/RigidBodyComponent.h"

namespace RT_PhysicsCore
{
    ResolutionSystem::ResolutionSystem(Scene& scene, CollisionSystem& collisionSystem, SolverMode initialMode)
        : ISystem(scene), collisionSystem(collisionSystem), solverMode(initialMode)
    {}

    ResolutionSystem::~ResolutionSystem()
    {}

    void ResolutionSystem::SetSolverMode(SolverMode mode)
    {
        solverMode = mode;
    }

    ResolutionSystem::SolverMode ResolutionSystem::GetSolverMode() const
    {
        return solverMode;
    }

    void ResolutionSystem::SetVelocityIterations(int iterations)
    {
        iterativeSolver.SetVelocityIterations(iterations);
    }

    void ResolutionSystem::SetPositionIterations(int iterations)
    {
        iterativeSolver.SetPositionIterations(iterations);
    }

    void ResolutionSystem::SetRestitutionVelocityThreshold(Real threshold)
    {
        iterativeSolver.SetRestitutionVelocityThreshold(threshold);
    }

    void ResolutionSystem::FixedUpdate(double dt)
    {
        Resolve(dt, collisionSystem.GetContactCache());
    }

    void ResolutionSystem::Resolve(double h, ContactCache& contactCache)
    {
        std::vector<Contact>& contacts = collisionSystem.GetContacts();
        if (contacts.empty())
        {
            return;
        }

        // 1. Partition active interaction graph into disjoint simulation islands
        std::vector<SimulationIsland> islands;
        islandBuilder.BuildIslands(scene, contacts, islands);

        ISolverBackend& backend = (solverMode == SolverMode::Exact) ?
            static_cast<ISolverBackend&>(exactSolver) : static_cast<ISolverBackend&>(iterativeSolver);

        // 2. Solve each island independently
        for (auto& island : islands)
        {
            std::vector<ContactConstraint> constraints;
            backend.PrepareConstraints(island, constraints, h);

            // Bind component pointers, center-of-mass lever arms, and pre-solve relative velocities
            for (auto& cc : constraints)
            {
                Contact* orig = cc.originalContact;
                if (!orig)
                {
                    continue;
                }

                cc.bodyA = scene.GetComponent<RigidBodyComponent>(orig->a);
                cc.bodyB = scene.GetComponent<RigidBodyComponent>(orig->b);
                auto* tA = scene.GetComponent<TransformComponent>(orig->a);
                auto* tB = scene.GetComponent<TransformComponent>(orig->b);

                for (size_t i = 0; i < cc.points.size(); ++i)
                {
                    auto& p = cc.points[i];
                    const auto& cp = orig->points[i];

                    // Transform contact points to lever arms from each body's center of mass
                    p.rA = tA ? (cp.position - tA->position) : cp.localAnchorA;
                    p.rB = tB ? (cp.position - tB->position) : cp.localAnchorB;

                    // Capture pre-solve approach speed for restitution bounce calculation
                    Vec3 vA = cc.bodyA ? (cc.bodyA->velocity + glm::cross(cc.bodyA->angularVelocity, p.rA)) : Vec3(0.0);
                    Vec3 vB = cc.bodyB ? (cc.bodyB->velocity + glm::cross(cc.bodyB->angularVelocity, p.rB)) : Vec3(0.0);
                    p.preSolveRelVelN = glm::dot(vB - vA, cc.normal);
                }
            }

            // 1. Solve dynamic restitution and shockwave propagation first
            backend.SolveRestitution(constraints, h);

            // 2. Solve unilateral non-penetration and Coulomb friction
            backend.SolveVelocity(constraints, h);

            // 3. Solve geometric penetration on split-impulse pseudo-velocities
            backend.SolvePenetration(constraints, h);
        }

        // 3. Store converged impulses into persistent cache for cross-frame warm starting
        for (const Contact& c : contacts)
        {
            contactCache.Store(c);
        }
    }
}