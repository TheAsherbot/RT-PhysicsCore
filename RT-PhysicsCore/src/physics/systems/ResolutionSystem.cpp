/**
 * @file ResolutionSystem.cpp
 * @brief Implementation of island constraint resolution coordinator and solver dispatch.
 */

#include "RT-PhysicsCore/physics/systems/ResolutionSystem.h"
#include "RT-PhysicsCore/physics/systems/CollisionSystem.h"

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

            backend.SolveVelocity(constraints, h);
            backend.SolveRestitution(constraints, h);
            backend.SolvePenetration(constraints, h);
        }

        // 3. Store converged impulses into persistent cache for cross-frame warm starting
        for (const Contact& c : contacts)
        {
            contactCache.Store(c);
        }
    }
}