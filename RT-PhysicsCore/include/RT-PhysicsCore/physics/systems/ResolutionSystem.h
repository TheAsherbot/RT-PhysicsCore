/**
 * @file ResolutionSystem.h
 * @brief Façade coordinator dispatching island constraints to iterative and exact solver backends.
 */

#pragma once

#include "RT-PhysicsCore/core/ecs/core/System.h"
#include "RT-PhysicsCore/physics/collision/Contact.h"
#include "RT-PhysicsCore/physics/collision/ContactCache.h"
#include "RT-PhysicsCore/physics/solver/IslandBuilder.h"
#include "RT-PhysicsCore/physics/solver/IterativeSolver.h"
#include "RT-PhysicsCore/physics/solver/ExactSolver.h"
#include <vector>

namespace RT_PhysicsCore
{
    class CollisionSystem;

    /**
     * @class ResolutionSystem
     * @brief Coordinates island graph building and delegates constraint resolution to selected backend.
     */
    class ResolutionSystem : public ISystem
    {
    public:
        /**
         * @enum SolverMode
         * @brief Mathematical solver formulation applied to island constraints.
         */
        enum class SolverMode
        {
            SequentialImpulses, ///< Projected Gauss-Seidel constraint solver.
            Exact               ///< Direct Lemke Delassus LCP solver with outer-loop friction.
        };

        /**
         * @brief Constructs the contact resolution system.
         * @param scene Reference to parent Scene.
         * @param collisionSystem CollisionSystem supplying active contact manifolds.
         * @param initialMode Initial solver backend formulation.
         */
        ResolutionSystem(Scene& scene, CollisionSystem& collisionSystem,
            SolverMode initialMode = SolverMode::SequentialImpulses);

        /**
         * @brief Default destructor.
         */
        ~ResolutionSystem() override;

        /**
         * @brief Executes resolution across contacts during standard fixed physics tick.
         * @param dt Fixed delta time in seconds.
         */
        void FixedUpdate(double dt) override;

        /**
         * @brief Solves island constraints for a specific substep interval.
         * @param h Substep duration in seconds.
         * @param contactCache Reference to persistent contact cache for impulse storage.
         */
        void Resolve(double h, ContactCache& contactCache);

        /**
         * @brief Selects between Iterative PGS and Exact Lemke LCP solvers.
         * @param mode Desired solver mode.
         */
        void SetSolverMode(SolverMode mode);

        /**
         * @brief Gets current solver formulation mode.
         * @return Active SolverMode.
         */
        SolverMode GetSolverMode() const;

        /**
         * @brief Sets iteration count for the iterative velocity impulse solver.
         * @param iterations Number of iterations.
         */
        void SetVelocityIterations(int iterations);

        /**
         * @brief Sets iteration count for split-impulse penetration resolution.
         * @param iterations Number of iterations.
         */
        void SetPositionIterations(int iterations);

        /**
         * @brief Sets closing speed threshold below which restitution is forced to zero.
         * @param threshold Velocity in meters per second.
         */
        void SetRestitutionVelocityThreshold(Real threshold);

    private:
        CollisionSystem& collisionSystem;
        SolverMode solverMode;

        IslandBuilder islandBuilder;
        IterativeSolver iterativeSolver;
        ExactSolver exactSolver;
    };
}