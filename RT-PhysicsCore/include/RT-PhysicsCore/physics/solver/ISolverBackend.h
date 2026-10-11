/**
 * @file ISolverBackend.h
 * @brief Abstract interface defining island constraint resolution passes.
 */

#pragma once

#include "RT-PhysicsCore/physics/solver/ContactConstraint.h"
#include "RT-PhysicsCore/physics/solver/IslandBuilder.h"
#include <vector>

namespace RT_PhysicsCore
{
    /**
     * @class ISolverBackend
     * @brief Interface implemented by iterative (PGS) and exact (Lemke LCP) constraint solvers.
     */
    class ISolverBackend
    {
    public:
        /**
         * @brief Virtual destructor.
         */
        virtual ~ISolverBackend();

        /**
         * @brief Prepares constraint rows and effective mass matrices for an island.
         * @param island Simulation island containing bodies and contacts.
         * @param constraints Output vector populated with precomputed constraints.
         * @param h Substep duration in seconds.
         */
        virtual void PrepareConstraints(SimulationIsland& island, std::vector<ContactConstraint>& constraints, double h) = 0;

        /**
         * @brief Solves non-penetration and friction constraints at the velocity level.
         * @param constraints Precomputed island constraints.
         * @param h Substep duration in seconds.
         */
        virtual void SolveVelocity(std::vector<ContactConstraint>& constraints, double h) = 0;

        /**
         * @brief Solves restitution rebound impulses for qualifying impacts.
         * @param constraints Precomputed island constraints.
         * @param h Substep duration in seconds.
         */
        virtual void SolveRestitution(std::vector<ContactConstraint>& constraints, double h) = 0;

        /**
         * @brief Resolves positional penetration via the split-impulse pseudo-velocity channel.
         * @param constraints Precomputed island constraints.
         * @param h Substep duration in seconds.
         */
        virtual void SolvePenetration(std::vector<ContactConstraint>& constraints, double h) = 0;
    };
}