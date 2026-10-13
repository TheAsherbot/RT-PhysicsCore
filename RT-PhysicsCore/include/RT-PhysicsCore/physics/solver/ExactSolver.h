/**
 * @file ExactSolver.h
 * @brief Exact Lemke LCP constraint solver with outer-loop Coulomb friction coupling.
 */

#pragma once

#include "RT-PhysicsCore/physics/solver/ISolverBackend.h"
#include "RT-PhysicsCore/physics/solver/IterativeSolver.h"

namespace RT_PhysicsCore
{
    /**
     * @class ExactSolver
     * @brief Direct LCP solver executing Delassus matrix Lemke pivoting with outer-loop friction convergence.
     */
    class ExactSolver : public ISolverBackend
    {
    public:
        /**
         * @brief Default constructor.
         */
        ExactSolver();

        /**
         * @brief Virtual destructor.
         */
        ~ExactSolver() override;

        /**
         * @brief Prepares constraint rows and effective mass matrices for an island.
         * @param island Simulation island containing bodies and contacts.
         * @param constraints Output vector populated with precomputed constraints.
         * @param h Substep duration in seconds.
         */
        void PrepareConstraints(SimulationIsland& island, std::vector<ContactConstraint>& constraints, double h) override;

        /**
         * @brief Solves non-penetration and friction constraints at the velocity level.
         * @param constraints Precomputed island constraints.
         * @param h Substep duration in seconds.
         */
        void SolveVelocity(std::vector<ContactConstraint>& constraints, double h) override;

        /**
         * @brief Solves restitution rebound impulses for qualifying impacts.
         * @param constraints Precomputed island constraints.
         * @param h Substep duration in seconds.
         */
        void SolveRestitution(std::vector<ContactConstraint>& constraints, double h) override;

        /**
         * @brief Resolves positional penetration via the split-impulse pseudo-velocity channel.
         * @param constraints Precomputed island constraints.
         * @param h Substep duration in seconds.
         */
        void SolvePenetration(std::vector<ContactConstraint>& constraints, double h) override;

        /**
         * @brief Sets maximum constraint points allowed before automatically delegating to iterative backend.
         * @param maxPoints Maximum point count threshold.
         */
        void SetExactMaxPoints(int maxPoints);

        /**
         * @brief Sets maximum number of outer fixed-point friction passes.
         * @param passes Pass count.
         */
        void SetMaxOuterFrictionPasses(int passes);

        /**
         * @brief Sets outer loop convergence tolerance on impulse changes.
         * @param tol Tolerance.
         */
        void SetFrictionTolerance(Real tol);

    private:
        struct BodyState
        {
            Vec3 velocity{ 0.0, 0.0, 0.0 };
            Vec3 angularMomentum{ 0.0, 0.0, 0.0 };
            Vec3 angularVelocity{ 0.0, 0.0, 0.0 };
        };

        struct PointRef
        {
            ContactConstraint* constraint{ nullptr };
            int pointIndex{ 0 };
        };

        static Vec3 ImpulseEffect(Real invMass, const Mat3& invInertiaWorld,
            const Vec3& rImpulse, const Vec3& rQuery, const Vec3& impulse);

        IterativeSolver fallbackSolver;
        int exactMaxPoints{ 96 };
        int maxOuterFrictionPasses{ 2 };
        Real frictionTolerance{ 1e-4 };
    };
}