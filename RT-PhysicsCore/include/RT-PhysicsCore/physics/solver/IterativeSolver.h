/**
 * @file IterativeSolver.h
 * @brief Projected Gauss-Seidel constraint solver with split-impulse position recovery.
 */

#pragma once

#include "RT-PhysicsCore/physics/solver/ISolverBackend.h"

namespace RT_PhysicsCore
{
    /**
     * @class IterativeSolver
     * @brief Iterative Projected Gauss-Seidel solver handling unilateral contact and circular Coulomb friction.
     */
    class IterativeSolver : public ISolverBackend
    {
    public:
        /**
         * @brief Default constructor.
         */
        IterativeSolver();

        /**
         * @brief Virtual destructor.
         */
        ~IterativeSolver() override;

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
         * @brief Sets iteration sweep count for velocity resolution.
         * @param iterations Iteration count.
         */
        void SetVelocityIterations(int iterations);

        /**
         * @brief Sets iteration sweep count for split-impulse penetration resolution.
         * @param iterations Iteration count.
         */
        void SetPositionIterations(int iterations);

        /**
         * @brief Sets penetration slop threshold below which no correction push is applied.
         * @param slop Slop in meters.
         */
        void SetSlop(Real slop);

        /**
         * @brief Sets Baumgarte stabilization factor for penetration recovery.
         * @param beta Fraction in [0, 1].
         */
        void SetBeta(Real beta);

        /**
         * @brief Sets maximum pseudo-velocity push speed limit.
         * @param speed Speed in meters per second.
         */
        void SetMaxPushSpeed(Real speed);

        /**
         * @brief Sets closing speed threshold below which restitution is forced to zero.
         * @param threshold Velocity in meters per second.
         */
        void SetRestitutionVelocityThreshold(Real threshold);

    private:
        static Real AngularEffectiveMass(const Vec3& r, const Vec3& axis, const Mat3& invInertiaWorld);
        static Real CrossEffectiveMass(const Vec3& r, const Vec3& a1, const Vec3& a2, const Mat3& invInertiaWorld);
        static void BuildTangentBasis(const Vec3& normal, Vec3& outT1, Vec3& outT2);

        int velocityIterations{ 50 };
        int positionIterations{ 20 };
        Real slopThreshold{ 0.0005 };
        Real positionBeta{ 0.2 };
        Real maxPushVelocity{ 3.0 };
        Real restitutionVelocityThreshold{ 0.5 };
        Real convergenceTolerance{ 1e-8 };
    };
}