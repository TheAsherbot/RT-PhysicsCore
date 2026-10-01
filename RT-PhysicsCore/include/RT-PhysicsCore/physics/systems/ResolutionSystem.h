/**
 * @file ResolutionSystem.h
 * @brief Contact constraint resolution system handling bounce, non-penetration, and friction.
 *
 * Supports iterative Sequential Impulses and exact Linear Complementarity Problem (LCP)
 * Lemke pivoting with true circular Coulomb friction cones and split-impulse position correction.
 */

#pragma once

#include "RT-PhysicsCore/core/ecs/core/System.h"
#include "RT-PhysicsCore/physics/collision/Contact.h"
#include <vector>

namespace RT_PhysicsCore
{
    class CollisionSystem;

    /**
     * @class ResolutionSystem
     * @brief Resolves collision contacts through velocity impulses and positional adjustments.
     */
    class ResolutionSystem : public ISystem
    {
    public:
        /**
         * @enum SolverMode
         * @brief Mathematical scheme used to compute normal velocity contact impulses.
         */
        enum class SolverMode
        {
            SequentialImpulses, ///< Iterative Projected Gauss-Seidel constraint solving.
            Exact               ///< Simultaneous exact normal solve via Lemke LCP algorithm.
        };

        /**
         * @enum IterationMode
         * @brief Loop termination policy for iterative solver passes.
         */
        enum class IterationMode
        {
            Fixed,   ///< Always executes a constant number of iterations.
            Adaptive ///< Dynamically iterates within a millisecond time budget.
        };

        /**
         * @brief Constructs the contact resolution system.
         * @param scene Reference to the parent Scene world.
         * @param collisionSystem CollisionSystem supplying active contact manifolds.
         * @param initialMode Initial solver formulation (default: SequentialImpulses).
         */
        ResolutionSystem(Scene& scene, CollisionSystem& collisionSystem,
            SolverMode initialMode = SolverMode::SequentialImpulses);

        /**
         * @brief Executes contact resolution during the fixed physics tick.
         * @param dt Fixed delta time in seconds.
         */
        void FixedUpdate(double dt) override;

        /**
         * @brief Selects between Sequential Impulses and Exact Lemke LCP solving.
         * @param mode Desired solver mode.
         */
        void SetSolverMode(SolverMode mode);

        /**
         * @brief Gets the current solver formulation mode.
         * @return Active SolverMode.
         */
        SolverMode GetSolverMode() const;

        /**
         * @brief Configures fixed vs. time-budgeted adaptive iteration.
         * @param mode Desired iteration mode.
         */
        void SetIterationMode(IterationMode mode);

        /**
         * @brief Gets the current iteration control policy.
         * @return Active IterationMode.
         */
        IterationMode GetIterationMode() const;

        /**
         * @brief Sets iteration count for the velocity impulse solver (Fixed mode).
         * @param iterations Number of iterations (default: 8).
         */
        void SetVelocityIterations(int iterations);

        /**
         * @brief Sets iteration count for positional penetration correction (Fixed mode).
         * @param iterations Number of iterations (default: 3).
         */
        void SetPositionIterations(int iterations);

        /**
         * @brief Configures iteration bounds for adaptive velocity solving.
         * @param minIterations Minimum iterations executed regardless of budget.
         * @param maxIterations Hard cap on total iterations.
         */
        void SetVelocityIterationBounds(int minIterations, int maxIterations);

        /**
         * @brief Configures iteration bounds for adaptive position correction.
         * @param minIterations Minimum iterations executed regardless of budget.
         * @param maxIterations Hard cap on total iterations.
         */
        void SetPositionIterationBounds(int minIterations, int maxIterations);

        /**
         * @brief Sets maximum execution time allowed for velocity resolution in Adaptive mode.
         * @param milliseconds Time budget in milliseconds.
         */
        void SetVelocityTimeBudgetMs(float milliseconds);

        /**
         * @brief Sets maximum execution time allowed for position correction in Adaptive mode.
         * @param milliseconds Time budget in milliseconds.
         */
        void SetPositionTimeBudgetMs(float milliseconds);

        /**
         * @brief Sets pivot cap for the Lemke LCP solver in Exact mode (0 = size-based default).
         * @param pivots Maximum pivot iterations.
         */
        void SetMaxLcpPivots(int pivots);

        /**
         * @brief Relative closing speed threshold below which restitution is forced to zero.
         *
         * Prevents resting contacts under gravity from endlessly micro-bouncing.
         *
         * @param threshold Closing speed threshold in m/s (default: 0.5 m/s).
         */
        void SetRestitutionVelocityThreshold(float threshold);

        /**
         * @brief Tangential relative speed threshold distinguishing static sticking from sliding friction.
         * @param threshold Tangential speed threshold in m/s (default: 0.01 m/s).
         */
        void SetFrictionVelocityThreshold(float threshold);

    private:
        void ResolveSequentialImpulses(const std::vector<Contact>& contacts);
        bool ResolveExact(const std::vector<Contact>& contacts);
        void CorrectPositions(const std::vector<Contact>& contacts);

        CollisionSystem& collisionSystem;

        SolverMode solverMode;
        IterationMode iterationMode = IterationMode::Fixed;

        int velocityIterations = 8;
        int minVelocityIterations = 4;
        int maxVelocityIterations = 20;
        float velocityTimeBudgetMs = 1.0f;

        int positionIterations = 3;
        int minPositionIterations = 1;
        int maxPositionIterations = 8;
        float positionTimeBudgetMs = 0.3f;

        int maxLcpPivots = 0;

        float restitutionVelocityThreshold = 0.5f;
        float frictionVelocityThreshold = 0.01f;
    };
}