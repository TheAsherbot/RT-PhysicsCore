#pragma once

#include "RT-PhysicsCore/core/ecs/core/System.h"
#include "RT-PhysicsCore/physics/collision/Contact.h"
#include <vector>

namespace RT_PhysicsCore
{
    class CollisionSystem;

    // Contact resolution. Two independent choices, each switchable at
    // startup (constructor) or at runtime (setters - e.g. from a hotkey):
    //
    //  - SolverMode: SequentialImpulses (iterative, bounded, predictable
    //    cost every step - see the design notes for why this is the
    //    default) or Exact (solves every contact simultaneously via
    //    Lemke's algorithm - genuinely more accurate per step, but no
    //    bounded worst-case cost, so it falls back to sequential impulses
    //    for that step - logged - if it doesn't converge in time).
    //  - IterationMode (sequential-impulses only): Fixed (always the
    //    configured iteration count) or Adaptive (keep iterating within a
    //    time budget, floored/ceilinged by min/max).
    //
    // Position correction (leftover-penetration cleanup) always runs the
    // same way regardless of which velocity solver was used - it's an
    // orthogonal concern. No restitution or friction yet.
    class ResolutionSystem : public ISystem
    {
    public:
        enum class SolverMode { SequentialImpulses, Exact };
        enum class IterationMode { Fixed, Adaptive };

        ResolutionSystem(Scene& scene, CollisionSystem& collisionSystem,
            SolverMode initialMode = SolverMode::SequentialImpulses);

        void FixedUpdate(double dt) override;

        void SetSolverMode(SolverMode mode);
        SolverMode GetSolverMode() const;

        void SetIterationMode(IterationMode mode);
        IterationMode GetIterationMode() const;

        void SetVelocityIterations(int iterations);
        void SetPositionIterations(int iterations);

        // Only consulted in Adaptive mode.
        void SetVelocityIterationBounds(int minIterations, int maxIterations);
        void SetPositionIterationBounds(int minIterations, int maxIterations);
        void SetVelocityTimeBudgetMs(float milliseconds);
        void SetPositionTimeBudgetMs(float milliseconds);

        // Only consulted in Exact mode. 0 = solver picks a size-based default.
        void SetMaxLcpPivots(int pivots);

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
    };
}
