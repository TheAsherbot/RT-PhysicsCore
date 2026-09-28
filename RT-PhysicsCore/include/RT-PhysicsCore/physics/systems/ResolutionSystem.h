#pragma once

#include "RT-PhysicsCore/core/ecs/core/System.h"
#include "RT-PhysicsCore/physics/collision/Contact.h"
#include <vector>

namespace RT_PhysicsCore
{
    class CollisionSystem;

    // Contact resolution: non-penetration, restitution (bounce), and
    // Coulomb friction (true circular cone, not the square-pyramid
    // approximation).
    //
    // Two independent choices, each switchable at startup (constructor)
    // or at runtime (setters - e.g. from a hotkey):
    //  - SolverMode: SequentialImpulses (iterative, bounded, predictable
    //    cost every step) or Exact (normal impulses solved simultaneously
    //    via Lemke's algorithm; friction still runs as an iterative pass
    //    afterward, bounded by those now-fixed normal impulses - jointly
    //    solving normal-and-friction exactly is a genuinely harder,
    //    nonlinear problem this isn't attempting). Falls back to
    //    sequential impulses (logged) if the exact solve doesn't converge.
    //  - IterationMode (sequential-impulses, and Exact's friction pass):
    //    Fixed (always the configured iteration count) or Adaptive (keep
    //    iterating within a time budget, floored/ceilinged by min/max).
    //
    // Position correction (leftover-penetration cleanup) always runs the
    // same way regardless of which velocity solver was used - it's an
    // orthogonal concern.
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

        // Below this closing speed, a contact is treated as resting
        // (restitution forced to 0) rather than a real impact - otherwise
        // a resting body's tiny per-step gravity drift reads as an
        // endless tiny bounce.
        void SetRestitutionVelocityThreshold(float threshold);

        // Below this tangential speed, a contact starts a step "stuck"
        // (uses the material's static friction coefficient for that
        // step) rather than "sliding" (kinetic).
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