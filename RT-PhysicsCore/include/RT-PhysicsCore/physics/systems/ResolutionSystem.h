#pragma once

#include "RT-PhysicsCore/core/ecs/core/System.h"

namespace RT_PhysicsCore
{
    class CollisionSystem;

    // Sequential-impulse contact solver: a velocity pass (non-penetration,
    // Newton's-third-law impulses, including the rotational coupling
    // through each body's inertia tensor) followed by a separate,
    // velocity-untouched position pass that corrects whatever penetration
    // is left over from this step's integration. No restitution or
    // friction yet - this is the plain non-penetration solve; those are
    // the next piece.
    class ResolutionSystem : public ISystem
    {
    public:
        ResolutionSystem(Scene& scene, CollisionSystem& collisionSystem);

        void FixedUpdate(double dt) override;

        void SetVelocityIterations(int iterations) { velocityIterations = iterations; }
        void SetPositionIterations(int iterations) { positionIterations = iterations; }

    private:
        CollisionSystem& collisionSystem;
        int velocityIterations = 8;
        int positionIterations = 3;
    };
}
