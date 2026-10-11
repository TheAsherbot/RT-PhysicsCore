/**
 * @file ContinuousCollisionSystem.h
 * @brief Continuous collision detection system managing shape sweeps and motion clamping.
 */

#pragma once

#include "RT-PhysicsCore/core/ecs/core/System.h"
#include "RT-PhysicsCore/core/math/MathTypes.h"
#include <vector>

namespace RT_PhysicsCore
{
    /**
     * @class ContinuousCollisionSystem
     * @brief Manages swept conservative advancement queries for high-velocity bodies to prevent tunneling.
     */
    class ContinuousCollisionSystem : public ISystem
    {
    public:
        /**
         * @brief Constructs the continuous collision system.
         * @param scene Reference to the parent Scene world.
         */
        ContinuousCollisionSystem(Scene& scene);

        /**
         * @brief Virtual destructor.
         */
        ~ContinuousCollisionSystem() override;

        /**
         * @brief Executes swept CCD queries across active bodies for the current substep.
         * @param h Substep delta time in seconds.
         * @param speculativeMargin Speculative margin distance threshold triggering CCD sweeps.
         */
        void ExecuteCCD(double h, Real speculativeMargin);

        /**
         * @brief Configures maximum conservative advancement iterations per shape cast.
         * @param iterations Iteration count (default: 24).
         */
        void SetMaxCastIterations(int iterations);

        /**
         * @brief Configures target skin security gap for shape casts.
         * @param gap Skin gap in meters (default: 0.001 m).
         */
        void SetTargetSkinGap(Real gap);

    private:
        int maxCastIterations{ 24 };
        Real targetSkinGap{ 0.001 };
    };
}