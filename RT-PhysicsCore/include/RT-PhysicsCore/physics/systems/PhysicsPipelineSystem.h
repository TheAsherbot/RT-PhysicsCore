/**
 * @file PhysicsPipelineSystem.h
 * @brief Master substepping orchestrator executing velocity integration, detection, solving, and CCD.
 */

#pragma once

#include "RT-PhysicsCore/core/ecs/core/System.h"
#include "RT-PhysicsCore/physics/systems/PhysicsSystem.h"
#include "RT-PhysicsCore/physics/systems/CollisionSystem.h"
#include "RT-PhysicsCore/physics/systems/ResolutionSystem.h"
#include "RT-PhysicsCore/physics/systems/ContinuousCollisionSystem.h"

namespace RT_PhysicsCore
{
    /**
     * @class PhysicsPipelineSystem
     * @brief Coordinates the unified substepping execution loop across all core physics systems.
     */
    class PhysicsPipelineSystem : public ISystem
    {
    public:
        /**
         * @brief Constructs the master physics pipeline system.
         * @param scene Reference to the parent Scene.
         * @param solverMode Initial solver formulation for contact resolution.
         * @param substeps Number of internal substeps per fixed tick (default: 4).
         */
        PhysicsPipelineSystem(Scene& scene,
            ResolutionSystem::SolverMode solverMode = ResolutionSystem::SolverMode::SequentialImpulses,
            int substeps = 4);

        /**
         * @brief Default destructor.
         */
        ~PhysicsPipelineSystem() override;

        /**
         * @brief Executes the master substepping loop for the fixed physics update.
         * @param dt Fixed delta time in seconds.
         */
        void FixedUpdate(double dt) override;

        /**
         * @brief Sets the number of internal substepping cycles per fixed tick.
         * @param substeps Substep count (must be >= 1).
         */
        void SetSubsteps(int substeps);

        /**
         * @brief Gets current substep count.
         * @return Substep count.
         */
        int GetSubsteps() const;

        /**
         * @brief Sets speculative margin distance threshold.
         * @param margin Margin in meters.
         */
        void SetSpeculativeMargin(Real margin);

        /**
         * @brief Accessor for underlying PhysicsSystem.
         * @return Reference to PhysicsSystem.
         */
        PhysicsSystem& GetPhysicsSystem();

        /**
         * @brief Accessor for underlying CollisionSystem.
         * @return Reference to CollisionSystem.
         */
        CollisionSystem& GetCollisionSystem();

        /**
         * @brief Accessor for underlying ResolutionSystem.
         * @return Reference to ResolutionSystem.
         */
        ResolutionSystem& GetResolutionSystem();

        /**
         * @brief Accessor for underlying ContinuousCollisionSystem.
         * @return Reference to ContinuousCollisionSystem.
         */
        ContinuousCollisionSystem& GetContinuousCollisionSystem();

    private:
        int substepCount{ 4 };
        Real speculativeMargin{ 0.002 };

        PhysicsSystem physicsSystem;
        CollisionSystem collisionSystem;
        ResolutionSystem resolutionSystem;
        ContinuousCollisionSystem continuousCollisionSystem;
    };
}