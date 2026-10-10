/**
 * @file PhysicsSystem.h
 * @brief Rigid body numerical integration system using symplectic Euler and DLM rotational splitting.
 */

#pragma once

#include "RT-PhysicsCore/core/ecs/core/System.h"
#include <glm/glm.hpp>

namespace RT_PhysicsCore
{
    /**
     * @class PhysicsSystem
     * @brief Evaluates Newtonian rigid body dynamics across active physics entities.
     *
     * Integrates linear motion via semi-implicit Euler and rotational dynamics
     * via the Dullweber-Leimkuhler-McLachlan (DLM) symplectic splitting scheme.
     */
    class PhysicsSystem : public ISystem
    {
    public:
        /**
         * @brief Constructs the physics dynamics integration system.
         * @param scene Reference to the parent ECS Scene containing entities to simulate.
         */
        explicit PhysicsSystem(Scene& scene);

        /**
         * @brief Advances rigid body state across one fixed physics timestep.
         * @param dt Fixed delta time in seconds.
         */
        void FixedUpdate(double dt) override;

        /**
         * @brief Sets the global gravitational acceleration vector.
         *
         * Applied as `forceAccum += mass * gravity` rather than direct velocity modification,
         * ensuring correct mass-independent free-fall acceleration.
         *
         * @param gravity Gravitational acceleration in m/s^2 (default: [0, -9.80665, 0]).
         */
        void SetGravity(const glm::vec3& gravity);

        /**
         * @brief Retrieves the current global gravitational acceleration vector.
         * @return Const reference to gravity vector in m/s^2.
         */
        const glm::vec3& GetGravity() const;

    private:
        glm::vec3 gravity{ 0.0f, -9.80665f, 0.0f };
    };
}