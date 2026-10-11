/**
 * @file PhysicsSystem.h
 * @brief Rigid body numerical integration system with split velocity and position passes.
 */

#pragma once

#include "RT-PhysicsCore/core/ecs/core/System.h"
#include "RT-PhysicsCore/core/math/MathTypes.h"

namespace RT_PhysicsCore
{
    /**
     * @class PhysicsSystem
     * @brief Evaluates Newtonian rigid body dynamics using split symplectic Euler and DLM rotation splitting.
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
         * @brief Integrates external forces and torques into linear and angular momentum.
         * @param h Substep duration in seconds.
         */
        void IntegrateVelocities(double h);

        /**
         * @brief Integrates positions, DLM rotations, and pseudo-velocity displacements.
         * @param h Substep duration in seconds.
         */
        void IntegratePositions(double h);

        /**
         * @brief Sets the global gravitational acceleration vector.
         * @param gravity Gravitational acceleration in m/s^2.
         */
        void SetGravity(const Vec3& gravity);

        /**
         * @brief Retrieves the current global gravitational acceleration vector.
         * @return Const reference to gravity vector in m/s^2.
         */
        const Vec3& GetGravity() const;

    private:
        Vec3 gravity{ 0.0, -9.80665, 0.0 };
    };
}