/**
 * @file CollisionSystem.h
 * @brief System responsible for broad-phase and narrow-phase collision detection.
 *
 * Traverses collidable entities, generates world-space AABBs, executes narrow-phase
 * intersection tests, and publishes active contact manifolds for resolution.
 */

#pragma once

#include "RT-PhysicsCore/core/ecs/core/System.h"
#include "RT-PhysicsCore/physics/collision/Contact.h"
#include <vector>

namespace RT_PhysicsCore
{
    /**
     * @class CollisionSystem
     * @brief Performs collision detection across all entities with Transform and Collider components.
     */
    class CollisionSystem : public ISystem
    {
    public:
        /**
         * @brief Constructs the CollisionSystem bound to the parent Scene.
         * @param scene Reference to the Scene containing entities to test.
         */
        explicit CollisionSystem(Scene& scene);

        /**
         * @brief Executes broad-phase and narrow-phase passes during the fixed physics tick.
         * @param dt Fixed delta time in seconds.
         */
        void FixedUpdate(double dt) override;

        /**
         * @brief Returns the list of active contact manifolds detected in the latest fixed tick.
         * @return Const reference to the active contact vector.
         */
        const std::vector<Contact>& GetContacts() const;

    private:
        std::vector<Contact> contacts;
    };
}