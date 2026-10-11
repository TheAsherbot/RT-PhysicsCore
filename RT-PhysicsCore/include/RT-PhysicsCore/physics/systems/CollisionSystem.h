/**
 * @file CollisionSystem.h
 * @brief Broad-phase swept AABB filtering and persistent narrow-phase collision management.
 *
 * Traverses collidable entities, generates world-space AABBs, executes narrow-phase
 * intersection tests, and publishes active contact manifolds for resolution.
 */

#pragma once

#include "RT-PhysicsCore/core/ecs/core/System.h"
#include "RT-PhysicsCore/physics/collision/Contact.h"
#include "RT-PhysicsCore/physics/collision/ContactCache.h"
#include <vector>

namespace RT_PhysicsCore
{
    /**
     * @class CollisionSystem
     * @brief Generates swept broad-phase pairs, executes signed-distance narrow phase, and maintains contact cache.
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
         * @brief Executes broad-phase and narrow-phase collision passes during the fixed physics tick.
         * @param dt Fixed delta time in seconds.
         */
        void FixedUpdate(double dt) override;

        /**
         * @brief Executes detection for a specific substep interval using speculative margins.
         * @param h Substep duration in seconds.
         * @param speculativeMargin Maximum positive gap distance for speculative contact formation.
         */
        void Detect(double h, Real speculativeMargin);

        /**
         * @brief Returns active contact manifolds generated during the latest detection pass.
         * @return Const reference to the active contact vector.
         */
        const std::vector<Contact>& GetContacts() const;

        /**
         * @brief Returns active contact manifolds for modification by solver write-backs.
         * @return Mutable reference to the active contact vector.
         */
        std::vector<Contact>& GetContacts();

        /**
         * @brief Retrieves the persistent contact cache.
         * @return Reference to ContactCache.
         */
        ContactCache& GetContactCache();

    private:
        std::vector<Contact> contacts;
        ContactCache contactCache;
    };
}