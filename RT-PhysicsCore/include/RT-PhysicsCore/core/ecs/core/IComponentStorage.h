/**
 * @file IComponentStorage.h
 * @brief Type-erased base interface for component storage pools.
 *
 * Allows the Scene to manage heterogeneous component collections uniformly without
 * needing compile-time knowledge of concrete component types.
 */

#pragma once

#include "RT-PhysicsCore/core/ecs/core/Entity.h"

namespace RT_PhysicsCore
{
    /**
     * @class IComponentStorage
     * @brief Type-erased base interface for component pools.
     *
     * Scene holds storages of many different component types in a single container
     * and must be able to perform entity clean-up without knowing T at call site
     * (e.g., when destroying an entity).
     */
    class IComponentStorage
    {
    public:
        /**
         * @brief Virtual destructor ensuring proper cleanup in derived component pools.
         */
        virtual ~IComponentStorage();

        /**
         * @brief Checks whether the specified entity owns a component in this pool.
         * @param entity The entity handle to inspect.
         * @return True if the entity is registered in this storage, false otherwise.
         */
        virtual bool Has(Entity entity) const = 0;

        /**
         * @brief Removes the component associated with the given entity, if present.
         * @param entity The entity handle whose component should be removed.
         */
        virtual void Remove(Entity entity) = 0;
    };
}