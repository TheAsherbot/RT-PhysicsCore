/**
 * @file ComponentStorage.h
 * @brief Contiguous sparse-set component storage template for the ECS.
 */

#pragma once

#include <vector>
#include <limits>
#include "RT-PhysicsCore/core/ecs/core/Entity.h"
#include "RT-PhysicsCore/core/ecs/core/IComponentStorage.h"

namespace RT_PhysicsCore
{
    /**
     * @class ComponentStorage
     * @brief Flat sparse-set contiguous storage container for component type T.
     *
     * Maps entity IDs to packed components via a flat sparse index vector for
     * O(1) direct lookup, insertion, and swap-and-pop removal without hash overhead.
     * @tparam T The component data type stored in this pool.
     */
    template<typename T>
    class ComponentStorage : public IComponentStorage
    {
    public:
        /**
         * @brief Checks whether the given entity owns an instance of component T.
         * @param entity The entity handle to inspect.
         * @return True if the entity has this component, false otherwise.
         */
        bool Has(Entity entity) const override;

        /**
         * @brief Retrieves a pointer to the entity's component for modification.
         * @param entity The target entity handle.
         * @return Pointer to the component if found; nullptr otherwise.
         */
        T* Get(Entity entity);

        /**
         * @brief Retrieves a const pointer to the entity's component for read-only access.
         * @param entity The target entity handle.
         * @return Const pointer to the component if found; nullptr otherwise.
         */
        const T* Get(Entity entity) const;

        /**
         * @brief Adds or replaces an instance of component T for the specified entity.
         * @param entity The entity handle to associate with the component.
         * @param component The component data to store.
         */
        void Add(Entity entity, const T& component);

        /**
         * @brief Removes the component for the given entity using swap-and-pop.
         * @param entity The entity handle whose component should be removed.
         */
        void Remove(Entity entity) override;

        /**
         * @brief Provides direct read-only access to the contiguous array of components.
         * @return Const reference to the packed component vector.
         */
        const std::vector<T>& GetComponents() const;

        /**
         * @brief Provides direct read-only access to the dense array of corresponding entity IDs.
         * @return Const reference to the entity vector matched 1:1 with GetComponents().
         */
        const std::vector<Entity>& GetEntities() const;

    private:
        static constexpr std::size_t invalidIndex = (std::numeric_limits<std::size_t>::max)();

        std::vector<T> components;
        std::vector<Entity> indexToEntity;
        std::vector<std::size_t> entityToIndex;
    };

    // --- Template definitions ---
    // Must live in the header: a template definition must be visible in every
    // translation unit that instantiates it for a given T.

    template<typename T>
    bool ComponentStorage<T>::Has(Entity entity) const
    {
        return entity < entityToIndex.size() && entityToIndex[entity] != invalidIndex;
    }

    template<typename T>
    T* ComponentStorage<T>::Get(Entity entity)
    {
        if (entity >= entityToIndex.size() || entityToIndex[entity] == invalidIndex)
        {
            return nullptr;
        }
        return &components[entityToIndex[entity]];
    }

    template<typename T>
    const T* ComponentStorage<T>::Get(Entity entity) const
    {
        if (entity >= entityToIndex.size() || entityToIndex[entity] == invalidIndex)
        {
            return nullptr;
        }
        return &components[entityToIndex[entity]];
    }

    template<typename T>
    void ComponentStorage<T>::Add(Entity entity, const T& component)
    {
        if (entity < entityToIndex.size() && entityToIndex[entity] != invalidIndex)
        {
            components[entityToIndex[entity]] = component;
            return;
        }

        if (entity >= entityToIndex.size())
        {
            std::size_t newSize = entity + 1;
            if (newSize < entityToIndex.size() * 2)
            {
                newSize = entityToIndex.size() * 2;
            }
            entityToIndex.resize(newSize, invalidIndex);
        }

        std::size_t index = components.size();
        components.push_back(component);
        entityToIndex[entity] = index;
        indexToEntity.push_back(entity);
    }

    template<typename T>
    void ComponentStorage<T>::Remove(Entity entity)
    {
        if (entity >= entityToIndex.size() || entityToIndex[entity] == invalidIndex)
        {
            return;
        }

        std::size_t index = entityToIndex[entity];
        std::size_t lastIndex = components.size() - 1;

        if (index != lastIndex)
        {
            components[index] = components[lastIndex];
            Entity movedEntity = indexToEntity[lastIndex];
            indexToEntity[index] = movedEntity;
            entityToIndex[movedEntity] = index;
        }

        components.pop_back();
        indexToEntity.pop_back();
        entityToIndex[entity] = invalidIndex;
    }

    template<typename T>
    const std::vector<T>& ComponentStorage<T>::GetComponents() const
    {
        return components;
    }

    template<typename T>
    const std::vector<Entity>& ComponentStorage<T>::GetEntities() const
    {
        return indexToEntity;
    }
}