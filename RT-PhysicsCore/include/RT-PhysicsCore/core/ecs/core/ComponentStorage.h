#pragma once

#include <vector>
#include <unordered_map>
#include "RT-PhysicsCore/core/ecs/core/Entity.h"
#include "RT-PhysicsCore/core/ecs/core/IComponentStorage.h"

namespace RT_PhysicsCore
{
    template<typename T>
    class ComponentStorage : public IComponentStorage
    {
    public:
        bool Has(Entity entity) const override;

        T* Get(Entity entity);
        const T* Get(Entity entity) const;

        void Add(Entity entity, const T& component);
        void Remove(Entity entity) override;

        const std::vector<T>& GetComponents() const;
        const std::vector<Entity>& GetEntities() const;

    private:
        std::vector<T> components;
        std::vector<Entity> indexToEntity;
        std::unordered_map<Entity, std::size_t> entityToIndex;
    };

    // --- Template definitions ---
    // Have to live here rather than in a .cpp: a template's definition must
    // be visible in every translation unit that instantiates it for a given
    // T, and ComponentStorage is instantiated once per component type, from
    // wherever in the engine that type gets added to a Scene. A .cpp here
    // could only ever satisfy calls made from within that one .cpp.

    template<typename T>
    bool ComponentStorage<T>::Has(Entity entity) const
    {
        return entityToIndex.find(entity) != entityToIndex.end();
    }

    template<typename T>
    T* ComponentStorage<T>::Get(Entity entity)
    {
        auto it = entityToIndex.find(entity);
        if (it == entityToIndex.end())
            return nullptr;
        return &components[it->second];
    }

    template<typename T>
    const T* ComponentStorage<T>::Get(Entity entity) const
    {
        auto it = entityToIndex.find(entity);
        if (it == entityToIndex.end())
            return nullptr;
        return &components[it->second];
    }

    template<typename T>
    void ComponentStorage<T>::Add(Entity entity, const T& component)
    {
        auto it = entityToIndex.find(entity);
        if (it != entityToIndex.end())
        {
            components[it->second] = component;
            return;
        }

        std::size_t index = components.size();
        components.push_back(component);
        entityToIndex[entity] = index;
        indexToEntity.push_back(entity);
    }

    template<typename T>
    void ComponentStorage<T>::Remove(Entity entity)
    {
        auto it = entityToIndex.find(entity);
        if (it == entityToIndex.end())
            return;

        std::size_t index = it->second;
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
        entityToIndex.erase(it);
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
