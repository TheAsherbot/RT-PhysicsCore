#pragma once

#include <vector>
#include <unordered_map>
#include <typeindex>
#include <tuple>
#include <memory>

#include "RT-PhysicsCore/core/ecs/core/Entity.h"
#include "RT-PhysicsCore/core/ecs/core/IComponentStorage.h"
#include "RT-PhysicsCore/core/ecs/core/ComponentStorage.h"

namespace RT_PhysicsCore
{
    class ISystem;

    class Scene
    {
    public:
        Scene() = default;

        // Declared here, defined in Scene.cpp *after* System.h is included -
        // see the comment above the definition for why.
        ~Scene();

        // Entity management - defined in Scene.cpp
        Entity CreateEntity();
        void DestroyEntity(Entity entity);
        const std::vector<Entity>& GetEntities() const;

        // Component API - templates, defined below in this same header
        template<typename T>
        bool HasComponent(Entity entity) const;

        template<typename T>
        T* GetComponent(Entity entity);

        template<typename T>
        const T* GetComponent(Entity entity) const;

        template<typename T>
        void AddComponent(Entity entity, const T& component);

        template<typename T>
        void RemoveComponent(Entity entity);

        // Query: entities that have all requested components - template, below
        template<typename... Ts>
        std::vector<Entity> Query() const;

        // Systems - defined in Scene.cpp
        void AddSystem(std::unique_ptr<ISystem> system);

        // Timing - defined in Scene.cpp
        void SetDeltaTime(double dt);
        void SetFixedDeltaTime(double dt);
        double GetDeltaTime() const;
        double GetFixedDeltaTime() const;

        // System update entry points - defined in Scene.cpp
        void UpdateSystems();
        void FixedUpdateSystems();
        void RenderUpdateSystems();

        // Hierarchy helpers - defined in Scene.cpp
        void SetParent(Entity child, Entity parent);

    private:
        Entity nextEntity{1};
        std::vector<Entity> entities;
        std::vector<Entity> freeEntities;

        double deltaTime{0.0};
        double fixedDeltaTime{0.0};

        std::unordered_map<std::type_index, std::unique_ptr<IComponentStorage>> storagesRaw;
        std::vector<std::unique_ptr<ISystem>> systems;

        // Private helpers - templates, defined below in this same header
        template<typename T>
        ComponentStorage<T>* GetStorage();

        template<typename T>
        const ComponentStorage<T>* GetStorageConst() const;

        template<typename T>
        ComponentStorage<T>* GetOrCreateStorage();

        // Defined in Scene.cpp (not templated)
        void RemoveAllComponents(Entity entity);
        void UpdateHierarchyOnDestroy(Entity entity);
    };

    // --- Template definitions ---
    // Have to live here rather than in Scene.cpp: a template's definition
    // must be visible in every translation unit that instantiates it for a
    // given T, and Scene is deliberately generic over arbitrary
    // component types added from anywhere in the engine. If these bodies
    // lived in Scene.cpp, only Scene.cpp itself could instantiate them; any
    // other .cpp calling scene.AddComponent<Foo>() would compile but fail
    // to link.

    template<typename T>
    bool Scene::HasComponent(Entity entity) const
    {
        const auto* storage = GetStorageConst<T>();
        if (!storage)
            return false;
        return storage->Has(entity);
    }

    template<typename T>
    T* Scene::GetComponent(Entity entity)
    {
        auto* storage = GetStorage<T>();
        if (!storage)
            return nullptr;
        return storage->Get(entity);
    }

    template<typename T>
    const T* Scene::GetComponent(Entity entity) const
    {
        const auto* storage = GetStorageConst<T>();
        if (!storage)
            return nullptr;
        return storage->Get(entity);
    }

    template<typename T>
    void Scene::AddComponent(Entity entity, const T& component)
    {
        auto* storage = GetOrCreateStorage<T>();
        storage->Add(entity, component);
    }

    template<typename T>
    void Scene::RemoveComponent(Entity entity)
    {
        auto* storage = GetStorage<T>();
        if (!storage)
            return;
        storage->Remove(entity);
    }

    template<typename... Ts>
    std::vector<Entity> Scene::Query() const
    {
        std::vector<Entity> result;

        if constexpr (sizeof...(Ts) == 0)
            return result;

        const auto* firstStorage = GetStorageConst<std::tuple_element_t<0, std::tuple<Ts...>>>();
        if (!firstStorage)
            return result;

        const auto& baseEntities = firstStorage->GetEntities();
        for (Entity e : baseEntities)
        {
            if ((HasComponent<Ts>(e) && ...))
                result.push_back(e);
        }

        return result;
    }

    template<typename T>
    ComponentStorage<T>* Scene::GetStorage()
    {
        std::type_index type = std::type_index(typeid(T));
        auto it = storagesRaw.find(type);
        if (it == storagesRaw.end())
            return nullptr;
        return static_cast<ComponentStorage<T>*>(it->second.get());
    }

    template<typename T>
    const ComponentStorage<T>* Scene::GetStorageConst() const
    {
        std::type_index type = std::type_index(typeid(T));
        auto it = storagesRaw.find(type);
        if (it == storagesRaw.end())
            return nullptr;
        return static_cast<const ComponentStorage<T>*>(it->second.get());
    }

    template<typename T>
    ComponentStorage<T>* Scene::GetOrCreateStorage()
    {
        std::type_index type = std::type_index(typeid(T));
        auto it = storagesRaw.find(type);
        if (it == storagesRaw.end())
        {
            auto storage = std::make_unique<ComponentStorage<T>>();
            auto* ptr = storage.get();
            storagesRaw[type] = std::move(storage);
            return ptr;
        }
        return static_cast<ComponentStorage<T>*>(it->second.get());
    }
}
