/**
 * @file Scene.h
 * @brief Central ECS world context managing entities, component pools, and systems.
 */

#pragma once

#include <vector>
#include <unordered_map>
#include <typeindex>
#include <tuple>
#include <memory>
#include <limits>

#include "RT-PhysicsCore/core/ecs/core/Entity.h"
#include "RT-PhysicsCore/core/ecs/core/IComponentStorage.h"
#include "RT-PhysicsCore/core/ecs/core/ComponentStorage.h"

namespace RT_PhysicsCore
{
    class ISystem;

    /**
     * @class Scene
     * @brief The ECS world container owning all entities, components, and systems.
     */
    class Scene
    {
    public:
        /**
         * @brief Constructs an empty scene with no entities or systems.
         */
        Scene();

        /**
         * @brief Destructor. Defined out-of-line in Scene.cpp to allow forward declaration of ISystem.
         */
        ~Scene();

        /**
         * @brief Creates a new unique Entity in the scene, recycling dead IDs when available.
         * @return The newly assigned Entity handle.
         */
        Entity CreateEntity();

        /**
         * @brief Destroys an entity, removes all its components, and updates hierarchy links.
         * @param entity The entity handle to destroy.
         */
        void DestroyEntity(Entity entity);

        /**
         * @brief Retrieves the list of all currently active entities.
         * @return Const reference to the active entity vector.
         */
        const std::vector<Entity>& GetEntities() const;

        /**
         * @brief Checks if an entity possesses a component of type T.
         * @tparam T The component type to check.
         * @param entity The entity handle to inspect.
         * @return True if the component exists on the entity, false otherwise.
         */
        template<typename T>
        bool HasComponent(Entity entity) const;

        /**
         * @brief Retrieves a mutable pointer to an entity's component of type T.
         * @tparam T The component type to retrieve.
         * @param entity The entity handle.
         * @return Pointer to the component if found; nullptr otherwise.
         */
        template<typename T>
        T* GetComponent(Entity entity);

        /**
         * @brief Retrieves a read-only const pointer to an entity's component of type T.
         * @tparam T The component type to retrieve.
         * @param entity The entity handle.
         * @return Const pointer to the component if found; nullptr otherwise.
         */
        template<typename T>
        const T* GetComponent(Entity entity) const;

        /**
         * @brief Adds or replaces a component of type T on an entity.
         * @tparam T The component type.
         * @param entity The entity handle.
         * @param component The component instance to assign.
         */
        template<typename T>
        void AddComponent(Entity entity, const T& component);

        /**
         * @brief Removes a component of type T from an entity if present.
         * @tparam T The component type to remove.
         * @param entity The entity handle.
         */
        template<typename T>
        void RemoveComponent(Entity entity);

        /**
         * @brief Queries all entities possessing all requested component types.
         * @note Iteration order is driven by the FIRST type specified in Ts...
         *       For optimal performance, specify the rarest component type first.
         * @tparam Ts List of component types that matching entities must have.
         * @return Vector of entities matching all component requirements.
         */
        template<typename... Ts>
        std::vector<Entity> Query() const;

        /**
         * @brief Adds and transfers ownership of an ISystem to the scene.
         * @param system Unique pointer to the system instance.
         */
        void AddSystem(std::unique_ptr<ISystem> system);

        /**
         * @brief Sets the current frame delta time in seconds.
         * @param dt Frame delta time in seconds.
         */
        void SetDeltaTime(double dt);

        /**
         * @brief Sets the fixed physics delta time in seconds.
         * @param dt Fixed delta time in seconds.
         */
        void SetFixedDeltaTime(double dt);

        /**
         * @brief Gets the current frame delta time in seconds.
         * @return Frame delta time in seconds.
         */
        double GetDeltaTime() const;

        /**
         * @brief Gets the fixed physics delta time in seconds.
         * @return Fixed physics delta time in seconds.
         */
        double GetFixedDeltaTime() const;

        /**
         * @brief Executes the Update() hook across all registered systems.
         */
        void UpdateSystems();

        /**
         * @brief Executes the FixedUpdate() hook across all registered systems.
         */
        void FixedUpdateSystems();

        /**
         * @brief Executes the RenderUpdate() hook across all registered systems.
         */
        void RenderUpdateSystems();

        /**
         * @brief Sets or updates the parent-child relationship between two entities.
         * @param child The child entity handle.
         * @param parent The parent entity handle (or invalidEntity to detach).
         */
        void SetParent(Entity child, Entity parent);

    private:
        static constexpr std::size_t invalidIndex = (std::numeric_limits<std::size_t>::max)();

        Entity nextEntity{ 1 };
        std::vector<Entity> entities;
        std::vector<Entity> freeEntities;
        std::vector<std::size_t> entityToIndex;

        double deltaTime{ 0.0 };
        double fixedDeltaTime{ 0.0 };

        std::unordered_map<std::type_index, std::unique_ptr<IComponentStorage>> storagesRaw;
        std::vector<std::unique_ptr<ISystem>> systems;

        template<typename T>
        ComponentStorage<T>* GetStorage();

        template<typename T>
        const ComponentStorage<T>* GetStorageConst() const;

        template<typename T>
        ComponentStorage<T>* GetOrCreateStorage();

        void RemoveAllComponents(Entity entity);
        void UpdateHierarchyOnDestroy(Entity entity);
    };

    // --- Template definitions ---

    template<typename T>
    bool Scene::HasComponent(Entity entity) const
    {
        const auto* storage = GetStorageConst<T>();
        if (!storage)
        {
            return false;
        }
        return storage->Has(entity);
    }

    template<typename T>
    T* Scene::GetComponent(Entity entity)
    {
        auto* storage = GetStorage<T>();
        if (!storage)
        {
            return nullptr;
        }
        return storage->Get(entity);
    }

    template<typename T>
    const T* Scene::GetComponent(Entity entity) const
    {
        const auto* storage = GetStorageConst<T>();
        if (!storage)
        {
            return nullptr;
        }
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
        {
            return;
        }
        storage->Remove(entity);
    }

    template<typename... Ts>
    std::vector<Entity> Scene::Query() const
    {
        std::vector<Entity> result;

        if constexpr (sizeof...(Ts) == 0)
        {
            return result;
        }

        std::tuple<const ComponentStorage<Ts>*...> storages{ GetStorageConst<Ts>()... };

        bool allValid = std::apply([](auto*... s)
            {
                return ((s != nullptr) && ...);
            }, storages);

        if (!allValid)
        {
            return result;
        }

        const auto* firstStorage = std::get<0>(storages);
        const auto& baseEntities = firstStorage->GetEntities();

        for (Entity e : baseEntities)
        {
            bool match = std::apply([e](auto*... s)
                {
                    return (s->Has(e) && ...);
                }, storages);

            if (match)
            {
                result.push_back(e);
            }
        }

        return result;
    }

    template<typename T>
    ComponentStorage<T>* Scene::GetStorage()
    {
        std::type_index type = std::type_index(typeid(T));
        auto it = storagesRaw.find(type);
        if (it == storagesRaw.end())
        {
            return nullptr;
        }
        return static_cast<ComponentStorage<T>*>(it->second.get());
    }

    template<typename T>
    const ComponentStorage<T>* Scene::GetStorageConst() const
    {
        std::type_index type = std::type_index(typeid(T));
        auto it = storagesRaw.find(type);
        if (it == storagesRaw.end())
        {
            return nullptr;
        }
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