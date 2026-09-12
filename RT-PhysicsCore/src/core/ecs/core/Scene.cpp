#include "RT-PhysicsCore/core/ecs/core/Scene.h"
#include "RT-PhysicsCore/core/ecs/core/System.h" // Scene::~Scene() needs ISystem to be a complete type
#include <algorithm>

#include "RT-PhysicsCore/core/ecs/components/HierarchyComponent.h"

namespace RT_PhysicsCore
{
    // Defined here rather than defaulted in the header so that
    // std::vector<std::unique_ptr<ISystem>>'s destruction doesn't depend on
    // include order in whichever .cpp happens to instantiate it - System.h
    // is guaranteed visible right above.
    Scene::~Scene() = default;

    Entity Scene::CreateEntity()
    {
        Entity id;
        if (!freeEntities.empty())
        {
            id = freeEntities.back();
            freeEntities.pop_back();
        }
        else
        {
            id = nextEntity++;
        }

        entities.push_back(id);
        return id;
    }

    void Scene::DestroyEntity(Entity entity)
    {
        entities.erase(
            std::remove(entities.begin(), entities.end(), entity),
            entities.end()
        );

        // Hierarchy fix-up must run *before* components are stripped: it
        // needs to read this entity's own HierarchyComponent, which
        // RemoveAllComponents() would otherwise erase first.
        UpdateHierarchyOnDestroy(entity);
        RemoveAllComponents(entity);

        freeEntities.push_back(entity);
    }

    const std::vector<Entity>& Scene::GetEntities() const
    {
        return entities;
    }

    void Scene::AddSystem(std::unique_ptr<ISystem> system)
    {
        systems.push_back(std::move(system));
    }

    void Scene::SetDeltaTime(double dt)
    {
        deltaTime = dt;
    }

    void Scene::SetFixedDeltaTime(double dt)
    {
        fixedDeltaTime = dt;
    }

    double Scene::GetDeltaTime() const
    {
        return deltaTime;
    }

    double Scene::GetFixedDeltaTime() const
    {
        return fixedDeltaTime;
    }

    void Scene::UpdateSystems()
    {
        for (auto& system : systems)
            system->Update();
    }

    void Scene::FixedUpdateSystems()
    {
        for (auto& system : systems)
            system->FixedUpdate(fixedDeltaTime);
    }

    void Scene::RenderUpdateSystems()
    {
        for (auto& system : systems)
            system->RenderUpdate();
    }

    void Scene::SetParent(Entity child, Entity parent)
    {
        auto* hierarchy = GetComponent<HierarchyComponent>(child);
        if (!hierarchy)
        {
            HierarchyComponent hc;
            hc.parent = parent;
            AddComponent(child, hc);
        }
        else
        {
            hierarchy->parent = parent;
        }

        if (parent != invalidEntity)
        {
            auto* parentHierarchy = GetComponent<HierarchyComponent>(parent);
            if (!parentHierarchy)
            {
                HierarchyComponent ph;
                ph.parent = invalidEntity;
                ph.children.push_back(child);
                AddComponent(parent, ph);
            }
            else
            {
                parentHierarchy->children.push_back(child);
            }
        }
    }

    void Scene::RemoveAllComponents(Entity entity)
    {
        // Every storage implements IComponentStorage::Remove(), so this
        // doesn't need to know T for each component type.
        for (auto& [type, storage] : storagesRaw)
        {
            storage->Remove(entity);
        }
    }

    void Scene::UpdateHierarchyOnDestroy(Entity entity)
    {
        auto* hierarchy = GetComponent<HierarchyComponent>(entity);
        if (!hierarchy)
            return;

        if (hierarchy->parent != invalidEntity)
        {
            auto* parentHierarchy = GetComponent<HierarchyComponent>(hierarchy->parent);
            if (parentHierarchy)
            {
                parentHierarchy->children.erase(
                    std::remove(parentHierarchy->children.begin(),
                                parentHierarchy->children.end(),
                                entity),
                    parentHierarchy->children.end()
                );
            }
        }

        for (Entity child : hierarchy->children)
        {
            auto* childHierarchy = GetComponent<HierarchyComponent>(child);
            if (childHierarchy)
                childHierarchy->parent = invalidEntity;
        }
    }
}
