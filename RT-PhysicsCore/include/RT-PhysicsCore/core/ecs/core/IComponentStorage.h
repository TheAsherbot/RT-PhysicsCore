#pragma once

#include "RT-PhysicsCore/core/ecs/core/Entity.h"

namespace RT_PhysicsCore
{
    // Type-erased base interface for component storage.
    //
    // Scene needs to hold storages of many different component types in one
    // container, and needs to be able to operate on "whichever storage this
    // is" without knowing T at the call site (e.g. when destroying an entity,
    // Scene must remove that entity's component from every storage, but it
    // doesn't know ahead of time which component types that entity has).
    class IComponentStorage
    {
    public:
        virtual ~IComponentStorage();

        virtual bool Has(Entity entity) const = 0;
        virtual void Remove(Entity entity) = 0;
    };
}
