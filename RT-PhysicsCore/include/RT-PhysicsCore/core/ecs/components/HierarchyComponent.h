#pragma once

#include <vector>
#include "RT-PhysicsCore/core/ecs/core/Entity.h"

namespace RT_PhysicsCore
{
    struct HierarchyComponent
    {
        Entity parent{invalidEntity};
        std::vector<Entity> children;
    };
}
