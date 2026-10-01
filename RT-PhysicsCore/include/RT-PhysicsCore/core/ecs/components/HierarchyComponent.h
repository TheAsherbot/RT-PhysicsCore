/**
 * @file HierarchyComponent.h
 * @brief Scene graph relationships tracking parent and child entities.
 */

#pragma once

#include <vector>
#include "RT-PhysicsCore/core/ecs/core/Entity.h"

namespace RT_PhysicsCore
{
    /**
     * @struct HierarchyComponent
     * @brief Establishes tree relationships for scene graph transform propagation.
     */
    struct HierarchyComponent
    {
        Entity parent{ invalidEntity };    ///< Parent entity handle (invalidEntity if root).
        std::vector<Entity> children;    ///< List of child entity handles.
    };
}