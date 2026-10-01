/**
 * @file PhysicsMaterialComponent.h
 * @brief Component associating an entity with a specific surface material profile.
 */

#pragma once

#include "RT-PhysicsCore/physics/PhysicsMaterial.h"

namespace RT_PhysicsCore
{
    /**
     * @struct PhysicsMaterialComponent
     * @brief Associates an entity with surface friction and restitution coefficients.
     *
     * Optional component: if omitted, CollisionSystem falls back to MaterialId::Default.
     */
    struct PhysicsMaterialComponent
    {
        MaterialId material{ MaterialId::Default }; ///< The surface material identifier.
    };
}