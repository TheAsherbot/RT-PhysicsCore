/**
 * @file Entity.h
 * @brief Entity identifier definition and reserved sentinel constants.
 *
 * Provides the lightweight handle used to address entity instances across the ECS.
 */

#pragma once

#include <cstdint>

namespace RT_PhysicsCore
{
    /**
     * @typedef Entity
     * @brief Unique identifier representing an entity in a Scene.
     */
    using Entity = std::uint32_t;

    /**
     * @brief Sentinel value indicating an invalid, null, or unassigned entity handle.
     */
    constexpr Entity invalidEntity = 0;
}