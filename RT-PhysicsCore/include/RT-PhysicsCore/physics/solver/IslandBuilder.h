/**
 * @file IslandBuilder.h
 * @brief Disjoint-set graph partitioning for dynamic bodies and contacts.
 */

#pragma once

#include "RT-PhysicsCore/core/ecs/core/Scene.h"
#include "RT-PhysicsCore/physics/collision/Contact.h"
#include <vector>

namespace RT_PhysicsCore
{
    /**
     * @struct SimulationIsland
     * @brief Independent simulation island containing coupled dynamic bodies and contact constraints.
     */
    struct SimulationIsland
    {
        std::vector<Entity> entities;       ///< Sorted collection of dynamic entities participating in the island.
        std::vector<Contact*> contacts;     ///< Contact manifolds interconnecting entities in this island.
    };

    /**
     * @class IslandBuilder
     * @brief Partitions the global physics graph into independent sub-graphs via disjoint-set union-find.
     */
    class IslandBuilder
    {
    public:
        /**
         * @brief Default constructor.
         */
        IslandBuilder();

        /**
         * @brief Partitions bodies and contacts into disjoint simulation islands.
         * @param scene Scene providing access to entity components.
         * @param contacts Master list of active contact manifolds.
         * @param outIslands Destination vector populated with sorted simulation islands.
         */
        void BuildIslands(Scene& scene, std::vector<Contact>& contacts, std::vector<SimulationIsland>& outIslands);

    private:
        struct Node
        {
            int parent{ -1 };
            int rank{ 0 };
        };

        int FindRoot(std::vector<Node>& nodes, int index);
        void UnionNodes(std::vector<Node>& nodes, int a, int b);
    };
}