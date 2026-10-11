/**
 * @file IslandBuilder.cpp
 * @brief Implementation of union-find disjoint-set partitioning for physics islands.
 */

#include "RT-PhysicsCore/physics/solver/IslandBuilder.h"
#include "RT-PhysicsCore/physics/components/RigidBodyComponent.h"
#include <algorithm>
#include <unordered_map>

namespace RT_PhysicsCore
{
    IslandBuilder::IslandBuilder()
    {}

    int IslandBuilder::FindRoot(std::vector<Node>& nodes, int index)
    {
        int root = index;
        while (nodes[root].parent >= 0)
        {
            root = nodes[root].parent;
        }

        // Path compression
        int curr = index;
        while (curr != root)
        {
            int next = nodes[curr].parent;
            if (next >= 0)
            {
                nodes[curr].parent = root;
            }
            curr = next;
        }

        return root;
    }

    void IslandBuilder::UnionNodes(std::vector<Node>& nodes, int a, int b)
    {
        int rootA = FindRoot(nodes, a);
        int rootB = FindRoot(nodes, b);

        if (rootA == rootB)
        {
            return;
        }

        if (nodes[rootA].rank < nodes[rootB].rank)
        {
            nodes[rootA].parent = rootB;
        }
        else if (nodes[rootA].rank > nodes[rootB].rank)
        {
            nodes[rootB].parent = rootA;
        }
        else
        {
            nodes[rootB].parent = rootA;
            nodes[rootA].rank++;
        }
    }

    void IslandBuilder::BuildIslands(Scene& scene, std::vector<Contact>& contacts, std::vector<SimulationIsland>& outIslands)
    {
        outIslands.clear();

        // 1. Collect all dynamic entities that participate in contacts
        std::vector<Entity> dynamicEntities;
        std::unordered_map<Entity, int> entityToIndex;

        auto RegisterDynamic = [&](Entity e)
            {
                auto* rb = scene.GetComponent<RigidBodyComponent>(e);
                if (rb && rb->invMass > 0.0)
                {
                    if (entityToIndex.find(e) == entityToIndex.end())
                    {
                        entityToIndex[e] = static_cast<int>(dynamicEntities.size());
                        dynamicEntities.push_back(e);
                    }
                }
            };

        for (Contact& c : contacts)
        {
            RegisterDynamic(c.a);
            RegisterDynamic(c.b);
        }

        if (dynamicEntities.empty())
        {
            return;
        }

        // 2. Initialize disjoint-set forest
        std::vector<Node> nodes(dynamicEntities.size());

        // 3. Union dynamic entities sharing contacts
        for (Contact& c : contacts)
        {
            auto itA = entityToIndex.find(c.a);
            auto itB = entityToIndex.find(c.b);

            if (itA != entityToIndex.end() && itB != entityToIndex.end())
            {
                UnionNodes(nodes, itA->second, itB->second);
            }
        }

        // 4. Group entities by their root partition
        std::unordered_map<int, int> rootToIslandIndex;
        for (size_t i = 0; i < dynamicEntities.size(); ++i)
        {
            int root = FindRoot(nodes, static_cast<int>(i));
            auto it = rootToIslandIndex.find(root);
            if (it == rootToIslandIndex.end())
            {
                rootToIslandIndex[root] = static_cast<int>(outIslands.size());
                SimulationIsland island;
                island.entities.push_back(dynamicEntities[i]);
                outIslands.push_back(std::move(island));
            }
            else
            {
                outIslands[it->second].entities.push_back(dynamicEntities[i]);
            }
        }

        // 5. Assign contacts to islands
        for (Contact& c : contacts)
        {
            auto itA = entityToIndex.find(c.a);
            auto itB = entityToIndex.find(c.b);

            int representative = -1;
            if (itA != entityToIndex.end())
            {
                representative = itA->second;
            }
            else if (itB != entityToIndex.end())
            {
                representative = itB->second;
            }

            if (representative != -1)
            {
                int root = FindRoot(nodes, representative);
                int islandIdx = rootToIslandIndex[root];
                outIslands[islandIdx].contacts.push_back(&c);
            }
        }

        // 6. Sort entities within each island for deterministic execution
        for (auto& island : outIslands)
        {
            std::sort(island.entities.begin(), island.entities.end());
        }
    }
}