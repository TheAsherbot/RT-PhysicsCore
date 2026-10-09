/**
 * @file BroadphaseScalingTest.cpp
 * @brief Measures CollisionSystem broadphase scaling performance.
 *
 * Spawns N non-overlapping objects and measures the time required to perform
 * broadphase sweeps, isolating AABB intersection performance.
 */

#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "RT-PhysicsCore/utils/Log.h"
#include "RT-PhysicsCore/core/ecs/core/Scene.h"
#include "RT-PhysicsCore/core/ecs/systems/TransformPropagationSystem.h"
#include "RT-PhysicsCore/core/ecs/components/TransformComponent.h"
#include "RT-PhysicsCore/physics/systems/CollisionSystem.h"
#include "RT-PhysicsCore/physics/components/ColliderComponent.h"

namespace
{
    using namespace RT_PhysicsCore;

    void RunTier(int objectCount)
    {
        Scene scene;
        scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
        auto colSys = std::make_unique<CollisionSystem>(scene);
        CollisionSystem* colSysPtr = colSys.get();
        scene.AddSystem(std::move(colSys));

        // Create a 3D grid of objects spaced far apart so they don't overlap
        int side = static_cast<int>(std::ceil(std::cbrt(objectCount)));
        int created = 0;

        for (int x = 0; x < side && created < objectCount; ++x)
        {
            for (int y = 0; y < side && created < objectCount; ++y)
            {
                for (int z = 0; z < side && created < objectCount; ++z)
                {
                    Entity e = scene.CreateEntity();
                    TransformComponent tc;
                    tc.position = { x * 10.0f, y * 10.0f, z * 10.0f };
                    scene.AddComponent(e, tc);

                    ColliderComponent cc;
                    cc.shape = ColliderShape::Box;
                    cc.size = { 1.0f, 1.0f, 1.0f };
                    scene.AddComponent(e, cc);

                    ++created;
                }
            }
        }

        // Warm up
        scene.SetFixedDeltaTime(1.0 / 60.0);
        scene.FixedUpdateSystems();

        // Measure
        constexpr int samples = 50;
        double totalMs = 0.0;

        for (int i = 0; i < samples; ++i)
        {
            auto start = std::chrono::high_resolution_clock::now();
            colSysPtr->FixedUpdate(1.0 / 60.0);
            auto end = std::chrono::high_resolution_clock::now();

            totalMs += std::chrono::duration<double, std::milli>(end - start).count();
        }

        double avgMs = totalMs / samples;
        int expectedPairs = (objectCount * (objectCount - 1)) / 2;

        std::ostringstream oss;
        oss << std::setw(6) << objectCount << " objs | "
            << std::setw(8) << expectedPairs << " pairs | "
            << std::fixed << std::setprecision(3) << avgMs << " ms avg";

        RT_LOG_INFO(oss.str());
    }
}

int main()
{
    RT_LOG_INFO("Broadphase Scaling Test harness starting.");
    RT_LOG_INFO("=== Broadphase Scaling ===");

    std::vector<int> tiers = { 50, 100, 250, 500, 1000, 2000, 5000, 10000 };
    for (int count : tiers)
    {
        RunTier(count);
    }

    RT_LOG_INFO("Broadphase checks completed.");
    return 0;
}