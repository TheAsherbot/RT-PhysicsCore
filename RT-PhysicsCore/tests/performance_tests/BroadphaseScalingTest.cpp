/**
 * @file BroadphaseScalingTest.cpp
 * @brief Measures CollisionSystem broadphase scaling under sparse and dense clustered distributions.
 */

#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "RT-PhysicsCore/utils/Log.h"
#include "RT-PhysicsCore/core/ecs/core/Scene.h"
#include "RT-PhysicsCore/core/ecs/systems/TransformPropagationSystem.h"
#include "RT-PhysicsCore/core/ecs/components/TransformComponent.h"
#include "RT-PhysicsCore/physics/systems/PhysicsPipelineSystem.h"
#include "RT-PhysicsCore/physics/components/ColliderComponent.h"
#include "RT-PhysicsCore/physics/components/RigidBodyComponent.h"
#include "RT-PhysicsCore/physics/MassProperties.h"

namespace
{
    using namespace RT_PhysicsCore;

    enum class Distribution
    {
        SparseGrid,
        DenseClusters
    };

    /**
     * @brief Measures swept AABB broadphase performance across a specific object count and spatial layout.
     * @param objectCount Number of bodies to populate.
     * @param dist Layout distribution type.
     */
    void RunDistributionBenchmark(int objectCount, Distribution dist)
    {
        Scene scene;
        scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
        auto pipeline = std::make_unique<PhysicsPipelineSystem>(
            scene, ResolutionSystem::SolverMode::SequentialImpulses, 4);
        PhysicsPipelineSystem* pipelinePtr = pipeline.get();
        scene.AddSystem(std::move(pipeline));

        std::mt19937 rng(42);
        std::uniform_real_distribution<float> velDist(-2.0f, 2.0f);

        if (dist == Distribution::SparseGrid)
        {
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
                        tc.position = { x * 8.0f, y * 8.0f, z * 8.0f };
                        scene.AddComponent(e, tc);

                        RigidBodyComponent rb = MakeDynamicBody(1.0f, ComputeBoxInertia(1.0f, { 0.5f, 0.5f, 0.5f }));
                        rb.velocity = { velDist(rng), velDist(rng), velDist(rng) };
                        scene.AddComponent(e, rb);

                        ColliderComponent cc;
                        cc.shape = ColliderShape::Box;
                        cc.size = { 0.5f, 0.5f, 0.5f };
                        scene.AddComponent(e, cc);

                        ++created;
                    }
                }
            }
        }
        else
        {
            // Clustered: 8 tight clusters with localized overlapping swept bounds
            constexpr int kClusters = 8;
            glm::vec3 clusterCenters[kClusters] = {
                { -20.0f, -20.0f, -20.0f }, { 20.0f, -20.0f, -20.0f },
                { -20.0f,  20.0f, -20.0f }, { 20.0f,  20.0f, -20.0f },
                { -20.0f, -20.0f,  20.0f }, { 20.0f, -20.0f,  20.0f },
                { -20.0f,  20.0f,  20.0f }, { 20.0f,  20.0f,  20.0f }
            };

            std::uniform_real_distribution<float> clusterOffset(-2.5f, 2.5f);

            for (int i = 0; i < objectCount; ++i)
            {
                const glm::vec3& center = clusterCenters[i % kClusters];
                Entity e = scene.CreateEntity();
                TransformComponent tc;
                tc.position = center + glm::vec3(clusterOffset(rng), clusterOffset(rng), clusterOffset(rng));
                scene.AddComponent(e, tc);

                RigidBodyComponent rb = MakeDynamicBody(1.0f, ComputeBoxInertia(1.0f, { 0.5f, 0.5f, 0.5f }));
                rb.velocity = { velDist(rng), velDist(rng), velDist(rng) };
                scene.AddComponent(e, rb);

                ColliderComponent cc;
                cc.shape = ColliderShape::Box;
                cc.size = { 0.5f, 0.5f, 0.5f };
                scene.AddComponent(e, cc);
            }
        }

        // Warm up and evaluate contacts
        constexpr double fixedDt = 1.0 / 60.0;
        pipelinePtr->GetCollisionSystem().Detect(fixedDt, 0.002);

        size_t initialOverlaps = pipelinePtr->GetCollisionSystem().GetContacts().size();

        // Sample execution
        constexpr int samples = 30;
        double totalMs = 0.0;

        for (int i = 0; i < samples; ++i)
        {
            auto start = std::chrono::high_resolution_clock::now();
            pipelinePtr->GetCollisionSystem().Detect(fixedDt, 0.002);
            auto end = std::chrono::high_resolution_clock::now();

            totalMs += std::chrono::duration<double, std::milli>(end - start).count();
        }

        double avgMs = totalMs / samples;
        int totalPairs = (objectCount * (objectCount - 1)) / 2;

        std::string distName = (dist == Distribution::SparseGrid) ? "Sparse" : "Dense-Clustered";

        std::ostringstream oss;
        oss << std::setw(15) << distName << " | "
            << std::setw(5) << objectCount << " objs | "
            << std::setw(8) << totalPairs << " pairs | "
            << std::setw(4) << initialOverlaps << " overlaps | "
            << std::fixed << std::setprecision(3) << avgMs << " ms avg";

        RT_LOG_INFO(oss.str());
    }
}

int main()
{
    RT_LOG_INFO("Broadphase Scaling Benchmark (Sparse vs Dense Distributions)");

    std::vector<int> tiers = { 50, 100, 250, 500, 1000 };

    RT_LOG_INFO("--- 1. Sparse Grid Distribution (Baseline Early-Out) ---");
    for (int count : tiers)
    {
        RunDistributionBenchmark(count, Distribution::SparseGrid);
    }

    RT_LOG_INFO("--- 2. Dense Clustered Distribution (Active Overlap Workload) ---");
    for (int count : tiers)
    {
        RunDistributionBenchmark(count, Distribution::DenseClusters);
    }

    RT_LOG_INFO("Broadphase benchmark completed successfully.");
    return 0;
}