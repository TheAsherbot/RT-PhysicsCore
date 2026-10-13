/**
 * @file PileStressTest.cpp
 * @brief Stress tests the full physics pipeline on a settling pile to find maximum object capacity at 60Hz.
 */

#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "RT-PhysicsCore/utils/Log.h"
#include "RT-PhysicsCore/core/ecs/core/Scene.h"
#include "RT-PhysicsCore/core/ecs/systems/TransformPropagationSystem.h"
#include "RT-PhysicsCore/core/ecs/components/TransformComponent.h"
#include "RT-PhysicsCore/physics/systems/PhysicsPipelineSystem.h"
#include "RT-PhysicsCore/physics/components/RigidBodyComponent.h"
#include "RT-PhysicsCore/physics/components/ColliderComponent.h"
#include "RT-PhysicsCore/physics/MassProperties.h"

namespace
{
    using namespace RT_PhysicsCore;
}

int main()
{
    RT_LOG_INFO("Pile Stress Test (Target: 60Hz / 16.67ms frame budget)");

    Scene scene;
    scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
    auto pipeline = std::make_unique<PhysicsPipelineSystem>(
        scene, ResolutionSystem::SolverMode::SequentialImpulses, 4);
    scene.AddSystem(std::move(pipeline));

    // Ground floor
    Entity g = scene.CreateEntity();
    TransformComponent gtc;
    gtc.position = { 0.0f, -0.5f, 0.0f };
    scene.AddComponent(g, gtc);
    scene.AddComponent(g, MakeStaticBody());
    ColliderComponent gcc;
    gcc.shape = ColliderShape::Box;
    gcc.size = { 100.0f, 0.5f, 100.0f };
    scene.AddComponent(g, gcc);

    constexpr double kTargetMs = 16.67;
    double currentAvgMs = 0.0;
    int objCount = 0;
    constexpr int kBatchSize = 15;
    constexpr int kMaxCap = 1500;

    scene.SetFixedDeltaTime(1.0 / 60.0);

    while (currentAvgMs < kTargetMs && objCount < kMaxCap)
    {
        // Add a batch of boxes distributed across a 3x5 spawn grid
        for (int i = 0; i < kBatchSize; ++i)
        {
            Entity e = scene.CreateEntity();
            TransformComponent tc;
            float gx = (i % 3 - 1) * 1.2f;
            float gz = (i / 3 - 2) * 1.2f;
            float gy = 2.0f + static_cast<float>(objCount) * 0.08f;
            tc.position = { gx, gy, gz };
            scene.AddComponent(e, tc);
            scene.AddComponent(e, MakeDynamicBody(1.0f, ComputeBoxInertia(1.0f, { 0.4f, 0.4f, 0.4f })));
            ColliderComponent cc;
            cc.shape = ColliderShape::Box;
            cc.size = { 0.4f, 0.4f, 0.4f };
            scene.AddComponent(e, cc);
            ++objCount;
        }

        // Allow 30 full simulation ticks (0.5 seconds) to allow newly spawned boxes to reach floor
        for (int i = 0; i < 30; ++i)
        {
            scene.FixedUpdateSystems();
        }

        // Measure 15 active resting steps
        constexpr int samples = 15;
        double totalMs = 0.0;

        for (int i = 0; i < samples; ++i)
        {
            auto start = std::chrono::high_resolution_clock::now();
            scene.FixedUpdateSystems();
            auto end = std::chrono::high_resolution_clock::now();

            totalMs += std::chrono::duration<double, std::milli>(end - start).count();
        }

        currentAvgMs = totalMs / samples;

        std::ostringstream oss;
        oss << "Capacity: " << std::setw(4) << objCount << " boxes | Step time: "
            << std::fixed << std::setprecision(2) << currentAvgMs << " ms";
        RT_LOG_INFO(oss.str());
    }

    std::ostringstream oss;
    oss << "Max stable resting pile capacity at 60Hz: " << objCount
        << " objects (last measured " << currentAvgMs << " ms)";
    RT_LOG_INFO(oss.str());

    return 0;
}