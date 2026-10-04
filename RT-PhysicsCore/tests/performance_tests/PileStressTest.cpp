/**
 * @file PileStressTest.cpp
 * @brief Stress tests the full pipeline to find maximum object capacity at 60Hz.
 */

#include <chrono>
#include <memory>
#include <sstream>
#include <string>

#include "RT-PhysicsCore/utils/Log.h"
#include "RT-PhysicsCore/core/ecs/core/Scene.h"
#include "RT-PhysicsCore/core/ecs/systems/TransformPropagationSystem.h"
#include "RT-PhysicsCore/core/ecs/components/TransformComponent.h"
#include "RT-PhysicsCore/physics/systems/PhysicsSystem.h"
#include "RT-PhysicsCore/physics/systems/CollisionSystem.h"
#include "RT-PhysicsCore/physics/systems/ResolutionSystem.h"
#include "RT-PhysicsCore/physics/components/RigidBodyComponent.h"
#include "RT-PhysicsCore/physics/components/ColliderComponent.h"
#include "RT-PhysicsCore/physics/MassProperties.h"

namespace
{
    using namespace RT_PhysicsCore;
}

int main()
{
    RT_LOG_INFO("Pile Stress Test (Target: 60Hz / 16.67ms)");

    Scene scene;
    scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
    scene.AddSystem(std::make_unique<PhysicsSystem>(scene));

    auto colSys = std::make_unique<CollisionSystem>(scene);
    CollisionSystem* colSysPtr = colSys.get();
    scene.AddSystem(std::move(colSys));

    scene.AddSystem(std::make_unique<ResolutionSystem>(scene, *colSysPtr, ResolutionSystem::SolverMode::SequentialImpulses));

    Entity g = scene.CreateEntity();
    TransformComponent gtc;
    gtc.position = { 0.0f, -0.5f, 0.0f };
    scene.AddComponent(g, gtc);
    scene.AddComponent(g, MakeStaticBody());
    ColliderComponent gcc;
    gcc.shape = ColliderShape::Box;
    gcc.size = { 100.0f, 0.5f, 100.0f };
    scene.AddComponent(g, gcc);

    constexpr double targetMs = 16.67;
    double currentAvgMs = 0.0;
    int objCount = 0;

    scene.SetFixedDeltaTime(1.0 / 60.0);

    // Iteratively add objects until we break the budget
    while (currentAvgMs < targetMs && objCount < 1000)
    {
        // Add 10 objects
        for (int i = 0; i < 10; ++i)
        {
            Entity e = scene.CreateEntity();
            TransformComponent tc;
            // Spawn spread out so they drop onto the pile
            tc.position = { (i % 3) * 0.5f, 2.0f + objCount * 0.1f, (i / 3) * 0.5f };
            scene.AddComponent(e, tc);
            scene.AddComponent(e, MakeDynamicBody(1.0f, ComputeBoxInertia(1.0f, { 0.4f, 0.4f, 0.4f })));
            ColliderComponent cc;
            cc.shape = ColliderShape::Box;
            cc.size = { 0.4f, 0.4f, 0.4f };
            scene.AddComponent(e, cc);
            ++objCount;
        }

        // Settle a bit
        for (int i = 0; i < 5; ++i)
        {
            scene.FixedUpdateSystems();
        }

        // Measure
        constexpr int samples = 10;
        double totalMs = 0.0;

        for (int i = 0; i < samples; ++i)
        {
            auto start = std::chrono::high_resolution_clock::now();
            scene.FixedUpdateSystems();
            auto end = std::chrono::high_resolution_clock::now();

            totalMs += std::chrono::duration<double, std::milli>(end - start).count();
        }

        currentAvgMs = totalMs / samples;
    }

    std::ostringstream oss;
    oss << "Max stable objects at 60Hz: " << objCount << " (Frame took " << currentAvgMs << " ms)";
    RT_LOG_INFO(oss.str());

    return 0;
}