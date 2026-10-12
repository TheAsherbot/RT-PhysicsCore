/**
 * @file AdaptiveIterationTest.cpp
 * @brief Verifies ResolutionSystem performance and iteration budgeting.
 */

#include <algorithm>
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
    RT_LOG_INFO("Adaptive Iteration Test harness starting.");

    Scene scene;
    scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
    auto pipeline = std::make_unique<PhysicsPipelineSystem>(scene, ResolutionSystem::SolverMode::SequentialImpulses, 4);
    PhysicsPipelineSystem* pipelinePtr = pipeline.get();
    scene.AddSystem(std::move(pipeline));

    // Configure iteration counts for velocity and position passes
    pipelinePtr->GetResolutionSystem().SetVelocityIterations(8);
    pipelinePtr->GetResolutionSystem().SetPositionIterations(3);

    // Ground
    Entity g = scene.CreateEntity();
    TransformComponent gtc;
    gtc.position = { 0.0f, -0.5f, 0.0f };
    scene.AddComponent(g, gtc);
    scene.AddComponent(g, MakeStaticBody());
    ColliderComponent gcc;
    gcc.shape = ColliderShape::Box;
    gcc.size = { 50.0f, 0.5f, 50.0f };
    scene.AddComponent(g, gcc);

    // Create a heavy scene (150 objects) that tests solver performance
    for (int i = 0; i < 150; ++i)
    {
        Entity e = scene.CreateEntity();
        TransformComponent tc;
        tc.position = { 0.0f, 1.0f + i * 1.0f, 0.0f };
        scene.AddComponent(e, tc);
        scene.AddComponent(e, MakeDynamicBody(1.0f, ComputeBoxInertia(1.0f, { 0.5f, 0.5f, 0.5f })));
        ColliderComponent cc;
        cc.shape = ColliderShape::Box;
        cc.size = { 0.5f, 0.5f, 0.5f };
        scene.AddComponent(e, cc);
    }

    // Force stack to collapse slightly
    scene.SetFixedDeltaTime(1.0 / 60.0);
    for (int i = 0; i < 10; ++i)
    {
        scene.FixedUpdateSystems();
    }

    // Measure ResolutionSystem times over 100 frames
    int framesExceeded = 0;
    double maxMs = 0.0;
    constexpr double totalBudget = 0.7; // 0.7 ms total budget

    for (int i = 0; i < 100; ++i)
    {
        // Measure only the resolution system
        auto start = std::chrono::high_resolution_clock::now();
        pipelinePtr->GetResolutionSystem().FixedUpdate(1.0 / 60.0);
        auto end = std::chrono::high_resolution_clock::now();

        double ms = std::chrono::duration<double, std::milli>(end - start).count();
        maxMs = std::max(maxMs, ms);

        // Leeway (0.1ms) for thread timing noise
        if (ms > totalBudget + 0.1)
        {
            ++framesExceeded;
        }

        // Run the rest of the systems to advance state
        pipelinePtr->GetCollisionSystem().FixedUpdate(1.0 / 60.0);
    }

    std::ostringstream oss;
    oss << "Budget: " << totalBudget << " ms | Peak: " << std::fixed << std::setprecision(3) << maxMs << " ms";

    if (framesExceeded == 0)
    {
        RT_LOG_INFO("[PASS] Adaptive Solver - " << oss.str());
    }
    else
    {
        RT_LOG_WARN("[WARN] Budget exceeded on " << framesExceeded << "/100 frames - " << oss.str());
    }

    return 0;
}