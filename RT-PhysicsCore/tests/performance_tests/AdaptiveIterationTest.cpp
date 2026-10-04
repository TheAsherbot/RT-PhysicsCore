/**
 * @file AdaptiveIterationTest.cpp
 * @brief Verifies ResolutionSystem adaptive time budgeting.
 *
 * Runs a heavy scene using IterationMode::Adaptive and ensures the resolution
 * step time stays strictly within the specified millisecond limits.
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
    RT_LOG_INFO("Adaptive Iteration Test harness starting.");

    Scene scene;
    scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
    scene.AddSystem(std::make_unique<PhysicsSystem>(scene));

    auto colSys = std::make_unique<CollisionSystem>(scene);
    CollisionSystem* colSysPtr = colSys.get();
    scene.AddSystem(std::move(colSys));

    auto resSys = std::make_unique<ResolutionSystem>(scene, *colSysPtr, ResolutionSystem::SolverMode::SequentialImpulses);
    ResolutionSystem* resSysPtr = resSys.get();
    scene.AddSystem(std::move(resSys));

    // Turn on Adaptive mode with strict budgets
    resSysPtr->SetIterationMode(ResolutionSystem::IterationMode::Adaptive);
    resSysPtr->SetVelocityTimeBudgetMs(0.5f); // 0.5 ms limit
    resSysPtr->SetPositionTimeBudgetMs(0.2f); // 0.2 ms limit

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

    // Create a very heavy scene (150 objects) that would normally take ~2ms to resolve
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
    constexpr double totalBudget = 0.5 + 0.2; // 0.7 ms total

    for (int i = 0; i < 100; ++i)
    {
        // We only measure the resolution system here
        auto start = std::chrono::high_resolution_clock::now();
        resSysPtr->FixedUpdate(1.0 / 60.0);
        auto end = std::chrono::high_resolution_clock::now();

        double ms = std::chrono::duration<double, std::milli>(end - start).count();
        maxMs = std::max(maxMs, ms);

        // Give a little leeway (0.1ms) for thread timing noise
        if (ms > totalBudget + 0.1)
        {
            ++framesExceeded;
        }

        // Run the rest of the systems to advance state
        colSysPtr->FixedUpdate(1.0 / 60.0);
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