/**
 * @file SolverScalingTest.cpp
 * @brief Measures ResolutionSystem performance scaling (SI vs LCP).
 *
 * Drops objects into a pile and times solver iterations to establish
 * O(n) vs O(n^3) scaling behavior.
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
#include "RT-PhysicsCore/physics/systems/PhysicsSystem.h"
#include "RT-PhysicsCore/physics/systems/CollisionSystem.h"
#include "RT-PhysicsCore/physics/systems/ResolutionSystem.h"
#include "RT-PhysicsCore/physics/components/RigidBodyComponent.h"
#include "RT-PhysicsCore/physics/components/ColliderComponent.h"
#include "RT-PhysicsCore/physics/MassProperties.h"

namespace
{
    using namespace RT_PhysicsCore;

    void MeasureSolver(int objectCount, ResolutionSystem::SolverMode mode, const std::string& modeName)
    {
        Scene scene;
        scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
        scene.AddSystem(std::make_unique<PhysicsSystem>(scene));

        auto colSys = std::make_unique<CollisionSystem>(scene);
        CollisionSystem* colSysPtr = colSys.get();
        scene.AddSystem(std::move(colSys));

        auto resSys = std::make_unique<ResolutionSystem>(scene, *colSysPtr, mode);
        ResolutionSystem* resSysPtr = resSys.get();
        scene.AddSystem(std::move(resSys));

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

        // Pile
        for (int i = 0; i < objectCount; ++i)
        {
            Entity e = scene.CreateEntity();
            TransformComponent tc;
            tc.position = { 0.0f, 1.0f + i * 1.2f, 0.0f };
            scene.AddComponent(e, tc);
            scene.AddComponent(e, MakeDynamicBody(1.0f, ComputeBoxInertia(1.0f, { 0.5f, 0.5f, 0.5f })));
            ColliderComponent cc;
            cc.shape = ColliderShape::Box;
            cc.size = { 0.5f, 0.5f, 0.5f };
            scene.AddComponent(e, cc);
        }

        // Settle the pile
        scene.SetFixedDeltaTime(1.0 / 60.0);
        for (int i = 0; i < 180; ++i)
        {
            scene.FixedUpdateSystems();
        }

        int contacts = colSysPtr->GetContacts().size();

        // Measure resolution time
        constexpr int samples = 30;
        double totalMs = 0.0;

        for (int i = 0; i < samples; ++i)
        {
            auto start = std::chrono::high_resolution_clock::now();
            resSysPtr->FixedUpdate(1.0 / 60.0);
            auto end = std::chrono::high_resolution_clock::now();

            totalMs += std::chrono::duration<double, std::milli>(end - start).count();
        }

        double avgMs = totalMs / samples;

        std::ostringstream oss;
        oss << std::left << std::setw(4) << objectCount << " objs | "
            << std::setw(4) << contacts << " contacts | "
            << std::setw(15) << modeName << " | "
            << std::fixed << std::setprecision(3) << avgMs << " ms avg";

        RT_LOG_INFO(oss.str());
    }
}

int main()
{
    RT_LOG_INFO("Solver Scaling Test (SI vs LCP)");

    std::vector<int> counts = { 10, 25, 50 }; // Cap at 50 to avoid freezing on LCP

    for (int count : counts)
    {
        MeasureSolver(count, ResolutionSystem::SolverMode::SequentialImpulses, "SI (8 iter)");
        MeasureSolver(count, ResolutionSystem::SolverMode::Exact, "LCP (Lemke)");
    }

    RT_LOG_INFO("Solver scaling checks completed.");
    return 0;
}