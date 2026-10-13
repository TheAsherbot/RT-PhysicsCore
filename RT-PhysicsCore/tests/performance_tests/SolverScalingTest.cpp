/**
 * @file SolverScalingTest.cpp
 * @brief Measures ResolutionSystem performance scaling (SI vs LCP) on active contact constraints.
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
#include "RT-PhysicsCore/physics/systems/PhysicsPipelineSystem.h"
#include "RT-PhysicsCore/physics/components/RigidBodyComponent.h"
#include "RT-PhysicsCore/physics/components/ColliderComponent.h"
#include "RT-PhysicsCore/physics/MassProperties.h"

namespace
{
    using namespace RT_PhysicsCore;

    /**
     * @brief Measures solver performance over an active 30-frame dynamic settling window.
     * @param objectCount Number of stacked boxes.
     * @param mode Solver backend formulation.
     * @param modeName Descriptive label.
     */
    void MeasureSolverDynamically(int objectCount, ResolutionSystem::SolverMode mode, const std::string& modeName)
    {
        Scene scene;
        scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
        auto pipeline = std::make_unique<PhysicsPipelineSystem>(scene, mode, 4);
        PhysicsPipelineSystem* pipelinePtr = pipeline.get();
        scene.AddSystem(std::move(pipeline));

        // Ground slab
        Entity g = scene.CreateEntity();
        TransformComponent gtc;
        gtc.position = { 0.0f, -0.5f, 0.0f };
        scene.AddComponent(g, gtc);
        scene.AddComponent(g, MakeStaticBody());
        ColliderComponent gcc;
        gcc.shape = ColliderShape::Box;
        gcc.size = { 50.0f, 0.5f, 50.0f };
        scene.AddComponent(g, gcc);

        // Axially aligned stack of boxes
        for (int i = 0; i < objectCount; ++i)
        {
            Entity e = scene.CreateEntity();
            TransformComponent tc;
            tc.position = { 0.0f, 0.5f + static_cast<float>(i) * 1.0f, 0.0f };
            scene.AddComponent(e, tc);
            scene.AddComponent(e, MakeDynamicBody(1.0f, ComputeBoxInertia(1.0f, { 0.5f, 0.5f, 0.5f })));
            ColliderComponent cc;
            cc.shape = ColliderShape::Box;
            cc.size = { 0.5f, 0.5f, 0.5f };
            scene.AddComponent(e, cc);
        }

        constexpr double fixedDt = 1.0 / 60.0;
        scene.SetFixedDeltaTime(fixedDt);

        // Pre-step 30 frames to initiate multi-body contacts
        for (int i = 0; i < 30; ++i)
        {
            scene.FixedUpdateSystems();
        }

        // Measure resolution time over the next 30 active physical steps
        constexpr int samples = 30;
        double totalResolutionMs = 0.0;
        size_t contactCountSum = 0;

        for (int i = 0; i < samples; ++i)
        {
            pipelinePtr->GetPhysicsSystem().IntegrateVelocities(fixedDt / 4.0);
            pipelinePtr->GetCollisionSystem().Detect(fixedDt / 4.0, 0.002);

            contactCountSum += pipelinePtr->GetCollisionSystem().GetContacts().size();

            auto start = std::chrono::high_resolution_clock::now();
            pipelinePtr->GetResolutionSystem().Resolve(fixedDt / 4.0, pipelinePtr->GetCollisionSystem().GetContactCache());
            auto end = std::chrono::high_resolution_clock::now();

            totalResolutionMs += std::chrono::duration<double, std::milli>(end - start).count();

            pipelinePtr->GetContinuousCollisionSystem().ExecuteCCD(fixedDt / 4.0, 0.002);
            pipelinePtr->GetPhysicsSystem().IntegratePositions(fixedDt / 4.0);
        }

        double avgMs = totalResolutionMs / samples;
        size_t avgContacts = contactCountSum / samples;

        std::ostringstream oss;
        oss << std::left << std::setw(4) << objectCount << " objs | "
            << std::setw(4) << avgContacts << " contacts | "
            << std::setw(15) << modeName << " | "
            << std::fixed << std::setprecision(3) << avgMs << " ms avg/substep";

        RT_LOG_INFO(oss.str());
    }
}

int main()
{
    RT_LOG_INFO("Solver Scaling Test (SI vs LCP on Active Constraints)");

    // Lemke exact formulation runs up to 96 points (24 boxes). 5, 10, and 20 span its valid domain.
    std::vector<int> counts = { 5, 10, 20 };

    for (int count : counts)
    {
        MeasureSolverDynamically(count, ResolutionSystem::SolverMode::SequentialImpulses, "SI (PGS)");
        MeasureSolverDynamically(count, ResolutionSystem::SolverMode::Exact, "LCP (Lemke)");
    }

    RT_LOG_INFO("Solver scaling benchmark completed.");
    return 0;
}