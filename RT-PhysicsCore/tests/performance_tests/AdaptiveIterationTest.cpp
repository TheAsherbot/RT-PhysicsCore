/**
 * @file AdaptiveIterationTest.cpp
 * @brief Verifies solver stability, iteration budgeting, and relative performance scaling.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "RT-PhysicsCore/core/math/MathTypes.h"
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

    struct BudgetProfile
    {
        std::string name;
        int velocityIterations;
        int positionIterations;
    };

    struct BenchmarkResult
    {
        std::string name;
        double avgResolveMs;
        double avgFrameMs;
        double peakFrameMs;
        float maxVelocity;
        float minFloorY;
        bool stable;
    };

    BenchmarkResult RunBudgetTest(const BudgetProfile& profile, int columnsPerSide, int boxesPerColumn, int frameCount)
    {
        Scene scene;
        scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
        auto pipeline = std::make_unique<PhysicsPipelineSystem>(
            scene, ResolutionSystem::SolverMode::SequentialImpulses, 4);
        PhysicsPipelineSystem* pipelinePtr = pipeline.get();
        scene.AddSystem(std::move(pipeline));

        pipelinePtr->GetResolutionSystem().SetVelocityIterations(profile.velocityIterations);
        pipelinePtr->GetResolutionSystem().SetPositionIterations(profile.positionIterations);

        // Ground slab
        Entity ground = scene.CreateEntity();
        TransformComponent gtc;
        gtc.position = { 0.0f, -0.5f, 0.0f };
        scene.AddComponent(ground, gtc);
        scene.AddComponent(ground, MakeStaticBody());
        ColliderComponent gcc;
        gcc.shape = ColliderShape::Box;
        gcc.size = { 50.0f, 0.5f, 50.0f };
        scene.AddComponent(ground, gcc);

        // Uniform multi-column grid of stacked boxes ensuring equal contact counts
        std::vector<Entity> dynamicBoxes;
        dynamicBoxes.reserve(columnsPerSide * columnsPerSide * boxesPerColumn);

        for (int colX = 0; colX < columnsPerSide; ++colX)
        {
            for (int colZ = 0; colZ < columnsPerSide; ++colZ)
            {
                float cx = (static_cast<float>(colX) - static_cast<float>(columnsPerSide - 1) * 0.5f) * 2.5f;
                float cz = (static_cast<float>(colZ) - static_cast<float>(columnsPerSide - 1) * 0.5f) * 2.5f;

                for (int i = 0; i < boxesPerColumn; ++i)
                {
                    Entity e = scene.CreateEntity();
                    TransformComponent tc;
                    tc.position = { cx, 0.5f + static_cast<float>(i) * 1.0f, cz };
                    scene.AddComponent(e, tc);
                    scene.AddComponent(e, MakeDynamicBody(1.0f, ComputeBoxInertia(1.0f, { 0.5f, 0.5f, 0.5f })));
                    ColliderComponent cc;
                    cc.shape = ColliderShape::Box;
                    cc.size = { 0.5f, 0.5f, 0.5f };
                    scene.AddComponent(e, cc);
                    dynamicBoxes.push_back(e);
                }
            }
        }

        constexpr double fixedDt = 1.0 / 60.0;
        scene.SetFixedDeltaTime(fixedDt);

        // Pre-step 20 frames so contacts form uniformly
        for (int i = 0; i < 20; ++i)
        {
            scene.FixedUpdateSystems();
        }

        double totalResolveMs = 0.0;
        double totalFrameMs = 0.0;
        double peakFrameMs = 0.0;
        float maxVel = 0.0f;
        float minBottomY = 1000.0f;
        bool hasNaN = false;

        double h = fixedDt / 4.0;

        for (int frame = 0; frame < frameCount; ++frame)
        {
            auto frameStart = std::chrono::high_resolution_clock::now();

            for (int step = 0; step < 4; ++step)
            {
                pipelinePtr->GetPhysicsSystem().IntegrateVelocities(h);
                pipelinePtr->GetCollisionSystem().Detect(h, 0.002);

                auto rStart = std::chrono::high_resolution_clock::now();
                pipelinePtr->GetResolutionSystem().Resolve(h, pipelinePtr->GetCollisionSystem().GetContactCache());
                auto rEnd = std::chrono::high_resolution_clock::now();

                totalResolveMs += std::chrono::duration<double, std::milli>(rEnd - rStart).count();

                pipelinePtr->GetContinuousCollisionSystem().ExecuteCCD(h, 0.002);
                pipelinePtr->GetPhysicsSystem().IntegratePositions(h);
            }

            auto frameEnd = std::chrono::high_resolution_clock::now();
            double stepMs = std::chrono::duration<double, std::milli>(frameEnd - frameStart).count();
            totalFrameMs += stepMs;
            peakFrameMs = std::max(peakFrameMs, stepMs);

            // Audit simulation stability
            for (Entity e : dynamicBoxes)
            {
                auto* rb = scene.GetComponent<RigidBodyComponent>(e);
                auto* tc = scene.GetComponent<TransformComponent>(e);
                if (rb && tc)
                {
                    if (std::isnan(rb->velocity.x) || std::isnan(rb->velocity.y) || std::isnan(rb->velocity.z))
                    {
                        hasNaN = true;
                    }

                    float speed = glm::length(rb->velocity);
                    maxVel = std::max(maxVel, speed);
                    minBottomY = std::min(minBottomY, ToFloat(tc->position.y));
                }
            }
        }

        double avgResolve = totalResolveMs / frameCount;
        double avgFrame = totalFrameMs / frameCount;

        bool noFloorTunneling = (minBottomY >= -0.1f);
        bool boundedVelocity = (maxVel < 20.0f);
        bool isStable = (!hasNaN && noFloorTunneling && boundedVelocity);

        return { profile.name, avgResolve, avgFrame, peakFrameMs, maxVel, minBottomY, isStable };
    }
}

int main()
{
    RT_LOG_INFO("Adaptive Iteration & Budget Scaling Test starting.");

    constexpr int kColumnsPerSide = 2; // 4 columns
    constexpr int kBoxesPerColumn = 10; // 40 dynamic boxes total
    constexpr int kFrameCount = 50;

    BudgetProfile fastBudget{ "Fast Budget (4 vel, 2 pos)", 4, 2 };
    BudgetProfile qualityBudget{ "Quality Budget (12 vel, 4 pos)", 12, 4 };

    BenchmarkResult resFast = RunBudgetTest(fastBudget, kColumnsPerSide, kBoxesPerColumn, kFrameCount);
    BenchmarkResult resQuality = RunBudgetTest(qualityBudget, kColumnsPerSide, kBoxesPerColumn, kFrameCount);

    RT_LOG_INFO("=== Benchmark Results (40 stacked boxes across 4 columns, " << kFrameCount << " frames) ===");
    RT_LOG_INFO(resFast.name << " | Resolve: " << std::fixed << std::setprecision(3)
        << resFast.avgResolveMs << " ms/f | Total: " << resFast.avgFrameMs
        << " ms/f | Stable: " << (resFast.stable ? "YES" : "NO"));
    RT_LOG_INFO(resQuality.name << " | Resolve: " << std::fixed << std::setprecision(3)
        << resQuality.avgResolveMs << " ms/f | Total: " << resQuality.avgFrameMs
        << " ms/f | Stable: " << (resQuality.stable ? "YES" : "NO"));

    // Fast budget solver resolution must strictly scale lower than Quality budget
    bool scalingValid = (resFast.avgResolveMs < resQuality.avgResolveMs);
    bool bothStable = (resFast.stable && resQuality.stable);
    bool pass = (bothStable && scalingValid);

    if (pass)
    {
        RT_LOG_INFO("[PASS] Adaptive Iteration scaling verified. Fast budget demonstrated lower cost while preserving stability.");
    }
    else
    {
        RT_LOG_ERROR("[FAIL] Adaptive Iteration test failed stability or scaling criteria.");
    }

    return pass ? 0 : 1;
}