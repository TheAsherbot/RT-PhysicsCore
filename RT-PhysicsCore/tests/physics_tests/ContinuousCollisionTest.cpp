/**
 * @file ContinuousCollisionTest.cpp
 * @brief Validates continuous collision detection (CCD), swept ShapeCast TOI, and anti-tunneling clamping.
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "RT-PhysicsCore/utils/Log.h"
#include "RT-PhysicsCore/utils/DebugDraw.h"
#include "RT-PhysicsCore/rendering/Renderer.h"

#include "RT-PhysicsCore/core/ecs/core/Scene.h"
#include "RT-PhysicsCore/core/ecs/systems/TransformPropagationSystem.h"
#include "RT-PhysicsCore/core/ecs/components/TransformComponent.h"
#include "RT-PhysicsCore/physics/systems/PhysicsPipelineSystem.h"
#include "RT-PhysicsCore/physics/components/RigidBodyComponent.h"
#include "RT-PhysicsCore/physics/components/ColliderComponent.h"
#include "RT-PhysicsCore/physics/components/PhysicsMaterialComponent.h"
#include "RT-PhysicsCore/physics/collision/ShapeCast.h"
#include "RT-PhysicsCore/physics/collision/Support.h"
#include "RT-PhysicsCore/physics/MassProperties.h"
#include "RT-PhysicsCore/physics/PhysicsMaterial.h"

#include <glm/gtc/quaternion.hpp>

namespace
{
    using namespace RT_PhysicsCore;

    /**
     * @brief Steps the simulation forward by a given tick count.
     * @param scene The active ECS scene.
     * @param ticks The number of fixed update ticks to execute.
     * @param dt The fixed delta time per tick.
     */
    void StepScene(Scene& scene, int ticks, double dt = 1.0 / 60.0)
    {
        for (int i = 0; i < ticks; ++i)
        {
            scene.SetFixedDeltaTime(dt);
            scene.FixedUpdateSystems();
        }
    }
}

int main(int argc, char** argv)
{
    RT_LOG_INFO("Continuous Collision Detection (CCD) Test harness starting.");

    bool enableGui = false;
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "--gui" || arg == "-g" || arg == "--visual" || arg == "-v")
        {
            enableGui = true;
            break;
        }
    }

    int passCount = 0;
    int totalCount = 0;
    constexpr double fixedDt = 1.0 / 60.0;

    // ── Check 1: High-Speed Bullet vs Thin Wall (Pipeline Tunneling Prevention) ──
    {
        ++totalCount;
        Scene scene;
        scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
        auto pipeline = std::make_unique<PhysicsPipelineSystem>(
            scene, ResolutionSystem::SolverMode::SequentialImpulses, 4);
        pipeline->GetPhysicsSystem().SetGravity({ 0.0f, 0.0f, 0.0f });
        scene.AddSystem(std::move(pipeline));

        // Thin wall at x = 0.0, thickness = 0.1m (half-extent = 0.05m)
        Entity wall = scene.CreateEntity();
        TransformComponent wtc;
        wtc.position = { 0.0f, 0.0f, 0.0f };
        scene.AddComponent(wall, wtc);
        scene.AddComponent(wall, MakeStaticBody());
        ColliderComponent wcc;
        wcc.shape = ColliderShape::Box;
        wcc.size = { 0.05f, 5.0f, 5.0f };
        scene.AddComponent(wall, wcc);
        PhysicsMaterialComponent wpm;
        wpm.material = MaterialId::Clay;
        scene.AddComponent(wall, wpm);

        // Bullet at x = -2.0m, radius = 0.1m, moving at +150 m/s
        // In 1 frame (1/60s), displacement is 2.5m -> discrete physics would land at x = +0.5m (complete tunnel!)
        Entity bullet = scene.CreateEntity();
        TransformComponent btc;
        btc.position = { -2.0f, 0.0f, 0.0f };
        scene.AddComponent(bullet, btc);
        RigidBodyComponent brb = MakeDynamicBody(0.5f, ComputeSphereInertia(0.5f, 0.1f));
        brb.velocity = { 150.0f, 0.0f, 0.0f };
        scene.AddComponent(bullet, brb);
        ColliderComponent bcc;
        bcc.shape = ColliderShape::Sphere;
        bcc.size = { 0.1f, 0.0f, 0.0f };
        scene.AddComponent(bullet, bcc);
        PhysicsMaterialComponent bpm;
        bpm.material = MaterialId::Clay; // zero restitution for clean stopping at barrier
        scene.AddComponent(bullet, bpm);

        StepScene(scene, 1, fixedDt);

        auto* finalBtc = scene.GetComponent<TransformComponent>(bullet);
        // Bullet must not pass through the wall: position.x must remain on the left side (<= 0.0)
        bool pass = (finalBtc->position.x <= 0.0f) && (finalBtc->position.x >= -0.25f);
        if (pass)
        {
            ++passCount;
        }

        std::ostringstream oss;
        oss << "BulletFinalX=" << finalBtc->position.x << " (expected in [-0.25, 0.0], discrete would be +0.5)";
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "High-Speed Bullet Anti-Tunneling - " << oss.str());
    }

    // ── Check 2: Direct ShapeCast TOI Analytical Accuracy ──
    {
        ++totalCount;
        ConvexCore coreA;
        coreA.shape = ColliderShape::Sphere;
        coreA.size = { 0.5f, 0.0f, 0.0f };
        coreA.position = { -5.0, 0.0, 0.0 };
        coreA.rotation = glm::mat3(1.0);
        coreA.coreRadius = 0.5;

        ConvexCore coreB;
        coreB.shape = ColliderShape::Sphere;
        coreB.size = { 0.5f, 0.0f, 0.0f };
        coreB.position = { 0.0, 0.0, 0.0 };
        coreB.rotation = glm::mat3(1.0);
        coreB.coreRadius = 0.5;

        Vec3 linDispA{ 10.0, 0.0, 0.0 };
        Vec3 zeroVec{ 0.0, 0.0, 0.0 };

        ShapeCastResult res = ShapeCast::Cast(
            coreA, linDispA, zeroVec, 0.5,
            coreB, zeroVec, zeroVec, 0.5,
            1.0, 0.001, 32);

        float expectedToi = 0.3999f;
        float err = std::abs(static_cast<float>(res.timeOfImpact) - expectedToi);
        bool pass = res.hasHit && (err < 0.005f);
        if (pass)
        {
            ++passCount;
        }

        std::ostringstream oss;
        oss << "hasHit=" << res.hasHit << " TOI=" << res.timeOfImpact << " Expected=" << expectedToi << " err=" << err;
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "ShapeCast TOI Analytical Accuracy - " << oss.str());
    }

    // ── Check 3: Near-Miss Parallel Glancing (No False Positives) ──
    {
        ++totalCount;
        ConvexCore coreA;
        coreA.shape = ColliderShape::Sphere;
        coreA.size = { 0.5f, 0.0f, 0.0f };
        coreA.position = { -5.0, 1.2, 0.0 };
        coreA.rotation = glm::mat3(1.0);
        coreA.coreRadius = 0.5;

        ConvexCore coreB;
        coreB.shape = ColliderShape::Box;
        coreB.size = { 0.5f, 0.5f, 0.5f };
        coreB.position = { 0.0, 0.0, 0.0 };
        coreB.rotation = glm::mat3(1.0);
        coreB.coreRadius = 0.0;

        Vec3 linDispA{ 10.0, 0.0, 0.0 };
        Vec3 zeroVec{ 0.0, 0.0, 0.0 };

        ShapeCastResult res = ShapeCast::Cast(
            coreA, linDispA, zeroVec, 0.5,
            coreB, zeroVec, zeroVec, 0.866,
            1.0, 0.001, 32);

        bool pass = !res.hasHit;
        if (pass)
        {
            ++passCount;
        }

        std::ostringstream oss;
        oss << "hasHit=" << res.hasHit << " (expected false, clearance=0.2m)";
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "Near-Miss Parallel Motion (No False Hits) - " << oss.str());
    }

    // ── Check 4: Fast Tumbling Box Swept Advancement ──
    {
        ++totalCount;
        ConvexCore coreA;
        coreA.shape = ColliderShape::Box;
        coreA.size = { 0.5f, 0.5f, 0.5f };
        coreA.position = { -4.0, 0.0, 0.0 };
        coreA.rotation = glm::mat3(1.0);
        coreA.coreRadius = 0.0;

        ConvexCore coreB;
        coreB.shape = ColliderShape::Box;
        coreB.size = { 0.2f, 5.0f, 5.0f };
        coreB.position = { 0.0, 0.0, 0.0 };
        coreB.rotation = glm::mat3(1.0);
        coreB.coreRadius = 0.0;

        Vec3 linDispA{ 8.0, 0.0, 0.0 };
        Vec3 angVelA{ 0.0, 0.0, 10.0 };
        Vec3 zeroVec{ 0.0, 0.0, 0.0 };

        ShapeCastResult res = ShapeCast::Cast(
            coreA, linDispA, angVelA, 0.866,
            coreB, zeroVec, zeroVec, 5.0,
            1.0, 0.005, 32);

        bool pass = res.hasHit && (res.timeOfImpact > 0.1f) && (res.timeOfImpact < 0.6f);
        if (pass)
        {
            ++passCount;
        }

        std::ostringstream oss;
        oss << "hasHit=" << res.hasHit << " TOI=" << res.timeOfImpact << " (expected ~0.4)";
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "Spinning Box Swept TOI - " << oss.str());
    }

    RT_LOG_INFO(passCount << "/" << totalCount << " automated checks passed.");

    if (!enableGui)
    {
        RT_LOG_INFO("Headless execution complete (pass --gui to open interactive visualizer).");
        return (passCount == totalCount) ? 0 : 1;
    }

    // ── Phase 2: Interactive visualization ──

    Renderer renderer(1280, 720, "RT-PhysicsCore Continuous Collision Test");
    if (!renderer.IsValid())
    {
        return 1;
    }

    Scene scene;
    scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
    auto pipeline = std::make_unique<PhysicsPipelineSystem>(
        scene, ResolutionSystem::SolverMode::SequentialImpulses, 4);
    pipeline->GetPhysicsSystem().SetGravity({ 0.0f, 0.0f, 0.0f });
    scene.AddSystem(std::move(pipeline));

    auto setupDemo = [&]()
        {
            for (Entity e : scene.GetEntities())
            {
                scene.DestroyEntity(e);
            }

            // Wall 1: Thin static barrier
            Entity w1 = scene.CreateEntity();
            TransformComponent wtc1;
            wtc1.position = { 0.0f, 2.0f, 0.0f };
            scene.AddComponent(w1, wtc1);
            scene.AddComponent(w1, MakeStaticBody());
            ColliderComponent wcc1;
            wcc1.shape = ColliderShape::Box;
            wcc1.size = { 0.05f, 2.0f, 2.0f };
            scene.AddComponent(w1, wcc1);

            // Wall 2: Angled barrier
            Entity w2 = scene.CreateEntity();
            TransformComponent wtc2;
            wtc2.position = { 0.0f, -2.0f, 0.0f };
            wtc2.rotation = glm::angleAxis(glm::radians(30.0f), glm::vec3(0, 0, 1));
            scene.AddComponent(w2, wtc2);
            scene.AddComponent(w2, MakeStaticBody());
            ColliderComponent wcc2;
            wcc2.shape = ColliderShape::Box;
            wcc2.size = { 0.05f, 2.0f, 2.0f };
            scene.AddComponent(w2, wcc2);

            // Fast projectile 1 (toward Wall 1)
            Entity p1 = scene.CreateEntity();
            TransformComponent ptc1;
            ptc1.position = { -8.0f, 2.0f, 0.0f };
            scene.AddComponent(p1, ptc1);
            RigidBodyComponent prb1 = MakeDynamicBody(1.0f, ComputeSphereInertia(1.0f, 0.2f));
            prb1.velocity = { 100.0f, 0.0f, 0.0f };
            scene.AddComponent(p1, prb1);
            ColliderComponent pcc1;
            pcc1.shape = ColliderShape::Sphere;
            pcc1.size = { 0.2f, 0.0f, 0.0f };
            scene.AddComponent(p1, pcc1);

            // Fast projectile 2 (toward Wall 2)
            Entity p2 = scene.CreateEntity();
            TransformComponent ptc2;
            ptc2.position = { -8.0f, -2.0f, 0.0f };
            scene.AddComponent(p2, ptc2);
            RigidBodyComponent prb2 = MakeDynamicBody(1.0f, ComputeSphereInertia(1.0f, 0.2f));
            prb2.velocity = { 100.0f, 0.0f, 0.0f };
            scene.AddComponent(p2, prb2);
            ColliderComponent pcc2;
            pcc2.shape = ColliderShape::Sphere;
            pcc2.size = { 0.2f, 0.0f, 0.0f };
            scene.AddComponent(p2, pcc2);
        };

    setupDemo();

    auto lastTime = std::chrono::steady_clock::now();
    double cycleElapsed = 0.0;
    double physicsAccum = 0.0;

    while (!renderer.ShouldClose())
    {
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - lastTime).count();
        lastTime = now;
        if (dt > 0.1)
        {
            dt = 0.1;
        }

        cycleElapsed += dt;
        if (cycleElapsed >= 2.5)
        {
            cycleElapsed = 0.0;
            setupDemo();
        }

        physicsAccum += dt;
        while (physicsAccum >= fixedDt)
        {
            scene.SetFixedDeltaTime(fixedDt);
            scene.FixedUpdateSystems();
            physicsAccum -= fixedDt;
        }

        renderer.BeginFrame();

        for (Entity e : scene.Query<TransformComponent, ColliderComponent>())
        {
            auto* tc = scene.GetComponent<TransformComponent>(e);
            auto* cc = scene.GetComponent<ColliderComponent>(e);
            auto* rb = scene.GetComponent<RigidBodyComponent>(e);

            glm::vec3 color = (rb && rb->invMass > 0.0f) ? glm::vec3(1.0f, 0.4f, 0.2f) : glm::vec3(0.3f, 0.8f, 0.5f);

            if (cc->shape == ColliderShape::Sphere)
            {
                DebugDraw::Sphere(tc->position, cc->size.x, color);
            }
            else
            {
                glm::mat3 rot = glm::mat3_cast(tc->rotation);
                glm::vec3 c[8];
                int idx = 0;
                for (float sx : { -1.0f, 1.0f })
                {
                    for (float sy : { -1.0f, 1.0f })
                    {
                        for (float sz : { -1.0f, 1.0f })
                        {
                            c[idx++] = tc->position + rot[0] * sx * cc->size.x + rot[1] * sy * cc->size.y + rot[2] * sz * cc->size.z;
                        }
                    }
                }
                static const int edges[12][2] = {
                    {0,1},{0,2},{0,4},{3,1},{3,2},{3,7},
                    {5,1},{5,4},{5,7},{6,2},{6,4},{6,7}
                };
                for (auto& edge : edges)
                {
                    DebugDraw::Line(c[edge[0]], c[edge[1]], color);
                }
            }
        }

        // Guide lines
        DebugDraw::Line({ -8.0f, 2.0f, 0.0f }, { 2.0f, 2.0f, 0.0f }, { 0.3f, 0.3f, 0.3f }, false);
        DebugDraw::Line({ -8.0f, -2.0f, 0.0f }, { 2.0f, -2.0f, 0.0f }, { 0.3f, 0.3f, 0.3f }, false);

        renderer.FlushDebugDraw();
        renderer.EndFrame();
    }

    return 0;
}