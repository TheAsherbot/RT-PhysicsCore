/**
 * @file ECSLifecycleTest.cpp
 * @brief Automated verification and visual demonstration of ECS entity lifecycle, recycling, and hierarchy cleanup.
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>

#include "RT-PhysicsCore/utils/Log.h"
#include "RT-PhysicsCore/utils/DebugDraw.h"
#include "RT-PhysicsCore/rendering/Renderer.h"

#include "RT-PhysicsCore/core/ecs/core/Scene.h"
#include "RT-PhysicsCore/core/ecs/systems/TransformPropagationSystem.h"
#include "RT-PhysicsCore/core/ecs/components/TransformComponent.h"
#include "RT-PhysicsCore/core/ecs/components/HierarchyComponent.h"
#include "RT-PhysicsCore/physics/components/RigidBodyComponent.h"
#include "RT-PhysicsCore/physics/components/ColliderComponent.h"

namespace
{
    using namespace RT_PhysicsCore;
}

int main(int argc, char* argv[])
{
    RT_LOG_INFO("ECS Lifecycle Test harness starting.");

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

    // ── Check 1: Multi-Component Dense Query Filtering ──
    {
        ++totalCount;
        Scene scene;
        std::vector<Entity> created;
        for (int i = 0; i < 100; ++i)
        {
            Entity e = scene.CreateEntity();
            created.push_back(e);
            TransformComponent tc;
            tc.position = { static_cast<float>(i), 0.0f, 0.0f };
            scene.AddComponent(e, tc);

            if (i % 2 == 0)
            {
                RigidBodyComponent rb;
                rb.invMass = 1.0;
                scene.AddComponent(e, rb);
            }
        }

        auto qTransform = scene.Query<TransformComponent>();
        auto qRigidBody = scene.Query<TransformComponent, RigidBodyComponent>();

        bool pass = (qTransform.size() == 100) && (qRigidBody.size() == 50);
        if (pass)
        {
            ++passCount;
        }

        std::ostringstream oss;
        oss << "qTransform=" << qTransform.size() << " (expected 100), qRigidBody=" << qRigidBody.size() << " (expected 50)";
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "Dense Query Filtering - " << oss.str());
    }

    // ── Check 2: Component Removal & Null Pointer Safety ──
    {
        ++totalCount;
        Scene scene;
        Entity e1 = scene.CreateEntity();
        Entity e2 = scene.CreateEntity();
        Entity e3 = scene.CreateEntity();

        RigidBodyComponent rb;
        rb.invMass = 1.0;
        scene.AddComponent(e1, rb);
        scene.AddComponent(e2, rb);
        scene.AddComponent(e3, rb);

        // Remove from middle entity e2
        scene.RemoveComponent<RigidBodyComponent>(e2);

        bool e2Has = scene.HasComponent<RigidBodyComponent>(e2);
        auto* e2Ptr = scene.GetComponent<RigidBodyComponent>(e2);
        bool e1Has = scene.HasComponent<RigidBodyComponent>(e1);
        bool e3Has = scene.HasComponent<RigidBodyComponent>(e3);

        auto remaining = scene.Query<RigidBodyComponent>();
        bool pass = (!e2Has) && (e2Ptr == nullptr) && e1Has && e3Has && (remaining.size() == 2);
        if (pass)
        {
            ++passCount;
        }

        std::ostringstream oss;
        oss << "e2Has=" << e2Has << ", e2PtrIsNull=" << (e2Ptr == nullptr) << ", remaining=" << remaining.size();
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "Component Removal & Null Safety - " << oss.str());
    }

    // ── Check 3: Mid-Step Entity Destruction & Active List Integrity ──
    {
        ++totalCount;
        Scene scene;
        std::vector<Entity> entities;
        for (int i = 0; i < 50; ++i)
        {
            Entity e = scene.CreateEntity();
            entities.push_back(e);
            TransformComponent tc;
            scene.AddComponent(e, tc);
        }

        // Destroy middle entity at index 25
        Entity destroyedEntity = entities[25];
        scene.DestroyEntity(destroyedEntity);

        const auto& active = scene.GetEntities();
        bool foundInActive = (std::find(active.begin(), active.end(), destroyedEntity) != active.end());
        bool hasComponentAfterDeath = scene.HasComponent<TransformComponent>(destroyedEntity);

        bool pass = (active.size() == 49) && (!foundInActive) && (!hasComponentAfterDeath);
        if (pass)
        {
            ++passCount;
        }

        std::ostringstream oss;
        oss << "activeSize=" << active.size() << " (expected 49), found=" << foundInActive
            << ", hasComp=" << hasComponentAfterDeath;
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "Mid-Step Destruction Integrity - " << oss.str());
    }

    // ── Check 4: Entity ID Recycling & Generational Safety ──
    {
        ++totalCount;
        Scene scene;
        Entity e1 = scene.CreateEntity();
        Entity e2 = scene.CreateEntity();
        Entity e3 = scene.CreateEntity();

        scene.DestroyEntity(e2);
        Entity recycled = scene.CreateEntity();

        // The destroyed ID (e2) must be recycled on the subsequent creation
        bool pass = (recycled == e2);
        if (pass)
        {
            ++passCount;
        }

        std::ostringstream oss;
        oss << "recycledID=" << recycled << " (expected " << e2 << ")";
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "Entity ID Recycling - " << oss.str());
    }

    // ── Check 5: Hierarchy Detachment on Parent Destruction ──
    {
        ++totalCount;
        Scene scene;
        Entity parent = scene.CreateEntity();
        Entity child1 = scene.CreateEntity();
        Entity child2 = scene.CreateEntity();

        scene.SetParent(child1, parent);
        scene.SetParent(child2, parent);

        scene.DestroyEntity(parent);

        auto* h1 = scene.GetComponent<HierarchyComponent>(child1);
        auto* h2 = scene.GetComponent<HierarchyComponent>(child2);

        bool c1Detached = h1 && (h1->parent == invalidEntity);
        bool c2Detached = h2 && (h2->parent == invalidEntity);

        bool pass = c1Detached && c2Detached;
        if (pass)
        {
            ++passCount;
        }

        std::ostringstream oss;
        oss << "c1Detached=" << c1Detached << ", c2Detached=" << c2Detached;
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "Hierarchy Cleanup on Destruction - " << oss.str());
    }

    RT_LOG_INFO(passCount << "/" << totalCount << " automated checks passed.");

    if (!enableGui)
    {
        RT_LOG_INFO("Headless execution complete (pass --gui to open interactive visualizer).");
        return (passCount == totalCount) ? 0 : 1;
    }

    // ── Phase 2: Interactive visualization ──

    Renderer renderer(1280, 720, "RT-PhysicsCore ECS Lifecycle Test");
    if (!renderer.IsValid())
    {
        return 1;
    }

    Scene scene;
    auto tps = std::make_unique<TransformPropagationSystem>(scene);

    // Planetary hierarchy: Central sun parent with orbiting moons
    Entity sun = scene.CreateEntity();
    TransformComponent stc;
    stc.position = { 0.0f, 0.0f, 0.0f };
    scene.AddComponent(sun, stc);

    constexpr int kMoonCount = 8;
    std::vector<Entity> moons;
    for (int i = 0; i < kMoonCount; ++i)
    {
        Entity m = scene.CreateEntity();
        TransformComponent mtc;
        float angle = (2.0f * 3.14159f * i) / kMoonCount;
        mtc.position = { std::cos(angle) * 3.0f, std::sin(angle) * 3.0f, 0.0f };
        scene.AddComponent(m, mtc);
        scene.SetParent(m, sun);
        moons.push_back(m);
    }

    auto lastTime = std::chrono::steady_clock::now();
    float totalTime = 0.0f;
    int churnCounter = 0;

    while (!renderer.ShouldClose())
    {
        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - lastTime).count();
        lastTime = now;
        if (dt > 0.1f)
        {
            dt = 0.1f;
        }
        totalTime += dt;

        // Rotate central sun to demonstrate hierarchy propagation
        auto* sunTc = scene.GetComponent<TransformComponent>(sun);
        if (sunTc)
        {
            sunTc->rotation = glm::angleAxis(totalTime * 1.5f, glm::vec3(0, 0, 1));
        }

        // Periodically destroy and recycle orbital nodes
        churnCounter++;
        if (churnCounter % 40 == 0 && !moons.empty())
        {
            Entity doomed = moons.back();
            moons.pop_back();
            scene.DestroyEntity(doomed);
        }
        else if (churnCounter % 40 == 20 && moons.size() < kMoonCount)
        {
            Entity reborn = scene.CreateEntity();
            TransformComponent rtc;
            float angle = totalTime * 2.0f;
            rtc.position = { std::cos(angle) * 3.0f, std::sin(angle) * 3.0f, 0.0f };
            scene.AddComponent(reborn, rtc);
            scene.SetParent(reborn, sun);
            moons.push_back(reborn);
        }

        tps->Update();

        renderer.BeginFrame();

        // Render central sun
        if (sunTc)
        {
            DebugDraw::Sphere(sunTc->position, 0.8f, { 1.0f, 0.9f, 0.2f });
        }

        // Render orbiting children and connecting hierarchy rays
        for (Entity m : moons)
        {
            auto* wt = scene.GetComponent<WorldTransformComponent>(m);
            if (wt)
            {
                DebugDraw::Sphere(wt->worldPosition, 0.35f, { 0.2f, 0.8f, 1.0f });
                if (sunTc)
                {
                    DebugDraw::Line(sunTc->position, wt->worldPosition, { 0.5f, 0.5f, 0.5f }, false);
                }
            }
        }

        renderer.FlushDebugDraw();
        renderer.EndFrame();
    }

    return 0;
}