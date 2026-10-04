/**
 * @file PositionCorrectionTest.cpp
 * @brief Validates the split-impulse position correction mechanism.
 *
 * Injects deeply overlapping bodies into the scene to ensure they separate over time
 * via position translation without gaining explosive kinetic energy.
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

#include "RT-PhysicsCore/utils/Log.h"
#include "RT-PhysicsCore/utils/DebugDraw.h"
#include "RT-PhysicsCore/rendering/Renderer.h"

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

    void StepScene(Scene& scene, int ticks, double dt = 1.0 / 60.0)
    {
        for (int i = 0; i < ticks; ++i)
        {
            scene.SetFixedDeltaTime(dt);
            scene.FixedUpdateSystems();
        }
    }
}

int main()
{
    RT_LOG_INFO("Position Correction Test harness starting.");

    int passCount = 0;
    int totalCount = 0;
    constexpr double fixedDt = 1.0 / 60.0;

    {
        ++totalCount;
        Scene scene;
        scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
        auto phys = std::make_unique<PhysicsSystem>(scene);
        phys->SetGravity({ 0.0f, 0.0f, 0.0f }); // disable gravity to isolate explosion behavior
        scene.AddSystem(std::move(phys));

        auto colSys = std::make_unique<CollisionSystem>(scene);
        scene.AddSystem(std::make_unique<ResolutionSystem>(scene, *colSys));
        scene.AddSystem(std::move(colSys));

        // Spawn two deeply overlapping spheres
        float radius = 1.0f;

        Entity a = scene.CreateEntity();
        TransformComponent tca;
        tca.position = { -0.2f, 0.0f, 0.0f }; // Should be at -1.0 to touch, so 0.8m overlap!
        scene.AddComponent(a, tca);
        scene.AddComponent(a, MakeDynamicBody(1.0f, ComputeSphereInertia(1.0f, radius)));
        ColliderComponent cca;
        cca.shape = ColliderShape::Sphere;
        cca.size = { radius, 0.0f, 0.0f };
        scene.AddComponent(a, cca);

        Entity b = scene.CreateEntity();
        TransformComponent tcb;
        tcb.position = { 0.2f, 0.0f, 0.0f };
        scene.AddComponent(b, tcb);
        scene.AddComponent(b, MakeDynamicBody(1.0f, ComputeSphereInertia(1.0f, radius)));
        ColliderComponent ccb;
        ccb.shape = ColliderShape::Sphere;
        ccb.size = { radius, 0.0f, 0.0f };
        scene.AddComponent(b, ccb);

        // Run for a few ticks to let split impulse separate them
        StepScene(scene, 10, fixedDt);

        auto* finalTca = scene.GetComponent<TransformComponent>(a);
        auto* finalTcb = scene.GetComponent<TransformComponent>(b);
        auto* rba = scene.GetComponent<RigidBodyComponent>(a);
        auto* rbb = scene.GetComponent<RigidBodyComponent>(b);

        float distance = glm::length(finalTca->position - finalTcb->position);
        float maxVel = std::max(glm::length(rba->velocity), glm::length(rbb->velocity));

        // They should be separated (dist > 1.95) and have very low velocity (split impulse doesn't add true velocity)
        bool pass = distance >= 1.9f && maxVel < 0.5f;
        if (pass) ++passCount;

        std::ostringstream oss;
        oss << "Separation=" << distance << " (expected ~2.0), maxVel=" << maxVel << " (expected near 0)";
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "Overlap Injection Separation - " << oss.str());
    }

    RT_LOG_INFO(passCount << "/" << totalCount << " automated checks passed.");

    // ── Phase 2: Interactive visualization ──

    Renderer renderer(1280, 720, "RT-PhysicsCore Position Correction Test");
    if (!renderer.IsValid()) return 1;

    Scene scene;
    scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
    auto phys = std::make_unique<PhysicsSystem>(scene);
    phys->SetGravity({ 0.0f, 0.0f, 0.0f });
    scene.AddSystem(std::move(phys));
    auto colSys = std::make_unique<CollisionSystem>(scene);
    scene.AddSystem(std::make_unique<ResolutionSystem>(scene, *colSys));
    scene.AddSystem(std::move(colSys));

    auto lastTime = std::chrono::steady_clock::now();
    double physicsAccum = 0.0;
    double spawnTimer = 0.0;

    while (!renderer.ShouldClose())
    {
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - lastTime).count();
        lastTime = now;
        if (dt > 0.1) dt = 0.1;

        spawnTimer += dt;
        if (spawnTimer > 2.0)
        {
            spawnTimer = 0.0;
            // Clear old bodies and spawn two new overlapping ones
            std::vector<Entity> toDestroy = scene.GetEntities();
            for (Entity e : toDestroy)
            {
                scene.DestroyEntity(e);
            }

            Entity a = scene.CreateEntity();
            TransformComponent tca;
            tca.position = { -0.1f, 0.0f, 0.0f };
            scene.AddComponent(a, tca);
            scene.AddComponent(a, MakeDynamicBody(1.0f, ComputeSphereInertia(1.0f, 1.0f)));
            ColliderComponent cca;
            cca.shape = ColliderShape::Sphere;
            cca.size = { 1.0f, 0.0f, 0.0f };
            scene.AddComponent(a, cca);

            Entity b = scene.CreateEntity();
            TransformComponent tcb;
            tcb.position = { 0.1f, 0.0f, 0.0f };
            scene.AddComponent(b, tcb);
            scene.AddComponent(b, MakeDynamicBody(1.0f, ComputeSphereInertia(1.0f, 1.0f)));
            ColliderComponent ccb;
            ccb.shape = ColliderShape::Sphere;
            ccb.size = { 1.0f, 0.0f, 0.0f };
            scene.AddComponent(b, ccb);
        }

        physicsAccum += dt;
        while (physicsAccum >= fixedDt)
        {
            scene.SetFixedDeltaTime(fixedDt);
            scene.FixedUpdateSystems();
            physicsAccum -= fixedDt;
        }

        for (Entity e : scene.Query<TransformComponent, ColliderComponent>())
        {
            auto* tc = scene.GetComponent<TransformComponent>(e);
            auto* cc = scene.GetComponent<ColliderComponent>(e);
            DebugDraw::Sphere(tc->position, cc->size.x, { 1.0f, 0.4f, 0.4f });
        }

        renderer.BeginFrame();
        renderer.FlushDebugDraw();
        renderer.EndFrame();
    }

    return 0;
}