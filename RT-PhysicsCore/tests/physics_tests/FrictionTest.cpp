/**
 * @file FrictionTest.cpp
 * @brief Validates Coulomb friction deceleration and stopping.
 *
 * Simulates a block sliding on a flat plane with an initial velocity. Checks if
 * kinetic friction stops the block at the expected theoretical distance: d = v^2 / (2 * mu_k * g).
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
#include "RT-PhysicsCore/physics/PhysicsMaterial.h"

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
    RT_LOG_INFO("Friction Test harness starting.");

    // Default material properties: mu_k = 0.4.
    // Self-colliding materials use geometric mean: sqrt(0.4 * 0.4) = 0.4.
    MaterialProperties mat = GetMaterialProperties(MaterialId::Default);
    float muk = mat.kineticFriction;
    float gravity = 9.80665f;

    int passCount = 0;
    int totalCount = 0;
    constexpr double fixedDt = 1.0 / 60.0;

    {
        ++totalCount;
        Scene scene;
        scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
        scene.AddSystem(std::make_unique<PhysicsSystem>(scene));
        auto colSys = std::make_unique<CollisionSystem>(scene);
        scene.AddSystem(std::make_unique<ResolutionSystem>(scene, *colSys));
        scene.AddSystem(std::move(colSys));

        Entity ground = scene.CreateEntity();
        TransformComponent gtc;
        gtc.position = { 0.0f, -0.5f, 0.0f };
        scene.AddComponent(ground, gtc);
        scene.AddComponent(ground, MakeStaticBody());
        ColliderComponent gcc;
        gcc.shape = ColliderShape::Box;
        gcc.size = { 20.0f, 0.5f, 20.0f };
        scene.AddComponent(ground, gcc);

        Entity box = scene.CreateEntity();
        TransformComponent btc;
        btc.position = { -10.0f, 0.5f, 0.0f };
        scene.AddComponent(box, btc);

        float v0 = 5.0f;
        RigidBodyComponent rb = MakeDynamicBody(1.0f, ComputeBoxInertia(1.0f, { 0.5f, 0.5f, 0.5f }));
        rb.velocity = { v0, 0.0f, 0.0f };
        scene.AddComponent(box, rb);

        ColliderComponent bcc;
        bcc.shape = ColliderShape::Box;
        bcc.size = { 0.5f, 0.5f, 0.5f };
        scene.AddComponent(box, bcc);

        float expectedDistance = (v0 * v0) / (2.0f * muk * gravity);
        float expectedFinalX = -10.0f + expectedDistance;

        // Simulate until it stops (v_x approaches 0)
        for (int i = 0; i < 300; ++i)
        {
            StepScene(scene, 1, fixedDt);
            auto* b_rb = scene.GetComponent<RigidBodyComponent>(box);
            if (b_rb->velocity.x <= 0.01f) break;
        }

        auto* finalTc = scene.GetComponent<TransformComponent>(box);
        float err = std::abs(finalTc->position.x - expectedFinalX);
        bool pass = err < 0.2f;
        if (pass) ++passCount;

        std::ostringstream oss;
        oss << "FinalX=" << finalTc->position.x << " Expected=" << expectedFinalX << " err=" << err;
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "Sliding Friction Deceleration - " << oss.str());
    }

    RT_LOG_INFO(passCount << "/" << totalCount << " automated checks passed.");

    // ── Phase 2: Interactive visualization ──

    Renderer renderer(1280, 720, "RT-PhysicsCore Friction Test");
    if (!renderer.IsValid()) return 1;

    Scene scene;
    scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
    scene.AddSystem(std::make_unique<PhysicsSystem>(scene));
    auto colSys = std::make_unique<CollisionSystem>(scene);
    scene.AddSystem(std::make_unique<ResolutionSystem>(scene, *colSys));
    scene.AddSystem(std::move(colSys));

    Entity ground = scene.CreateEntity();
    TransformComponent gtc;
    gtc.position = { 0.0f, -0.5f, 0.0f };
    scene.AddComponent(ground, gtc);
    scene.AddComponent(ground, MakeStaticBody());
    ColliderComponent gcc;
    gcc.shape = ColliderShape::Box;
    gcc.size = { 20.0f, 0.5f, 5.0f };
    scene.AddComponent(ground, gcc);

    Entity box = scene.CreateEntity();
    TransformComponent btc;
    btc.position = { -10.0f, 0.5f, 0.0f };
    scene.AddComponent(box, btc);
    RigidBodyComponent rb = MakeDynamicBody(1.0f, ComputeBoxInertia(1.0f, { 0.5f, 0.5f, 0.5f }));
    rb.velocity = { 10.0f, 0.0f, 0.0f }; // Fast slide
    scene.AddComponent(box, rb);
    ColliderComponent bcc;
    bcc.shape = ColliderShape::Box;
    bcc.size = { 0.5f, 0.5f, 0.5f };
    scene.AddComponent(box, bcc);

    auto lastTime = std::chrono::steady_clock::now();
    double physicsAccum = 0.0;

    // Draw theoretical stopping line
    float v0_vis = 10.0f;
    float dist = (v0_vis * v0_vis) / (2.0f * muk * gravity);
    float stopX = -10.0f + dist;

    while (!renderer.ShouldClose())
    {
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - lastTime).count();
        lastTime = now;
        if (dt > 0.1) dt = 0.1;

        physicsAccum += dt;
        while (physicsAccum >= fixedDt)
        {
            scene.SetFixedDeltaTime(fixedDt);
            scene.FixedUpdateSystems();
            physicsAccum -= fixedDt;
        }

        DebugDraw::Box({ 0.0f, -0.5f, 0.0f }, { 20.0f, 0.5f, 5.0f }, { 0.3f, 0.3f, 0.3f });

        auto* tc = scene.GetComponent<TransformComponent>(box);
        DebugDraw::Box(tc->position, { 0.5f, 0.5f, 0.5f }, { 1.0f, 0.5f, 0.0f });

        // Stopping line marker
        DebugDraw::Line({ stopX, 0.0f, -5.0f }, { stopX, 0.0f, 5.0f }, { 0.0f, 1.0f, 0.0f });
        DebugDraw::Sphere({ stopX, 0.0f, 0.0f }, 0.2f, { 0.0f, 1.0f, 0.0f }, 8, false);

        renderer.BeginFrame();
        renderer.FlushDebugDraw();
        renderer.EndFrame();
    }

    return 0;
}