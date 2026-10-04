/**
 * @file RestitutionTest.cpp
 * @brief Validates bounce mechanics (coefficient of restitution).
 *
 * Drops rigid bodies from known heights and verifies that their rebound height
 * matches the theoretical value: h_bounce = h_drop * e^2.
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
    RT_LOG_INFO("Restitution Test harness starting.");

    // Default material has e=0.3. For self-collision, e_eff = (0.3 + 0.3)/2 = 0.3.
    MaterialProperties mat = GetMaterialProperties(MaterialId::Default);
    float e = mat.restitution;

    int passCount = 0;
    int totalCount = 0;
    constexpr double fixedDt = 1.0 / 60.0;

    {
        ++totalCount;
        Scene scene;
        scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
        scene.AddSystem(std::make_unique<PhysicsSystem>(scene));
        auto colSys = std::make_unique<CollisionSystem>(scene);
        CollisionSystem* colSysPtr = colSys.get();
        scene.AddSystem(std::move(colSys));
        scene.AddSystem(std::make_unique<ResolutionSystem>(scene, *colSysPtr));

        Entity ground = scene.CreateEntity();
        TransformComponent gtc;
        gtc.position = { 0.0f, -0.5f, 0.0f };
        scene.AddComponent(ground, gtc);
        scene.AddComponent(ground, MakeStaticBody());
        ColliderComponent gcc;
        gcc.shape = ColliderShape::Box;
        gcc.size = { 10.0f, 0.5f, 10.0f };
        scene.AddComponent(ground, gcc);

        Entity ball = scene.CreateEntity();
        TransformComponent btc;
        float startHeight = 10.0f;
        float radius = 0.5f;
        btc.position = { 0.0f, startHeight, 0.0f };
        scene.AddComponent(ball, btc);
        scene.AddComponent(ball, MakeDynamicBody(1.0f, ComputeSphereInertia(1.0f, radius)));
        ColliderComponent bcc;
        bcc.shape = ColliderShape::Sphere;
        bcc.size = { radius, 0.0f, 0.0f };
        scene.AddComponent(ball, bcc);

        float expectedBounceHeight = (startHeight - radius) * (e * e) + radius;

        float maxObservedY = 0.0f;
        bool bounced = false;

        // Run for 3 seconds (180 ticks) to capture the first bounce peak
        for (int i = 0; i < 180; ++i)
        {
            StepScene(scene, 1, fixedDt);
            auto* tc = scene.GetComponent<TransformComponent>(ball);
            auto* rb = scene.GetComponent<RigidBodyComponent>(ball);

            if (rb->velocity.y > 0.0f) bounced = true;
            if (bounced && rb->velocity.y <= 0.0f && maxObservedY == 0.0f)
            {
                // We reached the apex
                maxObservedY = tc->position.y;
            }
        }

        float err = std::abs(maxObservedY - expectedBounceHeight);
        bool pass = err < 0.15f; // 15cm tolerance
        if (pass) ++passCount;

        std::ostringstream oss;
        oss << "MaxBounceY=" << maxObservedY << " Expected=" << expectedBounceHeight << " err=" << err;
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "Drop Restitution Check - " << oss.str());
    }

    RT_LOG_INFO(passCount << "/" << totalCount << " automated checks passed.");

    // ── Phase 2: Interactive visualization ──

    Renderer renderer(1280, 720, "RT-PhysicsCore Restitution Test");
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
    gcc.size = { 10.0f, 0.5f, 10.0f };
    scene.AddComponent(ground, gcc);

    Entity ball = scene.CreateEntity();
    TransformComponent btc;
    btc.position = { 0.0f, 10.0f, 0.0f };
    scene.AddComponent(ball, btc);
    scene.AddComponent(ball, MakeDynamicBody(1.0f, ComputeSphereInertia(1.0f, 0.5f)));
    ColliderComponent bcc;
    bcc.shape = ColliderShape::Sphere;
    bcc.size = { 0.5f, 0.0f, 0.0f };
    scene.AddComponent(ball, bcc);

    auto lastTime = std::chrono::steady_clock::now();
    double physicsAccum = 0.0;
    std::vector<glm::vec3> trail;

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

            auto* tc = scene.GetComponent<TransformComponent>(ball);
            if (trail.empty() || glm::length(trail.back() - tc->position) > 0.1f)
            {
                trail.push_back(tc->position);
                if (trail.size() > 500) trail.erase(trail.begin());
            }
        }

        // Draw ground
        DebugDraw::Box({ 0.0f, -0.5f, 0.0f }, { 10.0f, 0.5f, 10.0f }, { 0.3f, 0.3f, 0.3f });

        // Draw ball
        auto* tc = scene.GetComponent<TransformComponent>(ball);
        DebugDraw::Sphere(tc->position, 0.5f, { 0.2f, 0.8f, 1.0f });

        // Draw trail
        for (size_t i = 1; i < trail.size(); ++i)
        {
            DebugDraw::Line(trail[i - 1], trail[i], { 1.0f, 1.0f, 0.0f });
        }

        renderer.BeginFrame();
        renderer.FlushDebugDraw();
        renderer.EndFrame();
    }

    return 0;
}