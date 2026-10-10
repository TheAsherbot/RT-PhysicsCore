/**
 * @file FrictionTest.cpp
 * @brief Validates Coulomb friction deceleration, material profiles, and incline slip thresholds.
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
#include "RT-PhysicsCore/physics/systems/PhysicsSystem.h"
#include "RT-PhysicsCore/physics/systems/CollisionSystem.h"
#include "RT-PhysicsCore/physics/systems/ResolutionSystem.h"
#include "RT-PhysicsCore/physics/components/RigidBodyComponent.h"
#include "RT-PhysicsCore/physics/components/ColliderComponent.h"
#include "RT-PhysicsCore/physics/components/PhysicsMaterialComponent.h"
#include "RT-PhysicsCore/physics/MassProperties.h"
#include "RT-PhysicsCore/physics/PhysicsMaterial.h"

#include <glm/gtc/quaternion.hpp>

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

    int passCount = 0;
    int totalCount = 0;
    constexpr double fixedDt = 1.0 / 60.0;
    constexpr float gravity = 9.80665f;

    // ── Check 1: Sliding Deceleration on Flat Plane (Default Material) ──
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
        gcc.size = { 30.0f, 0.5f, 10.0f };
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

        float muk = GetMaterialProperties(MaterialId::Default).kineticFriction;
        float expectedDistance = (v0 * v0) / (2.0f * muk * gravity);
        float expectedFinalX = -10.0f + expectedDistance;

        for (int i = 0; i < 300; ++i)
        {
            StepScene(scene, 1, fixedDt);
            auto* bRb = scene.GetComponent<RigidBodyComponent>(box);
            if (bRb->velocity.x <= 0.01f)
            {
                break;
            }
        }

        auto* finalTc = scene.GetComponent<TransformComponent>(box);
        float err = std::abs(finalTc->position.x - expectedFinalX);
        float relativeErr = err / expectedDistance;
        bool pass = relativeErr < 0.08f;
        if (pass)
        {
            ++passCount;
        }

        std::ostringstream oss;
        oss << "FinalX=" << finalTc->position.x << " Expected=" << expectedFinalX
            << " relErr=" << (relativeErr * 100.0f) << "%";
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "Sliding Deceleration (Default) - " << oss.str());
    }

    // ── Check 2: Sliding Deceleration with Wood Material ──
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
        gcc.size = { 30.0f, 0.5f, 10.0f };
        scene.AddComponent(ground, gcc);
        PhysicsMaterialComponent gpm;
        gpm.material = MaterialId::Wood;
        scene.AddComponent(ground, gpm);

        Entity box = scene.CreateEntity();
        TransformComponent btc;
        btc.position = { -10.0f, 0.5f, 0.0f };
        scene.AddComponent(box, btc);

        float v0 = 4.0f;
        RigidBodyComponent rb = MakeDynamicBody(1.0f, ComputeBoxInertia(1.0f, { 0.5f, 0.5f, 0.5f }));
        rb.velocity = { v0, 0.0f, 0.0f };
        scene.AddComponent(box, rb);

        ColliderComponent bcc;
        bcc.shape = ColliderShape::Box;
        bcc.size = { 0.5f, 0.5f, 0.5f };
        scene.AddComponent(box, bcc);
        PhysicsMaterialComponent bpm;
        bpm.material = MaterialId::Wood;
        scene.AddComponent(box, bpm);

        float muk = GetMaterialProperties(MaterialId::Wood).kineticFriction;
        float expectedDistance = (v0 * v0) / (2.0f * muk * gravity);
        float expectedFinalX = -10.0f + expectedDistance;

        for (int i = 0; i < 300; ++i)
        {
            StepScene(scene, 1, fixedDt);
            auto* bRb = scene.GetComponent<RigidBodyComponent>(box);
            if (bRb->velocity.x <= 0.01f)
            {
                break;
            }
        }

        auto* finalTc = scene.GetComponent<TransformComponent>(box);
        float err = std::abs(finalTc->position.x - expectedFinalX);
        float relativeErr = err / expectedDistance;
        bool pass = relativeErr < 0.08f;
        if (pass)
        {
            ++passCount;
        }

        std::ostringstream oss;
        oss << "FinalX=" << finalTc->position.x << " Expected=" << expectedFinalX
            << " relErr=" << (relativeErr * 100.0f) << "%";
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "Sliding Deceleration (Wood) - " << oss.str());
    }

    // ── Check 3: Incline Static Friction Hold Below Critical Angle ──
    {
        ++totalCount;
        Scene scene;
        scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
        scene.AddSystem(std::make_unique<PhysicsSystem>(scene));
        auto colSys = std::make_unique<CollisionSystem>(scene);
        CollisionSystem* colSysPtr = colSys.get();
        scene.AddSystem(std::move(colSys));
        scene.AddSystem(std::make_unique<ResolutionSystem>(scene, *colSysPtr));

        float angleRad = glm::radians(15.0f);

        Entity ramp = scene.CreateEntity();
        TransformComponent rtc;
        rtc.position = { 0.0f, 0.0f, 0.0f };
        rtc.rotation = glm::angleAxis(angleRad, glm::vec3(0, 0, 1));
        scene.AddComponent(ramp, rtc);
        scene.AddComponent(ramp, MakeStaticBody());
        ColliderComponent rcc;
        rcc.shape = ColliderShape::Box;
        rcc.size = { 10.0f, 0.5f, 5.0f };
        scene.AddComponent(ramp, rcc);

        Entity box = scene.CreateEntity();
        TransformComponent btc;
        btc.position = { -2.0f * std::cos(angleRad), -2.0f * std::sin(angleRad) + 1.0f, 0.0f };
        btc.rotation = rtc.rotation;
        scene.AddComponent(box, btc);
        scene.AddComponent(box, MakeDynamicBody(1.0f, ComputeBoxInertia(1.0f, { 0.3f, 0.3f, 0.3f })));
        ColliderComponent bcc;
        bcc.shape = ColliderShape::Box;
        bcc.size = { 0.3f, 0.3f, 0.3f };
        scene.AddComponent(box, bcc);

        StepScene(scene, 120, fixedDt);

        auto* rb = scene.GetComponent<RigidBodyComponent>(box);
        float speed = glm::length(rb->velocity);
        bool pass = speed < 0.05f;
        if (pass)
        {
            ++passCount;
        }

        std::ostringstream oss;
        oss << "speed=" << speed << " (expected < 0.05 m/s)";
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "Static Incline Hold (15 deg < 31 deg) - " << oss.str());
    }

    // ── Check 4: Incline Static Friction Break Above Critical Angle ──
    {
        ++totalCount;
        Scene scene;
        scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
        scene.AddSystem(std::make_unique<PhysicsSystem>(scene));
        auto colSys = std::make_unique<CollisionSystem>(scene);
        CollisionSystem* colSysPtr = colSys.get();
        scene.AddSystem(std::move(colSys));
        scene.AddSystem(std::make_unique<ResolutionSystem>(scene, *colSysPtr));

        float angleRad = glm::radians(45.0f);

        Entity ramp = scene.CreateEntity();
        TransformComponent rtc;
        rtc.position = { 0.0f, 0.0f, 0.0f };
        rtc.rotation = glm::angleAxis(angleRad, glm::vec3(0, 0, 1));
        scene.AddComponent(ramp, rtc);
        scene.AddComponent(ramp, MakeStaticBody());
        ColliderComponent rcc;
        rcc.shape = ColliderShape::Box;
        rcc.size = { 10.0f, 0.5f, 5.0f };
        scene.AddComponent(ramp, rcc);

        Entity box = scene.CreateEntity();
        TransformComponent btc;
        btc.position = { -2.0f * std::cos(angleRad), -2.0f * std::sin(angleRad) + 1.0f, 0.0f };
        btc.rotation = rtc.rotation;
        scene.AddComponent(box, btc);
        scene.AddComponent(box, MakeDynamicBody(1.0f, ComputeBoxInertia(1.0f, { 0.3f, 0.3f, 0.3f })));
        ColliderComponent bcc;
        bcc.shape = ColliderShape::Box;
        bcc.size = { 0.3f, 0.3f, 0.3f };
        scene.AddComponent(box, bcc);

        StepScene(scene, 60, fixedDt);

        auto* rb = scene.GetComponent<RigidBodyComponent>(box);
        float speed = glm::length(rb->velocity);
        bool pass = speed > 0.5f;
        if (pass)
        {
            ++passCount;
        }

        std::ostringstream oss;
        oss << "speed=" << speed << " (expected > 0.5 m/s)";
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "Static Incline Slip (45 deg > 31 deg) - " << oss.str());
    }

    RT_LOG_INFO(passCount << "/" << totalCount << " automated checks passed.");

    // ── Phase 2: Interactive multi-scenario visualization ──

    Renderer renderer(1280, 720, "RT-PhysicsCore Friction Test");
    if (!renderer.IsValid())
    {
        return 1;
    }

    Scene scene;
    scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
    scene.AddSystem(std::make_unique<PhysicsSystem>(scene));
    auto colSys = std::make_unique<CollisionSystem>(scene);
    CollisionSystem* colSysPtr = colSys.get();
    scene.AddSystem(std::move(colSys));
    scene.AddSystem(std::make_unique<ResolutionSystem>(scene, *colSysPtr));

    int currentScenario = 1;
    constexpr int kTotalScenarios = 3;
    constexpr double kSecondsPerScenario = 6.0;
    double scenarioElapsed = 0.0;

    auto setupScenario = [&](int scenario)
        {
            currentScenario = scenario;
            scenarioElapsed = 0.0;

            std::vector<Entity> toDestroy = scene.GetEntities();
            for (Entity e : toDestroy)
            {
                scene.DestroyEntity(e);
            }

            if (scenario == 1)
            {
                RT_LOG_INFO("Showing Scenario 1: Multi-Material Sliding Race");
                Entity ground = scene.CreateEntity();
                TransformComponent gtc;
                gtc.position = { 0.0f, -0.5f, 0.0f };
                scene.AddComponent(ground, gtc);
                scene.AddComponent(ground, MakeStaticBody());
                ColliderComponent gcc;
                gcc.shape = ColliderShape::Box;
                gcc.size = { 25.0f, 0.5f, 10.0f };
                scene.AddComponent(ground, gcc);

                struct Racer { MaterialId mat; float z; };
                std::vector<Racer> racers = {
                    { MaterialId::BouncyIce, -3.0f },
                    { MaterialId::Wood, 0.0f },
                    { MaterialId::Rubber, 3.0f }
                };

                for (const auto& r : racers)
                {
                    Entity b = scene.CreateEntity();
                    TransformComponent tc;
                    tc.position = { -10.0f, 0.5f, r.z };
                    scene.AddComponent(b, tc);
                    RigidBodyComponent rb = MakeDynamicBody(1.0f, ComputeBoxInertia(1.0f, { 0.4f, 0.4f, 0.4f }));
                    rb.velocity = { 6.0f, 0.0f, 0.0f };
                    scene.AddComponent(b, rb);
                    ColliderComponent cc;
                    cc.shape = ColliderShape::Box;
                    cc.size = { 0.4f, 0.4f, 0.4f };
                    scene.AddComponent(b, cc);
                    PhysicsMaterialComponent pm;
                    pm.material = r.mat;
                    scene.AddComponent(b, pm);
                }
            }
            else if (scenario == 2)
            {
                RT_LOG_INFO("Showing Scenario 2: Dual Incline Critical Angles (15 deg vs 45 deg)");
                for (int i = 0; i < 2; ++i)
                {
                    float angle = (i == 0) ? 15.0f : 45.0f;
                    float angleRad = glm::radians(angle);
                    float z = (i == 0) ? -3.0f : 3.0f;

                    Entity ramp = scene.CreateEntity();
                    TransformComponent rtc;
                    rtc.position = { 0.0f, 2.0f, z };
                    rtc.rotation = glm::angleAxis(angleRad, glm::vec3(0, 0, 1));
                    scene.AddComponent(ramp, rtc);
                    scene.AddComponent(ramp, MakeStaticBody());
                    ColliderComponent rcc;
                    rcc.shape = ColliderShape::Box;
                    rcc.size = { 6.0f, 0.3f, 2.0f };
                    scene.AddComponent(ramp, rcc);

                    Entity box = scene.CreateEntity();
                    TransformComponent btc;
                    btc.position = rtc.position + glm::vec3(-2.0f * std::cos(angleRad), -2.0f * std::sin(angleRad) + 0.8f, 0.0f);
                    btc.rotation = rtc.rotation;
                    scene.AddComponent(box, btc);
                    scene.AddComponent(box, MakeDynamicBody(1.0f, ComputeBoxInertia(1.0f, { 0.4f, 0.4f, 0.4f })));
                    ColliderComponent bcc;
                    bcc.shape = ColliderShape::Box;
                    bcc.size = { 0.4f, 0.4f, 0.4f };
                    scene.AddComponent(box, bcc);
                }
            }
            else
            {
                RT_LOG_INFO("Showing Scenario 3: Heavy (10kg) vs Light (1kg) Sliding Distance");
                Entity ground = scene.CreateEntity();
                TransformComponent gtc;
                gtc.position = { 0.0f, -0.5f, 0.0f };
                scene.AddComponent(ground, gtc);
                scene.AddComponent(ground, MakeStaticBody());
                ColliderComponent gcc;
                gcc.shape = ColliderShape::Box;
                gcc.size = { 25.0f, 0.5f, 10.0f };
                scene.AddComponent(ground, gcc);

                // Light (1 kg)
                Entity b1 = scene.CreateEntity();
                TransformComponent tc1;
                tc1.position = { -10.0f, 0.5f, -2.5f };
                scene.AddComponent(b1, tc1);
                RigidBodyComponent rb1 = MakeDynamicBody(1.0f, ComputeBoxInertia(1.0f, { 0.4f, 0.4f, 0.4f }));
                rb1.velocity = { 5.0f, 0.0f, 0.0f };
                scene.AddComponent(b1, rb1);
                ColliderComponent cc1;
                cc1.shape = ColliderShape::Box;
                cc1.size = { 0.4f, 0.4f, 0.4f };
                scene.AddComponent(b1, cc1);

                // Heavy (10 kg)
                Entity b2 = scene.CreateEntity();
                TransformComponent tc2;
                tc2.position = { -10.0f, 0.5f, 2.5f };
                scene.AddComponent(b2, tc2);
                RigidBodyComponent rb2 = MakeDynamicBody(10.0f, ComputeBoxInertia(10.0f, { 0.4f, 0.4f, 0.4f }));
                rb2.velocity = { 5.0f, 0.0f, 0.0f };
                scene.AddComponent(b2, rb2);
                ColliderComponent cc2;
                cc2.shape = ColliderShape::Box;
                cc2.size = { 0.4f, 0.4f, 0.4f };
                scene.AddComponent(b2, cc2);
            }
        };

    setupScenario(1);

    auto lastTime = std::chrono::steady_clock::now();
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

        scenarioElapsed += dt;
        if (scenarioElapsed >= kSecondsPerScenario)
        {
            int nextScenario = (currentScenario % kTotalScenarios) + 1;
            setupScenario(nextScenario);
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

            glm::mat3 rot = glm::mat3_cast(tc->rotation);
            glm::vec3 color = (rb && rb->invMass > 0.0f) ? glm::vec3(0.2f, 0.8f, 0.3f) : glm::vec3(0.3f, 0.3f, 0.3f);

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

            if (rb && glm::length(rb->velocity) > 0.05f)
            {
                DebugDraw::Line(tc->position, tc->position + rb->velocity * 0.4f, { 1.0f, 1.0f, 0.0f });
            }
        }

        renderer.FlushDebugDraw();
        renderer.EndFrame();
    }

    return 0;
}