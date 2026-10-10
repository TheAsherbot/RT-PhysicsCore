/**
 * @file PositionCorrectionTest.cpp
 * @brief Validates split-impulse position correction across spheres, boxes, rotated corners, and stacks.
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
#include "RT-PhysicsCore/physics/MassProperties.h"

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
    RT_LOG_INFO("Position Correction Test harness starting.");

    int passCount = 0;
    int totalCount = 0;
    constexpr double fixedDt = 1.0 / 60.0;

    // ── Check 1: Deeply Overlapping Spheres (0.8m overlap) ──
    {
        ++totalCount;
        Scene scene;
        scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
        auto phys = std::make_unique<PhysicsSystem>(scene);
        phys->SetGravity({ 0.0f, 0.0f, 0.0f });
        scene.AddSystem(std::move(phys));

        auto colSys = std::make_unique<CollisionSystem>(scene);
        CollisionSystem* colSysPtr = colSys.get();
        scene.AddSystem(std::move(colSys));
        scene.AddSystem(std::make_unique<ResolutionSystem>(scene, *colSysPtr));

        float radius = 1.0f;
        Entity a = scene.CreateEntity();
        TransformComponent tca;
        tca.position = { -0.2f, 0.0f, 0.0f };
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

        StepScene(scene, 15, fixedDt);

        auto* finalTca = scene.GetComponent<TransformComponent>(a);
        auto* finalTcb = scene.GetComponent<TransformComponent>(b);
        auto* rba = scene.GetComponent<RigidBodyComponent>(a);
        auto* rbb = scene.GetComponent<RigidBodyComponent>(b);

        float distance = glm::length(finalTca->position - finalTcb->position);
        float maxVel = std::max(glm::length(rba->velocity), glm::length(rbb->velocity));

        bool pass = distance >= 1.9f && maxVel < 0.5f;
        if (pass)
        {
            ++passCount;
        }

        std::ostringstream oss;
        oss << "Separation=" << distance << " (expected ~2.0), maxVel=" << maxVel << " (expected < 0.5)";
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "Overlap Injection Separation (Spheres) - " << oss.str());
    }

    // ── Check 2: Deeply Overlapping Boxes (0.6m overlap) ──
    {
        ++totalCount;
        Scene scene;
        scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
        auto phys = std::make_unique<PhysicsSystem>(scene);
        phys->SetGravity({ 0.0f, 0.0f, 0.0f });
        scene.AddSystem(std::move(phys));

        auto colSys = std::make_unique<CollisionSystem>(scene);
        CollisionSystem* colSysPtr = colSys.get();
        scene.AddSystem(std::move(colSys));
        scene.AddSystem(std::make_unique<ResolutionSystem>(scene, *colSysPtr));

        Entity a = scene.CreateEntity();
        TransformComponent tca;
        tca.position = { -0.2f, 0.0f, 0.0f };
        scene.AddComponent(a, tca);
        scene.AddComponent(a, MakeDynamicBody(1.0f, ComputeBoxInertia(1.0f, { 0.5f, 0.5f, 0.5f })));
        ColliderComponent cca;
        cca.shape = ColliderShape::Box;
        cca.size = { 0.5f, 0.5f, 0.5f };
        scene.AddComponent(a, cca);

        Entity b = scene.CreateEntity();
        TransformComponent tcb;
        tcb.position = { 0.2f, 0.0f, 0.0f };
        scene.AddComponent(b, tcb);
        scene.AddComponent(b, MakeDynamicBody(1.0f, ComputeBoxInertia(1.0f, { 0.5f, 0.5f, 0.5f })));
        ColliderComponent ccb;
        ccb.shape = ColliderShape::Box;
        ccb.size = { 0.5f, 0.5f, 0.5f };
        scene.AddComponent(b, ccb);

        StepScene(scene, 15, fixedDt);

        auto* finalTca = scene.GetComponent<TransformComponent>(a);
        auto* finalTcb = scene.GetComponent<TransformComponent>(b);
        auto* rba = scene.GetComponent<RigidBodyComponent>(a);
        auto* rbb = scene.GetComponent<RigidBodyComponent>(b);

        float distance = glm::length(finalTca->position - finalTcb->position);
        float maxVel = std::max(glm::length(rba->velocity), glm::length(rbb->velocity));

        bool pass = distance >= 0.95f && maxVel < 0.5f;
        if (pass)
        {
            ++passCount;
        }

        std::ostringstream oss;
        oss << "Separation=" << distance << " (expected ~1.0), maxVel=" << maxVel << " (expected < 0.5)";
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "Overlap Injection Separation (Boxes) - " << oss.str());
    }

    // ── Check 3: Rotated Box Submerged Corner into Static Ground ──
    {
        ++totalCount;
        Scene scene;
        scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
        auto phys = std::make_unique<PhysicsSystem>(scene);
        phys->SetGravity({ 0.0f, 0.0f, 0.0f });
        scene.AddSystem(std::move(phys));

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

        Entity box = scene.CreateEntity();
        TransformComponent btc;
        btc.rotation = glm::angleAxis(glm::radians(45.0f), glm::vec3(0, 0, 1));
        btc.position = { 0.0f, 0.5f, 0.0f };
        scene.AddComponent(box, btc);
        scene.AddComponent(box, MakeDynamicBody(1.0f, ComputeBoxInertia(1.0f, { 0.5f, 0.5f, 0.5f })));
        ColliderComponent bcc;
        bcc.shape = ColliderShape::Box;
        bcc.size = { 0.5f, 0.5f, 0.5f };
        scene.AddComponent(box, bcc);

        StepScene(scene, 20, fixedDt);

        auto* finalTc = scene.GetComponent<TransformComponent>(box);
        auto* rb = scene.GetComponent<RigidBodyComponent>(box);

        bool pass = finalTc->position.y >= 0.65f && glm::length(rb->velocity) < 0.5f;
        if (pass)
        {
            ++passCount;
        }

        std::ostringstream oss;
        oss << "FinalY=" << finalTc->position.y << " vel=" << glm::length(rb->velocity)
            << " (expected y >= 0.65, vel < 0.5)";
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "Rotated Corner Floor Penetration - " << oss.str());
    }

    // ── Check 4: 3-Box Overlap Stack Settling ──
    {
        ++totalCount;
        Scene scene;
        scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
        auto phys = std::make_unique<PhysicsSystem>(scene);
        phys->SetGravity({ 0.0f, -9.80665f, 0.0f });
        scene.AddSystem(std::move(phys));

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

        Entity b1 = scene.CreateEntity();
        TransformComponent tc1;
        tc1.position = { 0.0f, 0.45f, 0.0f };
        scene.AddComponent(b1, tc1);
        scene.AddComponent(b1, MakeDynamicBody(1.0f, ComputeBoxInertia(1.0f, { 0.5f, 0.5f, 0.5f })));
        ColliderComponent cc1;
        cc1.shape = ColliderShape::Box;
        cc1.size = { 0.5f, 0.5f, 0.5f };
        scene.AddComponent(b1, cc1);

        Entity b2 = scene.CreateEntity();
        TransformComponent tc2;
        tc2.position = { 0.0f, 1.35f, 0.0f };
        scene.AddComponent(b2, tc2);
        scene.AddComponent(b2, MakeDynamicBody(1.0f, ComputeBoxInertia(1.0f, { 0.5f, 0.5f, 0.5f })));
        ColliderComponent cc2;
        cc2.shape = ColliderShape::Box;
        cc2.size = { 0.5f, 0.5f, 0.5f };
        scene.AddComponent(b2, cc2);

        Entity b3 = scene.CreateEntity();
        TransformComponent tc3;
        tc3.position = { 0.0f, 2.25f, 0.0f };
        scene.AddComponent(b3, tc3);
        scene.AddComponent(b3, MakeDynamicBody(1.0f, ComputeBoxInertia(1.0f, { 0.5f, 0.5f, 0.5f })));
        ColliderComponent cc3;
        cc3.shape = ColliderShape::Box;
        cc3.size = { 0.5f, 0.5f, 0.5f };
        scene.AddComponent(b3, cc3);

        StepScene(scene, 60, fixedDt);

        auto* rb1 = scene.GetComponent<RigidBodyComponent>(b1);
        auto* rb2 = scene.GetComponent<RigidBodyComponent>(b2);
        auto* rb3 = scene.GetComponent<RigidBodyComponent>(b3);

        float totalKineticEnergy = 0.5f * (glm::dot(rb1->velocity, rb1->velocity)
            + glm::dot(rb2->velocity, rb2->velocity)
            + glm::dot(rb3->velocity, rb3->velocity));

        bool pass = totalKineticEnergy < 0.1f;
        if (pass)
        {
            ++passCount;
        }

        std::ostringstream oss;
        oss << "KE=" << totalKineticEnergy << " (expected < 0.1)";
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "3-Box Penetration Stack Settling - " << oss.str());
    }

    RT_LOG_INFO(passCount << "/" << totalCount << " automated checks passed.");

    // ── Phase 2: Interactive visualization ──

    Renderer renderer(1280, 720, "RT-PhysicsCore Position Correction Test");
    if (!renderer.IsValid())
    {
        return 1;
    }

    Scene scene;
    scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
    auto phys = std::make_unique<PhysicsSystem>(scene);
    phys->SetGravity({ 0.0f, 0.0f, 0.0f });
    scene.AddSystem(std::move(phys));
    auto colSys = std::make_unique<CollisionSystem>(scene);
    CollisionSystem* colSysPtr = colSys.get();
    scene.AddSystem(std::move(colSys));
    scene.AddSystem(std::make_unique<ResolutionSystem>(scene, *colSysPtr));

    int currentScenario = 1;
    constexpr int kTotalScenarios = 4;
    constexpr double kSecondsPerScenario = 5.0;
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
                RT_LOG_INFO("Showing Scenario 1: Overlapping Spheres (0.8m overlap)");
                float radius = 1.0f;
                Entity a = scene.CreateEntity();
                TransformComponent tca;
                tca.position = { -0.2f, 0.0f, 0.0f };
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
            }
            else if (scenario == 2)
            {
                RT_LOG_INFO("Showing Scenario 2: Overlapping Boxes (0.6m overlap)");
                Entity a = scene.CreateEntity();
                TransformComponent tca;
                tca.position = { -0.2f, 0.0f, 0.0f };
                scene.AddComponent(a, tca);
                scene.AddComponent(a, MakeDynamicBody(1.0f, ComputeBoxInertia(1.0f, { 0.6f, 0.6f, 0.6f })));
                ColliderComponent cca;
                cca.shape = ColliderShape::Box;
                cca.size = { 0.6f, 0.6f, 0.6f };
                scene.AddComponent(a, cca);

                Entity b = scene.CreateEntity();
                TransformComponent tcb;
                tcb.position = { 0.2f, 0.0f, 0.0f };
                scene.AddComponent(b, tcb);
                scene.AddComponent(b, MakeDynamicBody(1.0f, ComputeBoxInertia(1.0f, { 0.6f, 0.6f, 0.6f })));
                ColliderComponent ccb;
                ccb.shape = ColliderShape::Box;
                ccb.size = { 0.6f, 0.6f, 0.6f };
                scene.AddComponent(b, ccb);
            }
            else if (scenario == 3)
            {
                RT_LOG_INFO("Showing Scenario 3: Rotated Corner Penetrating Floor");
                Entity ground = scene.CreateEntity();
                TransformComponent gtc;
                gtc.position = { 0.0f, -0.5f, 0.0f };
                scene.AddComponent(ground, gtc);
                scene.AddComponent(ground, MakeStaticBody());
                ColliderComponent gcc;
                gcc.shape = ColliderShape::Box;
                gcc.size = { 6.0f, 0.5f, 6.0f };
                scene.AddComponent(ground, gcc);

                Entity box = scene.CreateEntity();
                TransformComponent btc;
                btc.rotation = glm::angleAxis(glm::radians(45.0f), glm::vec3(0, 0, 1));
                btc.position = { 0.0f, 0.4f, 0.0f };
                scene.AddComponent(box, btc);
                scene.AddComponent(box, MakeDynamicBody(1.0f, ComputeBoxInertia(1.0f, { 0.5f, 0.5f, 0.5f })));
                ColliderComponent bcc;
                bcc.shape = ColliderShape::Box;
                bcc.size = { 0.5f, 0.5f, 0.5f };
                scene.AddComponent(box, bcc);
            }
            else
            {
                RT_LOG_INFO("Showing Scenario 4: 3-Box Overlapping Stack");
                Entity ground = scene.CreateEntity();
                TransformComponent gtc;
                gtc.position = { 0.0f, -0.5f, 0.0f };
                scene.AddComponent(ground, gtc);
                scene.AddComponent(ground, MakeStaticBody());
                ColliderComponent gcc;
                gcc.shape = ColliderShape::Box;
                gcc.size = { 6.0f, 0.5f, 6.0f };
                scene.AddComponent(ground, gcc);

                for (int i = 0; i < 3; ++i)
                {
                    Entity b = scene.CreateEntity();
                    TransformComponent tc;
                    tc.position = { 0.0f, 0.4f + i * 0.9f, 0.0f };
                    scene.AddComponent(b, tc);
                    scene.AddComponent(b, MakeDynamicBody(1.0f, ComputeBoxInertia(1.0f, { 0.5f, 0.5f, 0.5f })));
                    ColliderComponent cc;
                    cc.shape = ColliderShape::Box;
                    cc.size = { 0.5f, 0.5f, 0.5f };
                    scene.AddComponent(b, cc);
                }
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

            glm::vec3 color = (rb && rb->invMass > 0.0f) ? glm::vec3(0.2f, 0.8f, 0.3f) : glm::vec3(0.4f, 0.4f, 0.4f);

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

        renderer.FlushDebugDraw();
        renderer.EndFrame();
    }

    return 0;
}