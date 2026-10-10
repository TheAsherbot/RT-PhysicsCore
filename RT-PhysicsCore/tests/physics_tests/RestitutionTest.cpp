/**
 * @file RestitutionTest.cpp
 * @brief Validates bounce mechanics, material restitution profiles, and 1D momentum transfer.
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
    RT_LOG_INFO("Restitution Test harness starting.");

    int passCount = 0;
    int totalCount = 0;
    constexpr double fixedDt = 1.0 / 60.0;

    // ── Check 1: Drop Restitution Velocity Ratio (Default e=0.3) ──
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
        float radius = 0.5f;
        btc.position = { 0.0f, 2.0f, 0.0f };
        scene.AddComponent(ball, btc);
        scene.AddComponent(ball, MakeDynamicBody(1.0f, ComputeSphereInertia(1.0f, radius)));
        ColliderComponent bcc;
        bcc.shape = ColliderShape::Sphere;
        bcc.size = { radius, 0.0f, 0.0f };
        scene.AddComponent(ball, bcc);

        float preImpactSpeed = 0.0f;
        float postImpactSpeed = 0.0f;

        for (int i = 0; i < 60; ++i)
        {
            auto* rb = scene.GetComponent<RigidBodyComponent>(ball);
            if (rb->velocity.y < -0.1f)
            {
                preImpactSpeed = std::abs(rb->velocity.y);
            }

            StepScene(scene, 1, fixedDt);

            if (preImpactSpeed > 0.0f && rb->velocity.y > 0.0f && postImpactSpeed == 0.0f)
            {
                postImpactSpeed = rb->velocity.y;
                break;
            }
        }

        float measuredRatio = (preImpactSpeed > 0.0f) ? (postImpactSpeed / preImpactSpeed) : 0.0f;
        float targetE = GetMaterialProperties(MaterialId::Default).restitution;
        float err = std::abs(measuredRatio - targetE);
        bool pass = err < 0.05f;
        if (pass)
        {
            ++passCount;
        }

        std::ostringstream oss;
        oss << "vRatio=" << measuredRatio << " Expected=" << targetE << " err=" << err;
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "Drop Rebound Ratio (Default e=0.3) - " << oss.str());
    }

    // ── Check 2: Clay Material Dead Landing (e=0.0) ──
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
        float radius = 0.5f;
        btc.position = { 0.0f, 2.0f, 0.0f };
        scene.AddComponent(ball, btc);
        scene.AddComponent(ball, MakeDynamicBody(1.0f, ComputeSphereInertia(1.0f, radius)));
        ColliderComponent bcc;
        bcc.shape = ColliderShape::Sphere;
        bcc.size = { radius, 0.0f, 0.0f };
        scene.AddComponent(ball, bcc);
        PhysicsMaterialComponent bpm;
        bpm.material = MaterialId::Clay;
        scene.AddComponent(ball, bpm);

        StepScene(scene, 60, fixedDt);

        auto* rb = scene.GetComponent<RigidBodyComponent>(ball);
        auto* tc = scene.GetComponent<TransformComponent>(ball);
        bool pass = std::abs(rb->velocity.y) < 0.1f && tc->position.y <= (radius + 0.05f);
        if (pass)
        {
            ++passCount;
        }

        std::ostringstream oss;
        oss << "y=" << tc->position.y << " vy=" << rb->velocity.y << " (expected rest on floor)";
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "Clay Dead Impact (e=0.0) - " << oss.str());
    }

    // ── Check 3: SuperBall Elastic Rebound (e=1.0) ──
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
        PhysicsMaterialComponent gpm;
        gpm.material = MaterialId::SuperBall;
        scene.AddComponent(ground, gpm);

        Entity ball = scene.CreateEntity();
        TransformComponent btc;
        float radius = 0.5f;
        btc.position = { 0.0f, 2.0f, 0.0f };
        scene.AddComponent(ball, btc);
        scene.AddComponent(ball, MakeDynamicBody(1.0f, ComputeSphereInertia(1.0f, radius)));
        ColliderComponent bcc;
        bcc.shape = ColliderShape::Sphere;
        bcc.size = { radius, 0.0f, 0.0f };
        scene.AddComponent(ball, bcc);
        PhysicsMaterialComponent bpm;
        bpm.material = MaterialId::SuperBall;
        scene.AddComponent(ball, bpm);

        float preImpactSpeed = 0.0f;
        float postImpactSpeed = 0.0f;

        for (int i = 0; i < 60; ++i)
        {
            auto* rb = scene.GetComponent<RigidBodyComponent>(ball);
            if (rb->velocity.y < -0.1f)
            {
                preImpactSpeed = std::abs(rb->velocity.y);
            }

            StepScene(scene, 1, fixedDt);

            if (preImpactSpeed > 0.0f && rb->velocity.y > 0.0f && postImpactSpeed == 0.0f)
            {
                postImpactSpeed = rb->velocity.y;
                break;
            }
        }

        float measuredRatio = (preImpactSpeed > 0.0f) ? (postImpactSpeed / preImpactSpeed) : 0.0f;
        float err = std::abs(measuredRatio - 1.0f);
        bool pass = err < 0.05f;
        if (pass)
        {
            ++passCount;
        }

        std::ostringstream oss;
        oss << "vRatio=" << measuredRatio << " Expected=1.0 err=" << err;
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "SuperBall Rebound Ratio (e=1.0) - " << oss.str());
    }

    // ── Check 4: 1D Elastic Collision Velocity Exchange (e=1.0) ──
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
        tca.position = { -2.0f, 0.0f, 0.0f };
        scene.AddComponent(a, tca);
        RigidBodyComponent rba = MakeDynamicBody(1.0f, ComputeSphereInertia(1.0f, 0.5f));
        rba.velocity = { 5.0f, 0.0f, 0.0f };
        scene.AddComponent(a, rba);
        ColliderComponent cca;
        cca.shape = ColliderShape::Sphere;
        cca.size = { 0.5f, 0.0f, 0.0f };
        scene.AddComponent(a, cca);
        PhysicsMaterialComponent pma;
        pma.material = MaterialId::BouncyIce;
        scene.AddComponent(a, pma);

        Entity b = scene.CreateEntity();
        TransformComponent tcb;
        tcb.position = { 0.0f, 0.0f, 0.0f };
        scene.AddComponent(b, tcb);
        RigidBodyComponent rbb = MakeDynamicBody(1.0f, ComputeSphereInertia(1.0f, 0.5f));
        rbb.velocity = { 0.0f, 0.0f, 0.0f };
        scene.AddComponent(b, rbb);
        ColliderComponent ccb;
        ccb.shape = ColliderShape::Sphere;
        ccb.size = { 0.5f, 0.0f, 0.0f };
        scene.AddComponent(b, ccb);
        PhysicsMaterialComponent pmb;
        pmb.material = MaterialId::BouncyIce;
        scene.AddComponent(b, pmb);

        StepScene(scene, 30, fixedDt);

        auto* finalRba = scene.GetComponent<RigidBodyComponent>(a);
        auto* finalRbb = scene.GetComponent<RigidBodyComponent>(b);

        bool pass = std::abs(finalRba->velocity.x) < 0.2f && std::abs(finalRbb->velocity.x - 5.0f) < 0.2f;
        if (pass)
        {
            ++passCount;
        }

        std::ostringstream oss;
        oss << "vA=" << finalRba->velocity.x << " vB=" << finalRbb->velocity.x
            << " (expected vA~0, vB~5.0)";
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "1D Elastic Exchange (Two Spheres) - " << oss.str());
    }

    RT_LOG_INFO(passCount << "/" << totalCount << " automated checks passed.");

    // ── Phase 2: Interactive multi-scenario visualization ──

    Renderer renderer(1280, 720, "RT-PhysicsCore Restitution Test");
    if (!renderer.IsValid())
    {
        return 1;
    }

    Scene scene;
    scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
    auto phys = std::make_unique<PhysicsSystem>(scene);
    PhysicsSystem* physPtr = phys.get();
    scene.AddSystem(std::move(phys));
    auto colSys = std::make_unique<CollisionSystem>(scene);
    CollisionSystem* colSysPtr = colSys.get();
    scene.AddSystem(std::move(colSys));
    scene.AddSystem(std::make_unique<ResolutionSystem>(scene, *colSysPtr));

    int currentScenario = 1;
    constexpr int kTotalScenarios = 3;
    constexpr double kSecondsPerScenario = 6.0;
    double scenarioElapsed = 0.0;
    std::vector<glm::vec3> apexMarkers;

    auto setupScenario = [&](int scenario)
        {
            currentScenario = scenario;
            scenarioElapsed = 0.0;
            apexMarkers.clear();

            std::vector<Entity> toDestroy = scene.GetEntities();
            for (Entity e : toDestroy)
            {
                scene.DestroyEntity(e);
            }

            if (scenario == 1)
            {
                RT_LOG_INFO("Showing Scenario 1: 5-Ball Material Restitution Drop (0.0 to 1.0)");
                physPtr->SetGravity({ 0.0f, -9.80665f, 0.0f });

                Entity ground = scene.CreateEntity();
                TransformComponent gtc;
                gtc.position = { 0.0f, -0.5f, 0.0f };
                scene.AddComponent(ground, gtc);
                scene.AddComponent(ground, MakeStaticBody());
                ColliderComponent gcc;
                gcc.shape = ColliderShape::Box;
                gcc.size = { 15.0f, 0.5f, 5.0f };
                scene.AddComponent(ground, gcc);

                struct BallDef { MaterialId mat; float x; };
                std::vector<BallDef> defs = {
                    { MaterialId::Clay, -6.0f },
                    { MaterialId::Wood, -3.0f },
                    { MaterialId::Rubber, 0.0f },
                    { MaterialId::HardRubber, 3.0f },
                    { MaterialId::SuperBall, 6.0f }
                };

                for (const auto& d : defs)
                {
                    Entity b = scene.CreateEntity();
                    TransformComponent tc;
                    tc.position = { d.x, 8.0f, 0.0f };
                    scene.AddComponent(b, tc);
                    scene.AddComponent(b, MakeDynamicBody(1.0f, ComputeSphereInertia(1.0f, 0.4f)));
                    ColliderComponent cc;
                    cc.shape = ColliderShape::Sphere;
                    cc.size = { 0.4f, 0.0f, 0.0f };
                    scene.AddComponent(b, cc);
                    PhysicsMaterialComponent pm;
                    pm.material = d.mat;
                    scene.AddComponent(b, pm);
                }
            }
            else if (scenario == 2)
            {
                RT_LOG_INFO("Showing Scenario 2: Head-on Elastic 1D Momentum Transfer (Zero-G)");
                physPtr->SetGravity({ 0.0f, 0.0f, 0.0f });

                Entity a = scene.CreateEntity();
                TransformComponent tca;
                tca.position = { -5.0f, 0.0f, 0.0f };
                scene.AddComponent(a, tca);
                RigidBodyComponent rba = MakeDynamicBody(1.0f, ComputeSphereInertia(1.0f, 0.5f));
                rba.velocity = { 4.0f, 0.0f, 0.0f };
                scene.AddComponent(a, rba);
                ColliderComponent cca;
                cca.shape = ColliderShape::Sphere;
                cca.size = { 0.5f, 0.0f, 0.0f };
                scene.AddComponent(a, cca);
                PhysicsMaterialComponent pma;
                pma.material = MaterialId::BouncyIce;
                scene.AddComponent(a, pma);

                Entity b = scene.CreateEntity();
                TransformComponent tcb;
                tcb.position = { 0.0f, 0.0f, 0.0f };
                scene.AddComponent(b, tcb);
                RigidBodyComponent rbb = MakeDynamicBody(1.0f, ComputeSphereInertia(1.0f, 0.5f));
                rbb.velocity = { 0.0f, 0.0f, 0.0f };
                scene.AddComponent(b, rbb);
                ColliderComponent ccb;
                ccb.shape = ColliderShape::Sphere;
                ccb.size = { 0.5f, 0.0f, 0.0f };
                scene.AddComponent(b, ccb);
                PhysicsMaterialComponent pmb;
                pmb.material = MaterialId::BouncyIce;
                scene.AddComponent(b, pmb);
            }
            else
            {
                RT_LOG_INFO("Showing Scenario 3: Angled 45 deg Elastic Rebound Reflection");
                physPtr->SetGravity({ 0.0f, -9.80665f, 0.0f });

                Entity wedge = scene.CreateEntity();
                TransformComponent wtc;
                wtc.position = { 0.0f, 1.0f, 0.0f };
                wtc.rotation = glm::angleAxis(glm::radians(45.0f), glm::vec3(0, 0, 1));
                scene.AddComponent(wedge, wtc);
                scene.AddComponent(wedge, MakeStaticBody());
                ColliderComponent wcc;
                wcc.shape = ColliderShape::Box;
                wcc.size = { 3.0f, 0.3f, 3.0f };
                scene.AddComponent(wedge, wcc);
                PhysicsMaterialComponent wpm;
                wpm.material = MaterialId::SuperBall;
                scene.AddComponent(wedge, wpm);

                Entity ball = scene.CreateEntity();
                TransformComponent btc;
                btc.position = { 0.0f, 8.0f, 0.0f };
                scene.AddComponent(ball, btc);
                scene.AddComponent(ball, MakeDynamicBody(1.0f, ComputeSphereInertia(1.0f, 0.4f)));
                ColliderComponent bcc;
                bcc.shape = ColliderShape::Sphere;
                bcc.size = { 0.4f, 0.0f, 0.0f };
                scene.AddComponent(ball, bcc);
                PhysicsMaterialComponent bpm;
                bpm.material = MaterialId::SuperBall;
                scene.AddComponent(ball, bpm);
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

            if (currentScenario == 1)
            {
                for (Entity e : scene.Query<TransformComponent, RigidBodyComponent>())
                {
                    auto* tc = scene.GetComponent<TransformComponent>(e);
                    auto* rb = scene.GetComponent<RigidBodyComponent>(e);
                    if (rb->invMass > 0.0f && rb->velocity.y <= 0.0f && rb->velocity.y > -0.1f && tc->position.y > 1.0f)
                    {
                        bool isDuplicate = false;
                        for (const auto& marker : apexMarkers)
                        {
                            if (glm::distance(marker, tc->position) < 0.3f)
                            {
                                isDuplicate = true;
                                break;
                            }
                        }
                        if (!isDuplicate)
                        {
                            apexMarkers.push_back(tc->position);
                        }
                    }
                }
            }

            physicsAccum -= fixedDt;
        }

        renderer.BeginFrame();

        for (Entity e : scene.Query<TransformComponent, ColliderComponent>())
        {
            auto* tc = scene.GetComponent<TransformComponent>(e);
            auto* cc = scene.GetComponent<ColliderComponent>(e);
            auto* rb = scene.GetComponent<RigidBodyComponent>(e);

            glm::vec3 color = (rb && rb->invMass > 0.0f) ? glm::vec3(0.2f, 0.7f, 1.0f) : glm::vec3(0.35f, 0.35f, 0.35f);

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

        for (const auto& marker : apexMarkers)
        {
            DebugDraw::Sphere(marker, 0.08f, { 1.0f, 1.0f, 1.0f }, 8, false);
        }

        renderer.FlushDebugDraw();
        renderer.EndFrame();
    }

    return 0;
}