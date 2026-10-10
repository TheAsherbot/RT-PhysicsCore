/**
 * @file IntegrationTest.cpp
 * @brief Validates PhysicsSystem symplectic Euler integration against analytical solutions.
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
#include "RT-PhysicsCore/physics/components/RigidBodyComponent.h"
#include "RT-PhysicsCore/physics/components/ColliderComponent.h"
#include "RT-PhysicsCore/physics/MassProperties.h"

#include <glm/gtc/quaternion.hpp>

namespace
{
    using namespace RT_PhysicsCore;

    constexpr double kFixedDt = 1.0 / 60.0;
    constexpr float kGravity = 9.80665f;

    struct IntegrationScene
    {
        Scene scene;
        PhysicsSystem* physics = nullptr;
    };

    std::unique_ptr<IntegrationScene> CreateIntegrationScene()
    {
        auto is = std::make_unique<IntegrationScene>();
        is->scene.AddSystem(std::make_unique<TransformPropagationSystem>(is->scene));
        auto phys = std::make_unique<PhysicsSystem>(is->scene);
        is->physics = phys.get();
        is->scene.AddSystem(std::move(phys));
        return is;
    }

    void StepScene(Scene& scene, int ticks)
    {
        for (int i = 0; i < ticks; ++i)
        {
            scene.SetFixedDeltaTime(kFixedDt);
            scene.FixedUpdateSystems();
        }
    }

    // --- Test infrastructure ---

    struct TestResult
    {
        std::string name;
        bool passed;
        std::string detail;
    };

    std::vector<TestResult> results;

    void Assert(const std::string& name, bool condition, const std::string& detail)
    {
        results.push_back({ name, condition, detail });
    }

    // --- Visual data ---

    struct TrailPoint
    {
        glm::vec3 position;
        glm::vec3 color;
    };

    // --- Tests ---

    void RunFreeFallTest()
    {
        auto is = CreateIntegrationScene();
        Entity e = is->scene.CreateEntity();
        TransformComponent tc;
        tc.position = { 0.0f, 10.0f, 0.0f };
        is->scene.AddComponent(e, tc);
        RigidBodyComponent rb = MakeDynamicBody(1.0f, ComputeSphereInertia(1.0f, 0.5f));
        is->scene.AddComponent(e, rb);

        // 30 ticks = 0.5s
        StepScene(is->scene, 30);
        auto* t = is->scene.GetComponent<TransformComponent>(e);
        float expectedY = 10.0f - 0.5f * kGravity * 0.5f * 0.5f;
        float err = std::abs(t->position.y - expectedY);

        std::ostringstream oss;
        oss << "pos.y=" << t->position.y << " expected=" << expectedY << " err=" << err;
        Assert("Free-fall 0.5s", err < 0.15f, oss.str());
    }

    void RunFreeFallLongTest()
    {
        auto is = CreateIntegrationScene();
        Entity e = is->scene.CreateEntity();
        TransformComponent tc;
        tc.position = { 0.0f, 50.0f, 0.0f };
        is->scene.AddComponent(e, tc);
        is->scene.AddComponent(e, MakeDynamicBody(1.0f, ComputeSphereInertia(1.0f, 0.5f)));

        // 60 ticks = 1.0s
        StepScene(is->scene, 60);
        auto* t = is->scene.GetComponent<TransformComponent>(e);
        float expectedY = 50.0f - 0.5f * kGravity * 1.0f * 1.0f;
        float err = std::abs(t->position.y - expectedY);

        std::ostringstream oss;
        oss << "pos.y=" << t->position.y << " expected=" << expectedY << " err=" << err;
        Assert("Free-fall 1.0s", err < 0.25f, oss.str());
    }

    void RunProjectileTest()
    {
        auto is = CreateIntegrationScene();
        Entity e = is->scene.CreateEntity();
        TransformComponent tc;
        tc.position = { 0.0f, 5.0f, 0.0f };
        is->scene.AddComponent(e, tc);
        RigidBodyComponent rb = MakeDynamicBody(1.0f, ComputeSphereInertia(1.0f, 0.5f));
        rb.velocity = { 10.0f, 0.0f, 0.0f };
        is->scene.AddComponent(e, rb);

        // 60 ticks = 1.0s
        StepScene(is->scene, 60);
        auto* t = is->scene.GetComponent<TransformComponent>(e);
        float expectedX = 10.0f;
        float expectedY = 5.0f - 0.5f * kGravity;
        float errX = std::abs(t->position.x - expectedX);
        float errY = std::abs(t->position.y - expectedY);

        std::ostringstream oss;
        oss << "pos=[" << t->position.x << "," << t->position.y << "] expected=["
            << expectedX << "," << expectedY << "] err=[" << errX << "," << errY << "]";
        Assert("Projectile (horizontal throw)", errX < 0.2f && errY < 0.25f, oss.str());
    }

    void RunStaticBodyIgnoresForceTest()
    {
        auto is = CreateIntegrationScene();
        Entity e = is->scene.CreateEntity();
        TransformComponent tc;
        tc.position = { 0.0f, 0.0f, 0.0f };
        is->scene.AddComponent(e, tc);
        RigidBodyComponent rb = MakeStaticBody();
        rb.forceAccum = { 1000.0f, 1000.0f, 1000.0f };
        is->scene.AddComponent(e, rb);

        StepScene(is->scene, 120);
        auto* t = is->scene.GetComponent<TransformComponent>(e);
        float displacement = glm::length(t->position);

        std::ostringstream oss;
        oss << "displacement=" << displacement;
        Assert("Static body ignores forces", displacement < 0.001f, oss.str());
    }

    void RunAngularMomentumConservationTest()
    {
        auto is = CreateIntegrationScene();
        is->physics->SetGravity({ 0.0f, 0.0f, 0.0f });

        Entity e = is->scene.CreateEntity();
        TransformComponent tc;
        is->scene.AddComponent(e, tc);
        // Sphere: isotropic inertia, L should stay constant
        RigidBodyComponent rb = MakeDynamicBody(1.0f, ComputeSphereInertia(1.0f, 1.0f));
        rb.angularMomentum = { 0.0f, 2.0f, 0.0f };
        is->scene.AddComponent(e, rb);

        StepScene(is->scene, 600);
        auto* body = is->scene.GetComponent<RigidBodyComponent>(e);
        float magL = glm::length(body->angularMomentum);
        float drift = std::abs(magL - 2.0f);

        std::ostringstream oss;
        oss << "|L|=" << magL << " initial=2.0 drift=" << drift;
        Assert("Angular momentum conservation (sphere)", drift < 0.01f, oss.str());
    }

    void RunAsymmetricTumblingTest()
    {
        auto is = CreateIntegrationScene();
        is->physics->SetGravity({ 0.0f, 0.0f, 0.0f });

        Entity e = is->scene.CreateEntity();
        TransformComponent tc;
        is->scene.AddComponent(e, tc);
        // Asymmetric box: Dzhanibekov-capable
        glm::vec3 halfExtents{ 1.5f, 0.1f, 0.5f };
        RigidBodyComponent rb = MakeDynamicBody(1.0f, ComputeBoxInertia(1.0f, halfExtents));
        rb.angularMomentum = { 0.01f, 0.01f, 3.0f }; // primarily intermediate axis
        is->scene.AddComponent(e, rb);

        float initialMag = glm::length(rb.angularMomentum);
        auto* initialBody = is->scene.GetComponent<RigidBodyComponent>(e);
        // Compute the analytical initial kinetic energy from body inertia:
        glm::vec3 initL = rb.angularMomentum;
        float initialE = 0.5f * (
            initL.x * initL.x * rb.invInertiaBody[0][0] +
            initL.y * initL.y * rb.invInertiaBody[1][1] +
            initL.z * initL.z * rb.invInertiaBody[2][2]
            );

        float maxEnergyDrift = 0.0f;
        int flips = 0;
        bool hasPrev = false;
        bool prevSign = false;

        // Run 60 seconds (3600 ticks)
        for (int i = 0; i < 3600; ++i)
        {
            StepScene(is->scene, 1);
            auto* curTc = is->scene.GetComponent<TransformComponent>(e);
            auto* curBody = is->scene.GetComponent<RigidBodyComponent>(e);

            float curE = 0.5f * glm::dot(curBody->angularMomentum, curBody->angularVelocity);
            float eDrift = std::abs(curE - initialE) / initialE;
            if (eDrift > maxEnergyDrift)
            {
                maxEnergyDrift = eDrift;
            }

            glm::mat3 rot = glm::mat3_cast(curTc->rotation);
            glm::vec3 bodyL = glm::transpose(rot) * curBody->angularMomentum;
            bool sign = bodyL.z > 0.0f;
            if (hasPrev && sign != prevSign)
            {
                ++flips;
            }
            prevSign = sign;
            hasPrev = true;
        }

        auto* finalBody = is->scene.GetComponent<RigidBodyComponent>(e);
        float finalMag = glm::length(finalBody->angularMomentum);
        float lDrift = std::abs(finalMag - initialMag) / initialMag;

        std::ostringstream oss;
        oss << "|L| drift=" << (lDrift * 100.0f) << "% max E drift=" << (maxEnergyDrift * 100.0f)
            << "% flips=" << flips;
        Assert("Dzhanibekov energy conservation (60s)", maxEnergyDrift < 0.001f, oss.str());
        Assert("Dzhanibekov periodic flips (60s)", flips >= 15 && flips <= 25, oss.str());
    }

    void RunMajorAxisStabilityTest()
    {
        auto is = CreateIntegrationScene();
        is->physics->SetGravity({ 0.0f, 0.0f, 0.0f });

        Entity e = is->scene.CreateEntity();
        TransformComponent tc;
        is->scene.AddComponent(e, tc);
        glm::vec3 halfExtents{ 1.5f, 0.1f, 0.5f };
        RigidBodyComponent rb = MakeDynamicBody(1.0f, ComputeBoxInertia(1.0f, halfExtents));
        rb.angularMomentum = { 0.01f, 3.0f, 0.01f }; // major axis spin
        is->scene.AddComponent(e, rb);

        glm::vec3 initL = rb.angularMomentum;
        float initialE = 0.5f * (
            initL.x * initL.x * rb.invInertiaBody[0][0] +
            initL.y * initL.y * rb.invInertiaBody[1][1] +
            initL.z * initL.z * rb.invInertiaBody[2][2]
            );
        float minLyRatio = 1.0f;
        float maxEDrift = 0.0f;

        for (int i = 0; i < 3600; ++i)
        {
            StepScene(is->scene, 1);
            auto* curTc = is->scene.GetComponent<TransformComponent>(e);
            auto* curBody = is->scene.GetComponent<RigidBodyComponent>(e);

            float curE = 0.5f * glm::dot(curBody->angularMomentum, curBody->angularVelocity);
            float eDrift = std::abs(curE - initialE) / initialE;
            if (eDrift > maxEDrift)
            {
                maxEDrift = eDrift;
            }

            glm::mat3 rot = glm::mat3_cast(curTc->rotation);
            glm::vec3 bodyL = glm::transpose(rot) * curBody->angularMomentum;
            float ratio = std::abs(bodyL.y) / glm::length(curBody->angularMomentum);
            if (ratio < minLyRatio)
            {
                minLyRatio = ratio;
            }
        }

        std::ostringstream oss;
        oss << "min Ly ratio=" << minLyRatio << " max E drift=" << (maxEDrift * 100.0f) << "%";
        Assert("Major axis spin stability (60s)", minLyRatio >= 0.999f && maxEDrift < 0.001f, oss.str());
    }

    void RunQuaternionNormalizationTest()
    {
        auto is = CreateIntegrationScene();
        is->physics->SetGravity({ 0.0f, 0.0f, 0.0f });

        Entity e = is->scene.CreateEntity();
        TransformComponent tc;
        is->scene.AddComponent(e, tc);
        RigidBodyComponent rb = MakeDynamicBody(1.0f, ComputeBoxInertia(1.0f, { 1.0f, 0.5f, 0.3f }));
        rb.angularMomentum = { 5.0f, 3.0f, 7.0f }; // high spin
        is->scene.AddComponent(e, rb);

        StepScene(is->scene, 6000); // 100 seconds at 60Hz
        auto* t = is->scene.GetComponent<TransformComponent>(e);
        float qNorm = glm::length(t->rotation);

        std::ostringstream oss;
        oss << "|q| after 6000 ticks = " << qNorm;
        Assert("Quaternion normalization (6000 ticks)", std::abs(qNorm - 1.0f) < 0.001f, oss.str());
    }

    void RunAccumulatorClearingTest()
    {
        auto is = CreateIntegrationScene();
        is->physics->SetGravity({ 0.0f, 0.0f, 0.0f });

        Entity e = is->scene.CreateEntity();
        TransformComponent tc;
        is->scene.AddComponent(e, tc);
        RigidBodyComponent rb = MakeDynamicBody(1.0f, ComputeSphereInertia(1.0f, 0.5f));
        rb.forceAccum = { 60.0f, 0.0f, 0.0f };
        is->scene.AddComponent(e, rb);

        // Tick 1: force is applied
        StepScene(is->scene, 1);
        auto* body = is->scene.GetComponent<RigidBodyComponent>(e);
        float velAfter1 = body->velocity.x;

        // Tick 2: force should be cleared, velocity shouldn't change (no gravity)
        StepScene(is->scene, 1);
        float velAfter2 = body->velocity.x;
        float accelTick2 = std::abs(velAfter2 - velAfter1);

        std::ostringstream oss;
        oss << "v_tick1=" << velAfter1 << " v_tick2=" << velAfter2 << " accel_tick2=" << accelTick2;
        Assert("Force accumulator clearing", velAfter1 > 0.5f && accelTick2 < 0.01f, oss.str());
    }

    // --- Visual: trajectory drawing ---

    struct VisualTrajectory
    {
        std::string name;
        std::vector<glm::vec3> simulatedTrail;
        std::vector<glm::vec3> analyticalTrail;
        glm::vec3 simColor;
        glm::vec3 analyticalColor;
    };

    std::vector<VisualTrajectory> BuildVisualTrajectories()
    {
        std::vector<VisualTrajectory> visuals;

        // Free-fall trail
        {
            VisualTrajectory v;
            v.name = "Free-fall from y=10";
            v.simColor = { 0.3f, 0.6f, 1.0f };
            v.analyticalColor = { 1.0f, 1.0f, 1.0f };

            auto is = CreateIntegrationScene();
            Entity e = is->scene.CreateEntity();
            TransformComponent tc;
            tc.position = { 0.0f, 10.0f, 0.0f };
            is->scene.AddComponent(e, tc);
            is->scene.AddComponent(e, MakeDynamicBody(1.0f, ComputeSphereInertia(1.0f, 0.5f)));

            for (int i = 0; i <= 60; ++i)
            {
                auto* t = is->scene.GetComponent<TransformComponent>(e);
                v.simulatedTrail.push_back(t->position);
                float time = static_cast<float>(i) * static_cast<float>(kFixedDt);
                v.analyticalTrail.push_back({ 0.0f, 10.0f - 0.5f * kGravity * time * time, 0.0f });
                StepScene(is->scene, 1);
            }
            visuals.push_back(std::move(v));
        }

        // Projectile trail
        {
            VisualTrajectory v;
            v.name = "Projectile throw";
            v.simColor = { 1.0f, 0.5f, 0.2f };
            v.analyticalColor = { 1.0f, 1.0f, 1.0f };

            auto is = CreateIntegrationScene();
            Entity e = is->scene.CreateEntity();
            TransformComponent tc;
            tc.position = { -5.0f, 8.0f, 0.0f };
            is->scene.AddComponent(e, tc);
            RigidBodyComponent rb = MakeDynamicBody(1.0f, ComputeSphereInertia(1.0f, 0.5f));
            rb.velocity = { 8.0f, 5.0f, 0.0f };
            is->scene.AddComponent(e, rb);

            for (int i = 0; i <= 90; ++i)
            {
                auto* t = is->scene.GetComponent<TransformComponent>(e);
                v.simulatedTrail.push_back(t->position);
                float time = static_cast<float>(i) * static_cast<float>(kFixedDt);
                v.analyticalTrail.push_back({
                    -5.0f + 8.0f * time,
                    8.0f + 5.0f * time - 0.5f * kGravity * time * time,
                    0.0f
                    });
                StepScene(is->scene, 1);
            }
            visuals.push_back(std::move(v));
        }

        return visuals;
    }
}

int main()
{
    RT_LOG_INFO("Integration Test harness starting.");

    // ── Phase 1: Automated assertions ──

    RunFreeFallTest();
    RunFreeFallLongTest();
    RunProjectileTest();
    RunStaticBodyIgnoresForceTest();
    RunAngularMomentumConservationTest();
    RunAsymmetricTumblingTest();
    RunMajorAxisStabilityTest();
    RunQuaternionNormalizationTest();
    RunAccumulatorClearingTest();

    int passCount = 0;
    for (const auto& r : results)
    {
        if (r.passed)
        {
            ++passCount;
        }
        RT_LOG_INFO((r.passed ? "[PASS] " : "[FAIL] ") << r.name << " - " << r.detail);
    }
    RT_LOG_INFO(passCount << "/" << results.size() << " automated checks passed.");

    // ── Phase 2: Visual — trajectories + live angular demonstrations ──

    Renderer renderer(1280, 720, "RT-PhysicsCore Integration Test");
    if (!renderer.IsValid())
    {
        RT_LOG_FATAL("Renderer failed to initialize.");
        return 1;
    }

    auto trajectoryVisuals = BuildVisualTrajectories();

    struct LiveAngularCase
    {
        std::string name;
        glm::vec3 halfExtents;
        bool isSphere;
        glm::vec3 initialL;
        glm::vec3 bodyColor;
    };

    std::vector<LiveAngularCase> angularCases = {
        {"Spinning sphere (L conserved, omega constant)",
            { 0.5f, 0.5f, 0.5f }, true,
            { 0.0f, 2.0f, 0.0f }, { 0.3f, 0.8f, 0.4f }},
        {"Dzhanibekov tumble (L conserved, omega wobbles)",
            { 1.5f, 0.1f, 0.5f }, false,
            { 0.01f, 0.01f, 3.0f }, { 1.0f, 0.5f, 0.2f }},
        {"Fast spin - quaternion stability",
            { 1.0f, 0.5f, 0.3f }, false,
            { 5.0f, 3.0f, 7.0f }, { 0.6f, 0.3f, 0.9f }},
    };

    size_t totalVisuals = trajectoryVisuals.size() + angularCases.size();
    size_t current = 0;
    double elapsed = 0.0;
    constexpr double kSecondsPerVisual = 6.0;
    auto lastTime = std::chrono::steady_clock::now();

    std::unique_ptr<IntegrationScene> liveScene;
    Entity liveEntity = 0;

    auto activateCase = [&](size_t idx)
        {
            if (idx < trajectoryVisuals.size())
            {
                RT_LOG_INFO("Showing trajectory: " << trajectoryVisuals[idx].name);
                liveScene.reset();
            }
            else
            {
                size_t angIdx = idx - trajectoryVisuals.size();
                const auto& ac = angularCases[angIdx];
                RT_LOG_INFO("Showing live angular: " << ac.name);

                liveScene = CreateIntegrationScene();
                liveScene->physics->SetGravity({ 0.0f, 0.0f, 0.0f });

                liveEntity = liveScene->scene.CreateEntity();
                TransformComponent tc;
                liveScene->scene.AddComponent(liveEntity, tc);

                glm::mat3 inertia = ac.isSphere
                    ? ComputeSphereInertia(1.0f, ac.halfExtents.x)
                    : ComputeBoxInertia(1.0f, ac.halfExtents);
                RigidBodyComponent rb = MakeDynamicBody(1.0f, inertia);
                rb.angularMomentum = ac.initialL;
                liveScene->scene.AddComponent(liveEntity, rb);
            }
        };

    activateCase(0);

    while (!renderer.ShouldClose())
    {
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - lastTime).count();
        lastTime = now;
        if (dt > 0.1)
        {
            dt = 0.1;
        }
        elapsed += dt;

        if (elapsed >= kSecondsPerVisual)
        {
            elapsed = 0.0;
            current = (current + 1) % totalVisuals;
            activateCase(current);
        }

        if (current < trajectoryVisuals.size())
        {
            const auto& vis = trajectoryVisuals[current];

            for (size_t i = 1; i < vis.simulatedTrail.size(); ++i)
            {
                DebugDraw::Line(vis.simulatedTrail[i - 1], vis.simulatedTrail[i], vis.simColor);
            }
            if (!vis.simulatedTrail.empty())
            {
                DebugDraw::Sphere(vis.simulatedTrail.front(), 0.15f, vis.simColor, 8);
                DebugDraw::Sphere(vis.simulatedTrail.back(), 0.15f, vis.simColor, 8);
            }

            for (size_t i = 1; i < vis.analyticalTrail.size(); i += 2)
            {
                DebugDraw::Line(vis.analyticalTrail[i - 1], vis.analyticalTrail[i],
                    vis.analyticalColor);
            }

            size_t checkPoints[] = { 15, 30, 45, 60 };
            for (size_t cp : checkPoints)
            {
                if (cp < vis.simulatedTrail.size() && cp < vis.analyticalTrail.size())
                {
                    DebugDraw::Sphere(vis.simulatedTrail[cp], 0.08f,
                        { 0.0f, 1.0f, 0.0f }, 6, false);
                    DebugDraw::Sphere(vis.analyticalTrail[cp], 0.08f,
                        { 1.0f, 0.0f, 0.0f }, 6, false);
                    DebugDraw::Line(vis.simulatedTrail[cp], vis.analyticalTrail[cp],
                        { 1.0f, 1.0f, 0.0f }, false);
                }
            }
        }
        else if (liveScene)
        {
            size_t angIdx = current - trajectoryVisuals.size();
            const auto& ac = angularCases[angIdx];

            int subSteps = static_cast<int>(dt / kFixedDt);
            if (subSteps < 1)
            {
                subSteps = 1;
            }
            if (subSteps > 4)
            {
                subSteps = 4;
            }
            for (int s = 0; s < subSteps; ++s)
            {
                liveScene->scene.SetFixedDeltaTime(kFixedDt);
                liveScene->scene.FixedUpdateSystems();
            }

            auto* tc = liveScene->scene.GetComponent<TransformComponent>(liveEntity);
            auto* rb = liveScene->scene.GetComponent<RigidBodyComponent>(liveEntity);

            glm::mat3 rot = glm::mat3_cast(tc->rotation);
            glm::vec3 pos(0.0f);

            if (ac.isSphere)
            {
                DebugDraw::Sphere(pos, ac.halfExtents.x, ac.bodyColor);
            }
            else
            {
                glm::vec3 c[8];
                int idx = 0;
                for (float sx : { -1.0f, 1.0f })
                {
                    for (float sy : { -1.0f, 1.0f })
                    {
                        for (float sz : { -1.0f, 1.0f })
                        {
                            c[idx++] = pos
                                + rot[0] * sx * ac.halfExtents.x
                                + rot[1] * sy * ac.halfExtents.y
                                + rot[2] * sz * ac.halfExtents.z;
                        }
                    }
                }
                static const int edges[12][2] = {
                    {0,1},{0,2},{0,4},{3,1},{3,2},{3,7},
                    {5,1},{5,4},{5,7},{6,2},{6,4},{6,7}
                };
                for (auto& edge : edges)
                {
                    DebugDraw::Line(c[edge[0]], c[edge[1]], ac.bodyColor);
                }
            }

            float axisLen = 0.8f;
            DebugDraw::Line(pos, pos + rot[0] * axisLen, { 0.8f, 0.2f, 0.2f });
            DebugDraw::Line(pos, pos + rot[1] * axisLen, { 0.2f, 0.8f, 0.2f });
            DebugDraw::Line(pos, pos + rot[2] * axisLen, { 0.2f, 0.2f, 0.8f });

            float lScale = 1.5f;
            glm::vec3 lDir = rb->angularMomentum * lScale;
            DebugDraw::Line(pos, pos + lDir, { 1.0f, 0.0f, 1.0f }, false);
            DebugDraw::Sphere(pos + lDir, 0.06f, { 1.0f, 0.0f, 1.0f }, 6, false);

            glm::vec3 omegaDir = rb->angularVelocity * 0.3f;
            DebugDraw::Line(pos, pos + omegaDir, { 1.0f, 1.0f, 0.0f }, false);
            DebugDraw::Sphere(pos + omegaDir, 0.06f, { 1.0f, 1.0f, 0.0f }, 6, false);

            float lMag = glm::length(rb->angularMomentum);
            float initialMag = glm::length(ac.initialL);
            glm::vec3 magColor = (std::abs(lMag - initialMag) / initialMag < 0.01f)
                ? glm::vec3(0.0f, 1.0f, 0.0f)
                : glm::vec3(1.0f, 0.0f, 0.0f);
            DebugDraw::Sphere(pos, lMag * lScale, magColor, 16);
        }

        renderer.BeginFrame();
        renderer.FlushDebugDraw();
        renderer.EndFrame();
    }

    return 0;
}