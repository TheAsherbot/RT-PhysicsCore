/**
 * @file SolverComparisonTest.cpp
 * @brief Side-by-side comparison of Sequential Impulse and LCP Lemke contact solvers.
 *
 * Runs automated pass/fail assertions comparing solver output on identical scenarios,
 * then renders an interactive split visualization with both solvers running in parallel.
 */

#include <chrono>
#include <cmath>
#include <functional>
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
#include "RT-PhysicsCore/physics/collision/Contact.h"
#include "RT-PhysicsCore/physics/MassProperties.h"

#include <glm/gtc/quaternion.hpp>

namespace
{
    using namespace RT_PhysicsCore;

    /// Bundles a Scene with its physics system pointers. Heap-allocated via unique_ptr
    /// to keep the Scene address stable (systems hold Scene& references).
    struct PhysicsScene
    {
        Scene scene;
        PhysicsSystem* physics = nullptr;
        CollisionSystem* collision = nullptr;
        ResolutionSystem* resolution = nullptr;
    };

    std::unique_ptr<PhysicsScene> CreatePhysicsScene(ResolutionSystem::SolverMode mode)
    {
        auto ps = std::make_unique<PhysicsScene>();

        ps->scene.AddSystem(std::make_unique<TransformPropagationSystem>(ps->scene));

        auto phys = std::make_unique<PhysicsSystem>(ps->scene);
        ps->physics = phys.get();
        ps->scene.AddSystem(std::move(phys));

        auto col = std::make_unique<CollisionSystem>(ps->scene);
        ps->collision = col.get();
        ps->scene.AddSystem(std::move(col));

        auto res = std::make_unique<ResolutionSystem>(ps->scene, *ps->collision, mode);
        ps->resolution = res.get();
        ps->scene.AddSystem(std::move(res));

        return ps;
    }

    // --- Entity Factories ---

    Entity CreateGround(Scene& scene)
    {
        Entity e = scene.CreateEntity();
        TransformComponent tc;
        tc.position = { 0.0f, -0.5f, 0.0f };
        scene.AddComponent(e, tc);
        scene.AddComponent(e, MakeStaticBody());
        ColliderComponent cc;
        cc.shape = ColliderShape::Box;
        cc.size = { 10.0f, 0.5f, 10.0f };
        scene.AddComponent(e, cc);
        return e;
    }

    Entity CreateSphere(Scene& scene, const glm::vec3& pos, const glm::vec3& vel,
        float radius = 0.5f, float mass = 1.0f)
    {
        Entity e = scene.CreateEntity();
        TransformComponent tc;
        tc.position = pos;
        scene.AddComponent(e, tc);
        RigidBodyComponent rb = MakeDynamicBody(mass, ComputeSphereInertia(mass, radius));
        rb.velocity = vel;
        scene.AddComponent(e, rb);
        ColliderComponent cc;
        cc.shape = ColliderShape::Sphere;
        cc.size = { radius, 0.0f, 0.0f };
        scene.AddComponent(e, cc);
        return e;
    }

    Entity CreateBox(Scene& scene, const glm::vec3& pos, const glm::vec3& halfExtents,
        float mass, const glm::vec3& vel = glm::vec3(0.0f))
    {
        Entity e = scene.CreateEntity();
        TransformComponent tc;
        tc.position = pos;
        scene.AddComponent(e, tc);
        if (mass <= 0.0f)
        {
            scene.AddComponent(e, MakeStaticBody());
        }
        else
        {
            RigidBodyComponent rb = MakeDynamicBody(mass, ComputeBoxInertia(mass, halfExtents));
            rb.velocity = vel;
            scene.AddComponent(e, rb);
        }
        ColliderComponent cc;
        cc.shape = ColliderShape::Box;
        cc.size = halfExtents;
        scene.AddComponent(e, cc);
        return e;
    }

    Entity CreateWall(Scene& scene, const glm::vec3& pos, const glm::vec3& halfExtents)
    {
        return CreateBox(scene, pos, halfExtents, 0.0f);
    }

    void StepScene(Scene& scene, int ticks, double dt = 1.0 / 60.0)
    {
        for (int i = 0; i < ticks; ++i)
        {
            scene.SetFixedDeltaTime(dt);
            scene.FixedUpdateSystems();
        }
    }

    // --- Debug Drawing Helpers ---

    void DrawOrientedBox(const glm::vec3& center, const glm::vec3& halfExtents,
        const glm::mat3& rot, const glm::vec3& color)
    {
        glm::vec3 c[8];
        int idx = 0;
        for (float sx : {-1.0f, 1.0f})
        {
            for (float sy : {-1.0f, 1.0f})
            {
                for (float sz : {-1.0f, 1.0f})
                {
                    c[idx++] = center
                        + rot[0] * sx * halfExtents.x
                        + rot[1] * sy * halfExtents.y
                        + rot[2] * sz * halfExtents.z;
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

    void DrawSceneEntities(Scene& scene, const glm::vec3& offset, const glm::vec3& color)
    {
        auto entities = scene.Query<TransformComponent, ColliderComponent>();
        for (Entity e : entities)
        {
            auto* tc = scene.GetComponent<TransformComponent>(e);
            auto* cc = scene.GetComponent<ColliderComponent>(e);
            if (!tc || !cc)
            {
                continue;
            }

            glm::vec3 pos = tc->position + cc->offset + offset;
            glm::mat3 rot = glm::mat3_cast(tc->rotation);

            switch (cc->shape)
            {
            case ColliderShape::Sphere:
                DebugDraw::Sphere(pos, cc->size.x, color);
                break;
            case ColliderShape::Box:
                DrawOrientedBox(pos, cc->size, rot, color);
                break;
            case ColliderShape::Capsule:
            {
                glm::vec3 axis = rot[1] * cc->size.y;
                DebugDraw::Line(pos - axis, pos + axis, color);
                DebugDraw::Sphere(pos - axis, cc->size.x, color, 12);
                DebugDraw::Sphere(pos + axis, cc->size.x, color, 12);
                break;
            }
            }
        }
    }

    void DrawContactPoints(CollisionSystem* colSys, const glm::vec3& offset)
    {
        const auto& contacts = colSys->GetContacts();
        for (const Contact& c : contacts)
        {
            for (int i = 0; i < c.pointCount; ++i)
            {
                glm::vec3 p = c.points[i] + offset;
                DebugDraw::Sphere(p, 0.05f, { 1.0f, 1.0f, 0.0f }, 8, false);
                DebugDraw::Line(p, p + c.normal * 0.4f, { 1.0f, 0.0f, 0.0f }, false);
            }
        }
    }

    // --- Test Scenarios ---

    using ScenarioSetupFn = std::function<Entity(Scene&)>;

    struct ComparisonTestCase
    {
        std::string name;
        ScenarioSetupFn setup;
        int ticks;
        float posTolerance;
        float velTolerance;
    };

    std::vector<ComparisonTestCase> BuildTests()
    {
        std::vector<ComparisonTestCase> tests;

        tests.push_back({ "Ball drop onto ground",
            [](Scene& s) -> Entity
            {
                CreateGround(s);
                return CreateSphere(s, {0.0f, 5.0f, 0.0f}, {0.0f, 0.0f, 0.0f});
            },
            180, 0.15f, 0.2f });

        tests.push_back({ "Head-on sphere collision (equal mass)",
            [](Scene& s) -> Entity
            {
                CreateSphere(s, {-3.0f, 1.0f, 0.0f}, {5.0f, 0.0f, 0.0f});
                return CreateSphere(s, {3.0f, 1.0f, 0.0f}, {-5.0f, 0.0f, 0.0f});
            },
            30, 0.1f, 0.3f });

        tests.push_back({ "Three-body chain collision",
            [](Scene& s) -> Entity
            {
                CreateGround(s);
                CreateSphere(s, {-3.0f, 0.5f, 0.0f}, {6.0f, 0.0f, 0.0f});
                CreateSphere(s, {0.0f, 0.5f, 0.0f}, {0.0f, 0.0f, 0.0f});
                return CreateSphere(s, {3.0f, 0.5f, 0.0f}, {0.0f, 0.0f, 0.0f});
            },
            60, 0.3f, 0.5f });

        tests.push_back({ "Resting 3-box stack",
            [](Scene& s) -> Entity
            {
                CreateGround(s);
                CreateBox(s, {0.0f, 0.5f, 0.0f}, {0.5f, 0.5f, 0.5f}, 1.0f);
                CreateBox(s, {0.0f, 1.5f, 0.0f}, {0.5f, 0.5f, 0.5f}, 1.0f);
                return CreateBox(s, {0.0f, 2.5f, 0.0f}, {0.5f, 0.5f, 0.5f}, 1.0f);
            },
            300, 0.5f, 0.3f });

        tests.push_back({ "Ball in corner (simultaneous contacts)",
            [](Scene& s) -> Entity
            {
                CreateGround(s);
                CreateWall(s, {-5.5f, 2.5f, 0.0f}, {0.5f, 5.0f, 5.0f});
                return CreateSphere(s, {-4.0f, 2.0f, 0.0f}, {-5.0f, -5.0f, 0.0f});
            },
            60, 0.5f, 1.0f });

        return tests;
    }

    void DrawLabel(const glm::vec3& origin, const std::string& text,
        float charSize, float spacing, const glm::vec3& color)
    {
        float x = 0.0f;
        for (char ch : text)
        {
            glm::vec3 o = origin + glm::vec3(x, 0.0f, 0.0f);
            float s = charSize;
            switch (ch)
            {
            case 'S':
                DebugDraw::Line(o + glm::vec3(s, 2 * s, 0), o + glm::vec3(0, 2 * s, 0), color, false);
                DebugDraw::Line(o + glm::vec3(0, 2 * s, 0), o + glm::vec3(0, s, 0), color, false);
                DebugDraw::Line(o + glm::vec3(0, s, 0), o + glm::vec3(s, s, 0), color, false);
                DebugDraw::Line(o + glm::vec3(s, s, 0), o + glm::vec3(s, 0, 0), color, false);
                DebugDraw::Line(o + glm::vec3(s, 0, 0), o + glm::vec3(0, 0, 0), color, false);
                break;
            case 'I':
                DebugDraw::Line(o + glm::vec3(0, 2 * s, 0), o + glm::vec3(s, 2 * s, 0), color, false);
                DebugDraw::Line(o + glm::vec3(s * 0.5f, 2 * s, 0), o + glm::vec3(s * 0.5f, 0, 0), color, false);
                DebugDraw::Line(o + glm::vec3(0, 0, 0), o + glm::vec3(s, 0, 0), color, false);
                break;
            case 'L':
                DebugDraw::Line(o + glm::vec3(0, 2 * s, 0), o + glm::vec3(0, 0, 0), color, false);
                DebugDraw::Line(o + glm::vec3(0, 0, 0), o + glm::vec3(s, 0, 0), color, false);
                break;
            case 'C':
                DebugDraw::Line(o + glm::vec3(s, 2 * s, 0), o + glm::vec3(0, 2 * s, 0), color, false);
                DebugDraw::Line(o + glm::vec3(0, 2 * s, 0), o + glm::vec3(0, 0, 0), color, false);
                DebugDraw::Line(o + glm::vec3(0, 0, 0), o + glm::vec3(s, 0, 0), color, false);
                break;
            case 'P':
                DebugDraw::Line(o + glm::vec3(0, 0, 0), o + glm::vec3(0, 2 * s, 0), color, false);
                DebugDraw::Line(o + glm::vec3(0, 2 * s, 0), o + glm::vec3(s, 2 * s, 0), color, false);
                DebugDraw::Line(o + glm::vec3(s, 2 * s, 0), o + glm::vec3(s, s, 0), color, false);
                DebugDraw::Line(o + glm::vec3(s, s, 0), o + glm::vec3(0, s, 0), color, false);
                break;
            default:
                break;
            }
            x += charSize + spacing;
        }
    }
}

int main()
{
    RT_LOG_INFO("Solver Comparison Test harness starting.");

    auto tests = BuildTests();
    constexpr double fixedDt = 1.0 / 60.0;

    // ── Phase 1: Automated pass/fail assertions ──

    int passCount = 0;

    for (const auto& test : tests)
    {
        auto sceneA = CreatePhysicsScene(ResolutionSystem::SolverMode::SequentialImpulses);
        auto sceneB = CreatePhysicsScene(ResolutionSystem::SolverMode::Exact);

        Entity primaryA = test.setup(sceneA->scene);
        Entity primaryB = test.setup(sceneB->scene);

        StepScene(sceneA->scene, test.ticks, fixedDt);
        StepScene(sceneB->scene, test.ticks, fixedDt);

        auto* tcA = sceneA->scene.GetComponent<TransformComponent>(primaryA);
        auto* tcB = sceneB->scene.GetComponent<TransformComponent>(primaryB);
        auto* rbA = sceneA->scene.GetComponent<RigidBodyComponent>(primaryA);
        auto* rbB = sceneB->scene.GetComponent<RigidBodyComponent>(primaryB);

        float posDiff = glm::length(tcA->position - tcB->position);
        float velDiff = (rbA && rbB) ? glm::length(rbA->velocity - rbB->velocity) : 0.0f;

        bool pass = (posDiff <= test.posTolerance) && (velDiff <= test.velTolerance);
        if (pass)
        {
            ++passCount;
        }

        std::ostringstream oss;
        oss << (pass ? "[PASS] " : "[FAIL] ") << test.name
            << " | pos_diff=" << posDiff << " (tol=" << test.posTolerance << ")"
            << " vel_diff=" << velDiff << " (tol=" << test.velTolerance << ")"
            << " | SI=[" << tcA->position.x << "," << tcA->position.y << "," << tcA->position.z << "]"
            << " LCP=[" << tcB->position.x << "," << tcB->position.y << "," << tcB->position.z << "]";
        RT_LOG_INFO(oss.str());
    }
    RT_LOG_INFO(passCount << "/" << tests.size() << " automated checks passed.");

    // ── Phase 2: Interactive split visualization ──

    Renderer renderer(1280, 720, "RT-PhysicsCore Solver Comparison Test");
    if (!renderer.IsValid())
    {
        RT_LOG_FATAL("Renderer failed to initialize.");
        return 1;
    }

    size_t currentScenario = 0;
    double sceneElapsed = 0.0;
    double physicsAccum = 0.0;
    constexpr double kSecondsPerScenario = 8.0;

    auto visSI = CreatePhysicsScene(ResolutionSystem::SolverMode::SequentialImpulses);
    auto visLCP = CreatePhysicsScene(ResolutionSystem::SolverMode::Exact);
    tests[0].setup(visSI->scene);
    tests[0].setup(visLCP->scene);
    RT_LOG_INFO("Showing: " << tests[0].name);

    auto lastTime = std::chrono::steady_clock::now();

    const glm::vec3 kSIOffset{ -6.0f, 0.0f, 0.0f };
    const glm::vec3 kLCPOffset{ 6.0f, 0.0f, 0.0f };
    const glm::vec3 kSIColor{ 0.3f, 0.6f, 1.0f };
    const glm::vec3 kLCPColor{ 1.0f, 0.6f, 0.3f };

    while (!renderer.ShouldClose())
    {
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - lastTime).count();
        lastTime = now;
        if (dt > 0.1)
        {
            dt = 0.1;
        }

        sceneElapsed += dt;
        if (sceneElapsed >= kSecondsPerScenario)
        {
            sceneElapsed = 0.0;
            physicsAccum = 0.0;
            currentScenario = (currentScenario + 1) % tests.size();

            visSI = CreatePhysicsScene(ResolutionSystem::SolverMode::SequentialImpulses);
            visLCP = CreatePhysicsScene(ResolutionSystem::SolverMode::Exact);
            tests[currentScenario].setup(visSI->scene);
            tests[currentScenario].setup(visLCP->scene);
            RT_LOG_INFO("Showing: " << tests[currentScenario].name);
        }

        physicsAccum += dt;
        while (physicsAccum >= fixedDt)
        {
            visSI->scene.SetFixedDeltaTime(fixedDt);
            visSI->scene.FixedUpdateSystems();
            visLCP->scene.SetFixedDeltaTime(fixedDt);
            visLCP->scene.FixedUpdateSystems();
            physicsAccum -= fixedDt;
        }

        DrawSceneEntities(visSI->scene, kSIOffset, kSIColor);
        DrawContactPoints(visSI->collision, kSIOffset);

        DrawSceneEntities(visLCP->scene, kLCPOffset, kLCPColor);
        DrawContactPoints(visLCP->collision, kLCPOffset);

        // Vertical divider
        DebugDraw::Line({ 0.0f, -2.0f, -8.0f }, { 0.0f, 12.0f, -8.0f }, { 1.0f, 1.0f, 1.0f });
        DebugDraw::Line({ 0.0f, -2.0f, 8.0f }, { 0.0f, 12.0f, 8.0f }, { 1.0f, 1.0f, 1.0f });

        // Solver labels drawn as wireframe text
        DrawLabel(kSIOffset + glm::vec3(-1.0f, 8.5f, 0.0f), "SI", 0.5f, 0.2f, kSIColor);
        DrawLabel(kLCPOffset + glm::vec3(-1.5f, 8.5f, 0.0f), "LCP", 0.5f, 0.2f, kLCPColor);

        renderer.BeginFrame();
        renderer.FlushDebugDraw();
        renderer.EndFrame();
    }

    return 0;
}