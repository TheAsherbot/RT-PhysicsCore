/**
 * @file SolverStabilityTest.cpp
 * @brief Validates the stability of the contact solver under heavy resting loads.
 *
 * Simulates stacks of rigid bodies, monitoring total kinetic energy and maximum
 * penetration depth to ensure the system reaches a stable resting state.
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
#include "RT-PhysicsCore/physics/collision/Contact.h"
#include "RT-PhysicsCore/physics/MassProperties.h"

#include <glm/gtc/quaternion.hpp>

namespace
{
    using namespace RT_PhysicsCore;

    struct PhysicsScene
    {
        Scene scene;
        PhysicsSystem* physics = nullptr;
        CollisionSystem* collision = nullptr;
        ResolutionSystem* resolution = nullptr;
    };

    std::unique_ptr<PhysicsScene> CreatePhysicsScene()
    {
        auto ps = std::make_unique<PhysicsScene>();
        ps->scene.AddSystem(std::make_unique<TransformPropagationSystem>(ps->scene));

        auto phys = std::make_unique<PhysicsSystem>(ps->scene);
        ps->physics = phys.get();
        ps->scene.AddSystem(std::move(phys));

        auto col = std::make_unique<CollisionSystem>(ps->scene);
        ps->collision = col.get();
        ps->scene.AddSystem(std::move(col));

        auto res = std::make_unique<ResolutionSystem>(ps->scene, *ps->collision, ResolutionSystem::SolverMode::SequentialImpulses);
        ps->resolution = res.get();
        ps->scene.AddSystem(std::move(res));

        return ps;
    }

    Entity CreateGround(Scene& scene)
    {
        Entity e = scene.CreateEntity();
        TransformComponent tc;
        tc.position = { 0.0f, -0.5f, 0.0f };
        scene.AddComponent(e, tc);
        scene.AddComponent(e, MakeStaticBody());
        ColliderComponent cc;
        cc.shape = ColliderShape::Box;
        cc.size = { 20.0f, 0.5f, 20.0f };
        scene.AddComponent(e, cc);
        return e;
    }

    Entity CreateBox(Scene& scene, const glm::vec3& pos, const glm::vec3& halfExtents, float mass)
    {
        Entity e = scene.CreateEntity();
        TransformComponent tc;
        tc.position = pos;
        scene.AddComponent(e, tc);
        RigidBodyComponent rb = MakeDynamicBody(mass, ComputeBoxInertia(mass, halfExtents));
        scene.AddComponent(e, rb);
        ColliderComponent cc;
        cc.shape = ColliderShape::Box;
        cc.size = halfExtents;
        scene.AddComponent(e, cc);
        return e;
    }

    void StepScene(Scene& scene, int ticks, double dt = 1.0 / 60.0)
    {
        for (int i = 0; i < ticks; ++i)
        {
            scene.SetFixedDeltaTime(dt);
            scene.FixedUpdateSystems();
        }
    }

    float GetTotalKineticEnergy(Scene& scene)
    {
        float ke = 0.0f;
        for (Entity e : scene.Query<RigidBodyComponent>())
        {
            auto* rb = scene.GetComponent<RigidBodyComponent>(e);
            if (rb->invMass > 0.0f)
            {
                float speedSq = glm::dot(rb->velocity, rb->velocity);
                ke += 0.5f * rb->mass * speedSq;

                // Angular kinetic energy: 0.5 * w * L
                ke += 0.5f * glm::dot(rb->angularVelocity, rb->angularMomentum);
            }
        }
        return ke;
    }

    float GetMaxPenetration(CollisionSystem* colSys)
    {
        float maxPen = 0.0f;
        for (const Contact& c : colSys->GetContacts())
        {
            for (int i = 0; i < c.pointCount; ++i)
            {
                maxPen = std::max(maxPen, c.penetrations[i]);
            }
        }
        return maxPen;
    }

    void DrawOrientedBox(const glm::vec3& center, const glm::vec3& halfExtents, const glm::mat3& rot, const glm::vec3& color)
    {
        glm::vec3 c[8];
        int idx = 0;
        for (float sx : {-1.0f, 1.0f})
            for (float sy : {-1.0f, 1.0f})
                for (float sz : {-1.0f, 1.0f})
                    c[idx++] = center + rot[0] * sx * halfExtents.x + rot[1] * sy * halfExtents.y + rot[2] * sz * halfExtents.z;

        static const int edges[12][2] = {
            {0,1},{0,2},{0,4},{3,1},{3,2},{3,7},{5,1},{5,4},{5,7},{6,2},{6,4},{6,7} };
        for (auto& edge : edges)
            DebugDraw::Line(c[edge[0]], c[edge[1]], color);
    }
}

int main()
{
    RT_LOG_INFO("Solver Stability Test harness starting.");

    // ── Phase 1: Automated assertions ──

    int passCount = 0;
    int totalCount = 0;
    constexpr double fixedDt = 1.0 / 60.0;

    {
        ++totalCount;
        auto ps = CreatePhysicsScene();
        CreateGround(ps->scene);

        // Build a vertical stack of 10 boxes
        for (int i = 0; i < 10; ++i)
        {
            CreateBox(ps->scene, { 0.0f, 0.5f + i * 1.0f, 0.0f }, { 0.5f, 0.5f, 0.5f }, 1.0f);
        }

        // Simulate for 5 seconds (300 ticks) to let it settle
        StepScene(ps->scene, 300, fixedDt);

        float finalKe = GetTotalKineticEnergy(ps->scene);
        float maxPen = GetMaxPenetration(ps->collision);

        // Expect it to be practically asleep (very low kinetic energy)
        bool pass = finalKe < 0.05f && maxPen < 0.02f;
        if (pass) ++passCount;

        std::ostringstream oss;
        oss << "KE=" << finalKe << " (tol 0.05), MaxPen=" << maxPen << " (tol 0.02)";
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "10-Box Stack Settling - " << oss.str());
    }

    RT_LOG_INFO(passCount << "/" << totalCount << " automated checks passed.");

    // ── Phase 2: Interactive visualization ──

    Renderer renderer(1280, 720, "RT-PhysicsCore Solver Stability Test");
    if (!renderer.IsValid())
    {
        RT_LOG_FATAL("Renderer failed to initialize.");
        return 1;
    }

    auto ps = CreatePhysicsScene();
    CreateGround(ps->scene);

    // Build two stacks to visualize
    // Stack 1: Standard uniform boxes
    for (int i = 0; i < 8; ++i)
    {
        CreateBox(ps->scene, { -3.0f, 0.5f + i * 1.01f, 0.0f }, { 0.5f, 0.5f, 0.5f }, 1.0f);
    }

    // Stack 2: Interleaved sizes and masses
    for (int i = 0; i < 6; ++i)
    {
        float w = (i % 2 == 0) ? 1.0f : 0.5f;
        float h = (i % 2 == 0) ? 0.25f : 0.5f;
        float m = (i % 2 == 0) ? 5.0f : 1.0f;
        float y = 0.25f + i * 0.76f;
        CreateBox(ps->scene, { 3.0f, y, 0.0f }, { w, h, w }, m);
    }

    auto lastTime = std::chrono::steady_clock::now();
    double physicsAccum = 0.0;

    while (!renderer.ShouldClose())
    {
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - lastTime).count();
        lastTime = now;
        if (dt > 0.1) dt = 0.1;

        physicsAccum += dt;
        while (physicsAccum >= fixedDt)
        {
            ps->scene.SetFixedDeltaTime(fixedDt);
            ps->scene.FixedUpdateSystems();
            physicsAccum -= fixedDt;
        }

        // Render entities
        for (Entity e : ps->scene.Query<TransformComponent, ColliderComponent>())
        {
            auto* tc = ps->scene.GetComponent<TransformComponent>(e);
            auto* cc = ps->scene.GetComponent<ColliderComponent>(e);
            auto* rb = ps->scene.GetComponent<RigidBodyComponent>(e);

            glm::vec3 color = (rb && rb->invMass == 0.0f) ? glm::vec3(0.5f) : glm::vec3(0.3f, 0.8f, 0.4f);

            if (cc->shape == ColliderShape::Box)
            {
                DrawOrientedBox(tc->position + cc->offset, cc->size, glm::mat3_cast(tc->rotation), color);
            }
        }

        float currentKe = GetTotalKineticEnergy(ps->scene);

        // Draw an energy bar in the world to visualize the settling
        DebugDraw::Line({ -5.0f, 0.0f, -3.0f }, { -5.0f, 5.0f, -3.0f }, { 0.2f, 0.2f, 0.2f });
        float scaledKe = std::min(currentKe * 0.1f, 5.0f);
        DebugDraw::Line({ -5.0f, 0.0f, -3.0f }, { -5.0f, scaledKe, -3.0f }, { 1.0f, 0.0f, 0.0f });

        renderer.BeginFrame();
        renderer.FlushDebugDraw();
        renderer.EndFrame();
    }

    return 0;
}