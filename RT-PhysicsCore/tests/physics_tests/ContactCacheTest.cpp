/**
 * @file ContactCacheTest.cpp
 * @brief Automated verification and visual demonstration of temporal contact manifold warm-starting.
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
#include "RT-PhysicsCore/physics/components/RigidBodyComponent.h"
#include "RT-PhysicsCore/physics/components/ColliderComponent.h"
#include "RT-PhysicsCore/physics/collision/Contact.h"
#include "RT-PhysicsCore/physics/collision/ContactCache.h"
#include "RT-PhysicsCore/physics/systems/PhysicsPipelineSystem.h"

namespace
{
    using namespace RT_PhysicsCore;

    RigidBodyComponent MakeStaticBody()
    {
        RigidBodyComponent rb;
        rb.invMass = 0.0;
        rb.invInertiaWorld = Mat3(0.0);
        return rb;
    }

    RigidBodyComponent MakeDynamicBody(Real mass, const Mat3& inertia)
    {
        RigidBodyComponent rb;
        rb.invMass = (mass > 0.0) ? (1.0 / mass) : 0.0;
        rb.invInertiaWorld = (mass > 0.0) ? glm::inverse(inertia) : Mat3(0.0);
        return rb;
    }

    Mat3 ComputeBoxInertia(Real mass, const Vec3& halfExtents)
    {
        Real x2 = (2.0 * halfExtents.x) * (2.0 * halfExtents.x);
        Real y2 = (2.0 * halfExtents.y) * (2.0 * halfExtents.y);
        Real z2 = (2.0 * halfExtents.z) * (2.0 * halfExtents.z);
        Real factor = mass / 12.0;

        Mat3 inertia(0.0);
        inertia[0][0] = factor * (y2 + z2);
        inertia[1][1] = factor * (x2 + z2);
        inertia[2][2] = factor * (x2 + y2);
        return inertia;
    }
}

int main(int argc, char* argv[])
{
    RT_LOG_INFO("Contact Cache Test harness starting.");

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

    // ── Check 1: Impulse Persistence & Warm-Start Restoration ──
    {
        ++totalCount;
        ContactCache cache;
        Entity eA = 10;
        Entity eB = 20;

        Contact initialContact;
        initialContact.a = eA;
        initialContact.b = eB;
        initialContact.normal = { 0.0, 1.0, 0.0 };
        initialContact.tangent1 = { 1.0, 0.0, 0.0 };
        initialContact.tangent2 = { 0.0, 0.0, 1.0 };
        initialContact.pointCount = 1;
        initialContact.points[0].position = { 0.0, 0.5, 0.0 };
        initialContact.points[0].localAnchorA = { 0.0, -0.5, 0.0 };
        initialContact.points[0].localAnchorB = { 0.0, 0.5, 0.0 };
        initialContact.points[0].normalImpulse = 24.5;
        initialContact.points[0].tangentImpulse1 = 4.2;
        initialContact.points[0].tangentImpulse2 = -1.8;
        initialContact.points[0].isStuck = true;

        cache.BeginSubstep();
        cache.Store(initialContact);
        cache.EndSubstep();

        // Subsequent substep: query with zeroed impulses
        cache.BeginSubstep();
        Contact queryContact;
        queryContact.a = eA;
        queryContact.b = eB;
        queryContact.normal = { 0.0, 1.0, 0.0 };
        queryContact.tangent1 = { 1.0, 0.0, 0.0 };
        queryContact.tangent2 = { 0.0, 0.0, 1.0 };
        queryContact.pointCount = 1;
        queryContact.points[0].position = { 0.0, 0.5, 0.0 };
        queryContact.points[0].localAnchorA = { 0.0, -0.5, 0.0 };
        queryContact.points[0].localAnchorB = { 0.0, 0.5, 0.0 };
        queryContact.points[0].normalImpulse = 0.0;
        queryContact.points[0].tangentImpulse1 = 0.0;
        queryContact.points[0].tangentImpulse2 = 0.0;
        queryContact.points[0].isStuck = false;

        cache.MatchAndWarmStart(queryContact);

        bool normalMatch = std::abs(queryContact.points[0].normalImpulse - 24.5) < 1e-4;
        bool tangent1Match = std::abs(queryContact.points[0].tangentImpulse1 - 4.2) < 1e-4;
        bool tangent2Match = std::abs(queryContact.points[0].tangentImpulse2 - (-1.8)) < 1e-4;
        bool stuckMatch = (queryContact.points[0].isStuck == true);

        bool pass = normalMatch && tangent1Match && tangent2Match && stuckMatch;
        if (pass)
        {
            ++passCount;
        }

        std::ostringstream oss;
        oss << "normImp=" << queryContact.points[0].normalImpulse << " (expected 24.5), tan1="
            << queryContact.points[0].tangentImpulse1 << " (expected 4.2), tan2="
            << queryContact.points[0].tangentImpulse2 << " (expected -1.8)";
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "Warm-Start Impulse Persistence - " << oss.str());
    }

    // ── Check 2: Angular Normal Shift Rejection ──
    {
        ++totalCount;
        ContactCache cache;
        Entity eA = 10;
        Entity eB = 20;

        Contact initialContact;
        initialContact.a = eA;
        initialContact.b = eB;
        initialContact.normal = { 0.0, 1.0, 0.0 };
        initialContact.tangent1 = { 1.0, 0.0, 0.0 };
        initialContact.tangent2 = { 0.0, 0.0, 1.0 };
        initialContact.pointCount = 1;
        initialContact.points[0].localAnchorA = { 0.0, -0.5, 0.0 };
        initialContact.points[0].localAnchorB = { 0.0, 0.5, 0.0 };
        initialContact.points[0].normalImpulse = 15.0;

        cache.BeginSubstep();
        cache.Store(initialContact);
        cache.EndSubstep();

        // Next substep: query with tilted normal (> 18 degree angular deviation, dot < 0.95)
        cache.BeginSubstep();
        Contact tiltedContact;
        tiltedContact.a = eA;
        tiltedContact.b = eB;
        tiltedContact.normal = glm::normalize(Vec3(0.5, 0.5, 0.0));
        tiltedContact.tangent1 = { 0.0, 0.0, 1.0 };
        tiltedContact.tangent2 = glm::cross(tiltedContact.normal, tiltedContact.tangent1);
        tiltedContact.pointCount = 1;
        tiltedContact.points[0].localAnchorA = { 0.0, -0.5, 0.0 };
        tiltedContact.points[0].localAnchorB = { 0.0, 0.5, 0.0 };
        tiltedContact.points[0].normalImpulse = 0.0;

        cache.MatchAndWarmStart(tiltedContact);

        bool pass = (tiltedContact.points[0].normalImpulse == 0.0);
        if (pass)
        {
            ++passCount;
        }

        std::ostringstream oss;
        oss << "normalDot=" << glm::dot(initialContact.normal, tiltedContact.normal)
            << " normImp=" << tiltedContact.points[0].normalImpulse << " (expected 0.0)";
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "Angular Normal Shift Rejection - " << oss.str());
    }

    // ── Check 3: Substep Age Invalidation / Pruning ──
    {
        ++totalCount;
        ContactCache cache;
        Entity eA = 10;
        Entity eB = 20;

        Contact initialContact;
        initialContact.a = eA;
        initialContact.b = eB;
        initialContact.normal = { 0.0, 1.0, 0.0 };
        initialContact.tangent1 = { 1.0, 0.0, 0.0 };
        initialContact.tangent2 = { 0.0, 0.0, 1.0 };
        initialContact.pointCount = 1;
        initialContact.points[0].localAnchorA = { 0.0, -0.5, 0.0 };
        initialContact.points[0].localAnchorB = { 0.0, 0.5, 0.0 };
        initialContact.points[0].normalImpulse = 18.0;

        cache.BeginSubstep();
        cache.Store(initialContact);
        cache.EndSubstep();

        // Advance through an idle substep where bodies separated (no match or store)
        cache.BeginSubstep();
        cache.EndSubstep(); // Inactive manifold purged

        // Query in subsequent substep
        cache.BeginSubstep();
        Contact queryContact = initialContact;
        queryContact.points[0].normalImpulse = 0.0;
        cache.MatchAndWarmStart(queryContact);
        cache.EndSubstep();

        bool pass = (queryContact.points[0].normalImpulse == 0.0);
        if (pass)
        {
            ++passCount;
        }

        std::ostringstream oss;
        oss << "normImp=" << queryContact.points[0].normalImpulse << " (expected 0.0 after pruning)";
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "Substep Age Invalidation / Pruning - " << oss.str());
    }

    // ── Check 4: Symmetric Entity Order Inversion Matching ──
    {
        ++totalCount;
        ContactCache cache;
        Entity eA = 10;
        Entity eB = 20;

        Contact initialContact;
        initialContact.a = eA;
        initialContact.b = eB;
        initialContact.normal = { 0.0, 1.0, 0.0 };
        initialContact.tangent1 = { 1.0, 0.0, 0.0 };
        initialContact.tangent2 = { 0.0, 0.0, 1.0 };
        initialContact.pointCount = 1;
        initialContact.points[0].localAnchorA = { 0.1, -0.5, 0.2 };
        initialContact.points[0].localAnchorB = { 0.3, 0.5, 0.4 };
        initialContact.points[0].normalImpulse = 32.0;
        initialContact.points[0].tangentImpulse1 = 5.0;
        initialContact.points[0].tangentImpulse2 = 2.0;

        cache.BeginSubstep();
        cache.Store(initialContact);
        cache.EndSubstep();

        // Subsequent query with inverted entity pairing: A = eB, B = eA
        cache.BeginSubstep();
        Contact invertedContact;
        invertedContact.a = eB;
        invertedContact.b = eA;
        invertedContact.normal = initialContact.normal;
        invertedContact.tangent1 = initialContact.tangent1;
        invertedContact.tangent2 = initialContact.tangent2;
        invertedContact.pointCount = 1;
        // Inverted: localAnchorA corresponds to body B, localAnchorB corresponds to body A
        invertedContact.points[0].localAnchorA = initialContact.points[0].localAnchorB;
        invertedContact.points[0].localAnchorB = initialContact.points[0].localAnchorA;
        invertedContact.points[0].normalImpulse = 0.0;
        invertedContact.points[0].tangentImpulse1 = 0.0;
        invertedContact.points[0].tangentImpulse2 = 0.0;

        cache.MatchAndWarmStart(invertedContact);

        bool pass = std::abs(invertedContact.points[0].normalImpulse - 32.0) < 1e-4;
        if (pass)
        {
            ++passCount;
        }

        std::ostringstream oss;
        oss << "normImp=" << invertedContact.points[0].normalImpulse << " (expected 32.0 on inverted pair)";
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "Symmetric Entity Order Inversion Matching - " << oss.str());
    }

    RT_LOG_INFO(passCount << "/" << totalCount << " automated checks passed.");

    if (!enableGui)
    {
        RT_LOG_INFO("Headless execution complete (pass --gui to open interactive visualizer).");
        return (passCount == totalCount) ? 0 : 1;
    }

    // ── Phase 2: Interactive visualization ──

    Renderer renderer(1280, 720, "RT-PhysicsCore Contact Cache Test");
    if (!renderer.IsValid())
    {
        return 1;
    }

    Scene scene;
    scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
    auto pipeline = std::make_unique<PhysicsPipelineSystem>(
        scene, ResolutionSystem::SolverMode::SequentialImpulses, 8);
    pipeline->GetPhysicsSystem().SetGravity({ 0.0f, -9.81f, 0.0f });
    scene.AddSystem(std::move(pipeline));

    auto setupDemo = [&]()
        {
            for (Entity e : scene.GetEntities())
            {
                scene.DestroyEntity(e);
            }

            // Static inclined plane (15 degrees)
            Entity ground = scene.CreateEntity();
            TransformComponent gtc;
            gtc.position = { 0.0f, 0.0f, 0.0f };
            gtc.rotation = glm::angleAxis(glm::radians(15.0f), glm::vec3(0, 0, 1));
            scene.AddComponent(ground, gtc);
            scene.AddComponent(ground, MakeStaticBody());
            ColliderComponent gcc;
            gcc.shape = ColliderShape::Box;
            gcc.size = { 6.0f, 0.2f, 2.0f };
            scene.AddComponent(ground, gcc);

            // Dynamic sliding box with friction
            Entity box = scene.CreateEntity();
            TransformComponent btc;
            btc.position = { -1.5f, 1.5f, 0.0f };
            btc.rotation = glm::angleAxis(glm::radians(15.0f), glm::vec3(0, 0, 1));
            scene.AddComponent(box, btc);
            scene.AddComponent(box, MakeDynamicBody(5.0, ComputeBoxInertia(5.0, { 0.5, 0.5, 0.5 })));
            ColliderComponent bcc;
            bcc.shape = ColliderShape::Box;
            bcc.size = { 0.5f, 0.5f, 0.5f };
            scene.AddComponent(box, bcc);
        };

    setupDemo();

    auto lastTime = std::chrono::steady_clock::now();
    double cycleElapsed = 0.0;
    double physicsAccum = 0.0;
    constexpr double fixedDt = 1.0 / 60.0;

    while (!renderer.ShouldClose())
    {
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - lastTime).count();
        lastTime = now;
        if (dt > 0.1)
        {
            dt = 0.1;
        }

        cycleElapsed += dt;
        if (cycleElapsed >= 3.0)
        {
            cycleElapsed = 0.0;
            setupDemo();
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

            glm::vec3 color = (rb && rb->invMass > 0.0f) ? glm::vec3(1.0f, 0.4f, 0.2f) : glm::vec3(0.3f, 0.8f, 0.5f);

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

        // Draw contact normal & friction vector cues
        DebugDraw::Line({ 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.2f, 1.0f, 0.2f }, false);

        renderer.FlushDebugDraw();
        renderer.EndFrame();
    }

    return 0;
}