/**
 * @file IslandBuilderTest.cpp
 * @brief Automated verification and visual demonstration of disjoint-set simulation island partitioning.
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>

#include "RT-PhysicsCore/utils/Log.h"
#include "RT-PhysicsCore/utils/DebugDraw.h"
#include "RT-PhysicsCore/rendering/Renderer.h"

#include "RT-PhysicsCore/core/ecs/core/Scene.h"
#include "RT-PhysicsCore/core/ecs/systems/TransformPropagationSystem.h"
#include "RT-PhysicsCore/core/ecs/components/TransformComponent.h"
#include "RT-PhysicsCore/physics/components/RigidBodyComponent.h"
#include "RT-PhysicsCore/physics/components/ColliderComponent.h"
#include "RT-PhysicsCore/physics/collision/Contact.h"
#include "RT-PhysicsCore/physics/solver/IslandBuilder.h"
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
    RT_LOG_INFO("Island Builder Test harness starting.");

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

    // ── Check 1: Disjoint Stacks Isolation & Static Ground Exclusion ──
    {
        ++totalCount;
        Scene scene;
        IslandBuilder builder;

        Entity ground = scene.CreateEntity();
        scene.AddComponent(ground, MakeStaticBody());

        // Stack 1: Dynamic b1 and b2
        Entity b1 = scene.CreateEntity();
        scene.AddComponent(b1, MakeDynamicBody(1.0, ComputeBoxInertia(1.0, { 0.5, 0.5, 0.5 })));
        Entity b2 = scene.CreateEntity();
        scene.AddComponent(b2, MakeDynamicBody(1.0, ComputeBoxInertia(1.0, { 0.5, 0.5, 0.5 })));

        // Stack 2: Dynamic b3 and b4
        Entity b3 = scene.CreateEntity();
        scene.AddComponent(b3, MakeDynamicBody(1.0, ComputeBoxInertia(1.0, { 0.5, 0.5, 0.5 })));
        Entity b4 = scene.CreateEntity();
        scene.AddComponent(b4, MakeDynamicBody(1.0, ComputeBoxInertia(1.0, { 0.5, 0.5, 0.5 })));

        std::vector<Contact> contacts;
        Contact c1; c1.a = ground; c1.b = b1; contacts.push_back(c1);
        Contact c2; c2.a = b1; c2.b = b2; contacts.push_back(c2);
        Contact c3; c3.a = ground; c3.b = b3; contacts.push_back(c3);
        Contact c4; c4.a = b3; c4.b = b4; contacts.push_back(c4);

        std::vector<SimulationIsland> islands;
        builder.BuildIslands(scene, contacts, islands);

        bool twoIslands = (islands.size() == 2);
        bool groundExcluded = true;
        for (const auto& island : islands)
        {
            if (std::find(island.entities.begin(), island.entities.end(), ground) != island.entities.end())
            {
                groundExcluded = false;
            }
        }

        bool pass = twoIslands && groundExcluded && (islands[0].entities.size() == 2) && (islands[1].entities.size() == 2);
        if (pass)
        {
            ++passCount;
        }

        std::ostringstream oss;
        oss << "islands=" << islands.size() << " (expected 2), groundExcluded=" << groundExcluded;
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "Disjoint Stacks Static Isolation - " << oss.str());
    }

    // ── Check 2: Dynamic Bridge Coupling ──
    {
        ++totalCount;
        Scene scene;
        IslandBuilder builder;

        Entity ground = scene.CreateEntity();
        scene.AddComponent(ground, MakeStaticBody());

        Entity b1 = scene.CreateEntity();
        scene.AddComponent(b1, MakeDynamicBody(1.0, ComputeBoxInertia(1.0, { 0.5, 0.5, 0.5 })));
        Entity b2 = scene.CreateEntity();
        scene.AddComponent(b2, MakeDynamicBody(1.0, ComputeBoxInertia(1.0, { 0.5, 0.5, 0.5 })));
        Entity b3 = scene.CreateEntity();
        scene.AddComponent(b3, MakeDynamicBody(1.0, ComputeBoxInertia(1.0, { 0.5, 0.5, 0.5 })));
        Entity b4 = scene.CreateEntity();
        scene.AddComponent(b4, MakeDynamicBody(1.0, ComputeBoxInertia(1.0, { 0.5, 0.5, 0.5 })));

        // Spanning dynamic plank connecting top boxes b2 and b4
        Entity plank = scene.CreateEntity();
        scene.AddComponent(plank, MakeDynamicBody(2.0, ComputeBoxInertia(2.0, { 2.0, 0.2, 0.5 })));

        std::vector<Contact> contacts;
        Contact c1; c1.a = ground; c1.b = b1; contacts.push_back(c1);
        Contact c2; c2.a = b1; c2.b = b2; contacts.push_back(c2);
        Contact c3; c3.a = ground; c3.b = b3; contacts.push_back(c3);
        Contact c4; c4.a = b3; c4.b = b4; contacts.push_back(c4);
        Contact cBridge1; cBridge1.a = b2; cBridge1.b = plank; contacts.push_back(cBridge1);
        Contact cBridge2; cBridge2.a = b4; cBridge2.b = plank; contacts.push_back(cBridge2);

        std::vector<SimulationIsland> islands;
        builder.BuildIslands(scene, contacts, islands);

        bool singleIsland = (islands.size() == 1);
        bool hasAllEntities = singleIsland && (islands[0].entities.size() == 5);
        bool hasAllContacts = singleIsland && (islands[0].contacts.size() == 6);

        bool pass = singleIsland && hasAllEntities && hasAllContacts;
        if (pass)
        {
            ++passCount;
        }

        std::ostringstream oss;
        oss << "islands=" << islands.size() << " (expected 1), entities="
            << (singleIsland ? islands[0].entities.size() : 0) << " (expected 5)";
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "Dynamic Bridge Coupling - " << oss.str());
    }

    // ── Check 3: Free-Floating Dynamic Body Exclusion ──
    {
        ++totalCount;
        Scene scene;
        IslandBuilder builder;

        Entity bDynamic = scene.CreateEntity();
        scene.AddComponent(bDynamic, MakeDynamicBody(1.0, ComputeBoxInertia(1.0, { 0.5, 0.5, 0.5 })));

        // Free-floating body has no active contact constraints
        std::vector<Contact> contacts;
        std::vector<SimulationIsland> islands;
        builder.BuildIslands(scene, contacts, islands);

        bool pass = islands.empty();
        if (pass)
        {
            ++passCount;
        }

        std::ostringstream oss;
        oss << "islands=" << islands.size() << " (expected 0 for unconstrained dynamic body)";
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "Free-Floating Unconstrained Exclusion - " << oss.str());
    }

    // ── Check 4: Deterministic Sorted Entity Ordering ──
    {
        ++totalCount;
        Scene scene;
        IslandBuilder builder;

        // Disperse IDs by creating and destroying temporary entities
        Entity dummy1 = scene.CreateEntity();
        Entity bA = scene.CreateEntity();
        Entity dummy2 = scene.CreateEntity();
        Entity bB = scene.CreateEntity();
        Entity dummy3 = scene.CreateEntity();
        Entity bC = scene.CreateEntity();

        scene.DestroyEntity(dummy1);
        scene.DestroyEntity(dummy2);
        scene.DestroyEntity(dummy3);

        scene.AddComponent(bA, MakeDynamicBody(1.0, ComputeBoxInertia(1.0, { 0.5, 0.5, 0.5 })));
        scene.AddComponent(bB, MakeDynamicBody(1.0, ComputeBoxInertia(1.0, { 0.5, 0.5, 0.5 })));
        scene.AddComponent(bC, MakeDynamicBody(1.0, ComputeBoxInertia(1.0, { 0.5, 0.5, 0.5 })));

        std::vector<Contact> contacts;
        Contact c1; c1.a = bC; c1.b = bA; contacts.push_back(c1);
        Contact c2; c2.a = bB; c2.b = bC; contacts.push_back(c2);

        std::vector<SimulationIsland> islands;
        builder.BuildIslands(scene, contacts, islands);

        bool sorted = false;
        if (islands.size() == 1 && islands[0].entities.size() == 3)
        {
            sorted = std::is_sorted(islands[0].entities.begin(), islands[0].entities.end());
        }

        bool pass = sorted;
        if (pass)
        {
            ++passCount;
        }

        std::ostringstream oss;
        oss << "sorted=" << sorted << " order=[";
        if (!islands.empty())
        {
            for (size_t i = 0; i < islands[0].entities.size(); ++i)
            {
                oss << islands[0].entities[i] << (i + 1 < islands[0].entities.size() ? "," : "");
            }
        }
        oss << "]";
        RT_LOG_INFO((pass ? "[PASS] " : "[FAIL] ") << "Deterministic Entity Ordering - " << oss.str());
    }

    RT_LOG_INFO(passCount << "/" << totalCount << " automated checks passed.");

    if (!enableGui)
    {
        RT_LOG_INFO("Headless execution complete (pass --gui to open interactive visualizer).");
        return (passCount == totalCount) ? 0 : 1;
    }

    // ── Phase 2: Interactive visualization ──

    Renderer renderer(1280, 720, "RT-PhysicsCore Island Builder Test");
    if (!renderer.IsValid())
    {
        return 1;
    }

    Scene scene;
    scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
    auto pipeline = std::make_unique<PhysicsPipelineSystem>(
        scene, ResolutionSystem::SolverMode::SequentialImpulses, 10);
    pipeline->GetPhysicsSystem().SetGravity({ 0.0f, -9.81f, 0.0f });
    scene.AddSystem(std::move(pipeline));

    auto setupDemo = [&]()
        {
            for (Entity e : scene.GetEntities())
            {
                scene.DestroyEntity(e);
            }

            // Static floor
            Entity floor = scene.CreateEntity();
            TransformComponent ftc;
            ftc.position = { 0.0f, -0.5f, 0.0f };
            scene.AddComponent(floor, ftc);
            scene.AddComponent(floor, MakeStaticBody());
            ColliderComponent fcc;
            fcc.shape = ColliderShape::Box;
            fcc.size = { 10.0f, 0.5f, 4.0f };
            scene.AddComponent(floor, fcc);

            // Stack 1 (Left - Island 0)
            for (int i = 0; i < 3; ++i)
            {
                Entity e = scene.CreateEntity();
                TransformComponent tc;
                tc.position = { -3.0f, 0.55f + i * 1.05f, 0.0f };
                scene.AddComponent(e, tc);
                scene.AddComponent(e, MakeDynamicBody(1.0, ComputeBoxInertia(1.0, { 0.5, 0.5, 0.5 })));
                ColliderComponent cc;
                cc.shape = ColliderShape::Box;
                cc.size = { 0.5f, 0.5f, 0.5f };
                scene.AddComponent(e, cc);
            }

            // Stack 2 (Right - Island 1)
            for (int i = 0; i < 3; ++i)
            {
                Entity e = scene.CreateEntity();
                TransformComponent tc;
                tc.position = { 3.0f, 0.55f + i * 1.05f, 0.0f };
                scene.AddComponent(e, tc);
                scene.AddComponent(e, MakeDynamicBody(1.0, ComputeBoxInertia(1.0, { 0.5, 0.5, 0.5 })));
                ColliderComponent cc;
                cc.shape = ColliderShape::Box;
                cc.size = { 0.5f, 0.5f, 0.5f };
                scene.AddComponent(e, cc);
            }
        };

    setupDemo();

    auto lastTime = std::chrono::steady_clock::now();
    double cycleElapsed = 0.0;
    double physicsAccum = 0.0;
    constexpr double fixedDt = 1.0 / 60.0;
    bool plankDropped = false;

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
        if (cycleElapsed >= 1.5 && !plankDropped)
        {
            plankDropped = true;
            // Drop bridging plank spanning both stacks
            Entity plank = scene.CreateEntity();
            TransformComponent ptc;
            ptc.position = { 0.0f, 4.5f, 0.0f };
            scene.AddComponent(plank, ptc);
            scene.AddComponent(plank, MakeDynamicBody(3.0, ComputeBoxInertia(3.0, { 3.5, 0.15, 0.5 })));
            ColliderComponent pcc;
            pcc.shape = ColliderShape::Box;
            pcc.size = { 3.5f, 0.15f, 0.5f };
            scene.AddComponent(plank, pcc);
        }

        if (cycleElapsed >= 4.0)
        {
            cycleElapsed = 0.0;
            plankDropped = false;
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

        // Color code entities: Left stack = Orange, Right stack = Cyan, Plank/Merged = Magenta
        for (Entity e : scene.Query<TransformComponent, ColliderComponent>())
        {
            auto* tc = scene.GetComponent<TransformComponent>(e);
            auto* cc = scene.GetComponent<ColliderComponent>(e);
            auto* rb = scene.GetComponent<RigidBodyComponent>(e);

            glm::vec3 color{ 0.3f, 0.8f, 0.5f }; // Floor green
            if (rb && rb->invMass > 0.0f)
            {
                if (plankDropped && cycleElapsed > 1.9)
                {
                    color = glm::vec3(0.9f, 0.2f, 0.9f); // Merged island (Magenta)
                }
                else if (tc->position.x < -1.0f)
                {
                    color = glm::vec3(1.0f, 0.5f, 0.1f); // Island 0 (Orange)
                }
                else if (tc->position.x > 1.0f)
                {
                    color = glm::vec3(0.1f, 0.7f, 1.0f); // Island 1 (Cyan)
                }
                else
                {
                    color = glm::vec3(0.9f, 0.8f, 0.2f); // Plank falling (Yellow)
                }
            }

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

        // Draw island coupling guide lines
        DebugDraw::Line({ -3.0f, 0.0f, 0.0f }, { -3.0f, 4.0f, 0.0f }, { 1.0f, 0.5f, 0.1f }, false);
        DebugDraw::Line({ 3.0f, 0.0f, 0.0f }, { 3.0f, 4.0f, 0.0f }, { 0.1f, 0.7f, 1.0f }, false);

        renderer.FlushDebugDraw();
        renderer.EndFrame();
    }

    return 0;
}