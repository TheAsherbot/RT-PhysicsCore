/**
 * @file InclinedPlane.cpp
 * @brief Interactive friction laboratory.
 */

#include <memory>
#include <vector>
#include <cmath>
#include <GLFW/glfw3.h>
#include <imgui/imgui.h>

#include "RT-PhysicsCore/core/Engine.h"
#include "RT-PhysicsCore/core/ecs/core/Scene.h"
#include "RT-PhysicsCore/core/ecs/systems/TransformPropagationSystem.h"
#include "RT-PhysicsCore/core/ecs/components/TransformComponent.h"
#include "RT-PhysicsCore/physics/systems/PhysicsSystem.h"
#include "RT-PhysicsCore/physics/components/RigidBodyComponent.h"
#include "RT-PhysicsCore/physics/MassProperties.h"
#include "RT-PhysicsCore/physics/systems/CollisionSystem.h"
#include "RT-PhysicsCore/physics/components/ColliderComponent.h"
#include "RT-PhysicsCore/physics/systems/ResolutionSystem.h"
#include "RT-PhysicsCore/physics/components/PhysicsMaterialComponent.h"
#include "RT-PhysicsCore/rendering/Renderer.h"
#include "RT-PhysicsCore/rendering/systems/RenderSystem.h"
#include "RT-PhysicsCore/rendering/components/MeshComponent.h"
#include "RT-PhysicsCore/utils/DebugDraw.h"

#include <glm/gtc/quaternion.hpp>

int main()
{
    using namespace RT_PhysicsCore;

    Renderer renderer(1280, 720, "Example: Inclined Plane");
    if (!renderer.IsValid()) return 1;

    Engine engine(60.0);
    Scene scene;

    scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
    scene.AddSystem(std::make_unique<PhysicsSystem>(scene));
    auto colSys = std::make_unique<CollisionSystem>(scene);
    scene.AddSystem(std::make_unique<ResolutionSystem>(scene, *colSys, ResolutionSystem::SolverMode::SequentialImpulses));
    scene.AddSystem(std::move(colSys));
    scene.AddSystem(std::make_unique<RenderSystem>(scene, renderer));

    // Ramp (Static)
    Entity ramp = scene.CreateEntity();
    TransformComponent rtc;
    rtc.position = { 0.0f, 0.0f, 0.0f };
    rtc.scale = { 12.0f, 0.6f, 6.0f };
    scene.AddComponent(ramp, rtc);
    scene.AddComponent(ramp, MakeStaticBody());
    MeshComponent rmc; rmc.shape = PrimitiveShape::Cube; rmc.color = { 0.4f, 0.4f, 0.4f };
    scene.AddComponent(ramp, rmc);
    ColliderComponent rcc; rcc.shape = ColliderShape::Box; rcc.size = { 6.0f, 0.3f, 3.0f };
    scene.AddComponent(ramp, rcc);
    PhysicsMaterialComponent rpm; rpm.material = MaterialId::Rubber; // Use a grippy ramp
    scene.AddComponent(ramp, rpm);

    struct TestObject { Entity e; MaterialId mat; std::string name; float mu_s; float mu_k; };
    std::vector<TestObject> objects;

    auto spawnObjects = [&]() {
        for (auto& obj : objects) scene.DestroyEntity(obj.e);
        objects.clear();

        // 1. Rubber Block
        Entity e1 = scene.CreateEntity();
        TransformComponent t1; t1.position = { -4.0f, 1.0f, -2.0f }; t1.scale = { 0.6f, 0.6f, 0.6f };
        scene.AddComponent(e1, t1);
        scene.AddComponent(e1, MakeDynamicBody(1.0f, ComputeBoxInertia(1.0f, { 0.3f, 0.3f, 0.3f })));
        ColliderComponent c1; c1.shape = ColliderShape::Box; c1.size = { 0.3f, 0.3f, 0.3f };
        scene.AddComponent(e1, c1);
        MeshComponent m1; m1.shape = PrimitiveShape::Cube; m1.color = { 0.8f, 0.2f, 0.2f };
        scene.AddComponent(e1, m1);
        PhysicsMaterialComponent p1; p1.material = MaterialId::Rubber;
        scene.AddComponent(e1, p1);
        objects.push_back({ e1, MaterialId::Rubber, "Rubber Block", 0.8f, 0.6f });

        // 2. Ice Ball
        Entity e2 = scene.CreateEntity();
        TransformComponent t2; t2.position = { 0.0f, 1.0f, -2.0f }; t2.scale = { 0.6f, 0.6f, 0.6f };
        scene.AddComponent(e2, t2);
        scene.AddComponent(e2, MakeDynamicBody(1.0f, ComputeSphereInertia(1.0f, 0.3f)));
        ColliderComponent c2; c2.shape = ColliderShape::Sphere; c2.size = { 0.3f, 0.0f, 0.0f };
        scene.AddComponent(e2, c2);
        MeshComponent m2; m2.shape = PrimitiveShape::Sphere; m2.color = { 0.2f, 0.6f, 0.8f };
        scene.AddComponent(e2, m2);
        PhysicsMaterialComponent p2; p2.material = MaterialId::BouncyIce; // low friction
        scene.AddComponent(e2, p2);
        objects.push_back({ e2, MaterialId::BouncyIce, "Ice Ball", 0.0f, 0.0f });

        // 3. Wood Block
        Entity e3 = scene.CreateEntity();
        TransformComponent t3; t3.position = { 4.0f, 1.0f, -2.0f }; t3.scale = { 0.4f, 0.6f, 0.4f };
        scene.AddComponent(e3, t3);
        scene.AddComponent(e3, MakeDynamicBody(1.0f, ComputeBoxInertia(1.0f, { 0.2f, 0.3f, 0.2f })));
        ColliderComponent c3; c3.shape = ColliderShape::Box; c3.size = { 0.2f, 0.3f, 0.2f };
        scene.AddComponent(e3, c3);
        MeshComponent m3; m3.shape = PrimitiveShape::Cube; m3.color = { 0.5f, 0.8f, 0.2f };
        scene.AddComponent(e3, m3);
        PhysicsMaterialComponent p3; p3.material = MaterialId::Wood;
        scene.AddComponent(e3, p3);
        objects.push_back({ e3, MaterialId::Wood, "Wood Block", 0.5f, 0.4f });
        };
    spawnObjects();

    float rampAngleDeg = 0.0f;

    engine.SetUpdateCallback([&](double dt) {
        scene.SetDeltaTime(dt);
        scene.UpdateSystems();
        if (renderer.GetInput().WasKeyPressed(GLFW_KEY_R)) spawnObjects();
        if (renderer.ShouldClose()) engine.RequestExit();
        });

    engine.SetFixedUpdateCallback([&](double dt) {
        // Update ramp rotation based on UI
        auto* rtc_ptr = scene.GetComponent<TransformComponent>(ramp);
        rtc_ptr->rotation = glm::angleAxis(glm::radians(rampAngleDeg), glm::vec3(0, 0, 1));

        scene.SetFixedDeltaTime(dt);
        scene.FixedUpdateSystems();

        // Recycle objects that fall off
        for (auto& obj : objects)
        {
            auto* tc = scene.GetComponent<TransformComponent>(obj.e);
            if (tc->position.y < -10.0f)
            {
                auto* rb = scene.GetComponent<RigidBodyComponent>(obj.e);
                tc->position.y = 5.0f;
                tc->position.x = tc->position.x < 0 ? -4.0f : (tc->position.x > 0 ? 4.0f : 0.0f);
                tc->position.z = -2.0f;
                rb->velocity = { 0,0,0 };
                rb->angularVelocity = { 0,0,0 };
            }
        }
        });

    engine.SetRenderCallback([&](double) {
        renderer.BeginFrame();
        scene.RenderUpdateSystems();

        ImGui::SetNextWindowPos(ImVec2(10, 10));
        ImGui::Begin("Inclined Plane Laboratory", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::SliderFloat("Ramp Angle", &rampAngleDeg, 0.0f, 60.0f, "%.1f deg");
        ImGui::Separator();

        for (const auto& obj : objects)
        {
            auto* rb = scene.GetComponent<RigidBodyComponent>(obj.e);
            float speed = glm::length(rb->velocity);
            bool isSliding = speed > 0.05f;

            // Note: Since ramp friction is high, actual effective mu is roughly sqrt(mu_obj * mu_ramp)
            // For a rubber ramp (mu=0.8), the effective critical angles are slightly higher than standalone.

            ImGui::Text("%-15s | Status: ", obj.name.c_str());
            ImGui::SameLine();
            if (isSliding) ImGui::TextColored(ImVec4(1.0f, 0.2f, 0.2f, 1.0f), "SLIDING (%.1f m/s)", speed);
            else ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.2f, 1.0f), "HOLDING");
        }
        ImGui::End();

        // Draw bounding boxes indicating state
        for (const auto& obj : objects)
        {
            auto* tc = scene.GetComponent<TransformComponent>(obj.e);
            auto* cc = scene.GetComponent<ColliderComponent>(obj.e);
            auto* rb = scene.GetComponent<RigidBodyComponent>(obj.e);

            glm::vec3 color = glm::length(rb->velocity) > 0.05f ? glm::vec3(1, 0, 0) : glm::vec3(0, 1, 0);
            if (cc->shape == ColliderShape::Box)
            {
                DebugDraw::Line(tc->position, tc->position + glm::vec3(0, 1.5f, 0), color);
            }
            else
            {
                DebugDraw::Sphere(tc->position, cc->size.x + 0.05f, color, 16, false);
            }
        }

        renderer.FlushDebugDraw();
        renderer.EndFrame();
        });

    engine.Run();
    return 0;
}