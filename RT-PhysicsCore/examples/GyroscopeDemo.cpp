/**
 * @file GyroscopeDemo.cpp
 * @brief Demonstrates the Dzhanibekov effect (Intermediate Axis Theorem).
 */

#include <memory>
#include <GLFW/glfw3.h>
#include <imgui/imgui.h>

#include "RT-PhysicsCore/core/Engine.h"
#include "RT-PhysicsCore/core/ecs/core/Scene.h"
#include "RT-PhysicsCore/core/ecs/systems/TransformPropagationSystem.h"
#include "RT-PhysicsCore/core/ecs/components/TransformComponent.h"
#include "RT-PhysicsCore/physics/systems/PhysicsSystem.h"
#include "RT-PhysicsCore/physics/components/RigidBodyComponent.h"
#include "RT-PhysicsCore/physics/MassProperties.h"
#include "RT-PhysicsCore/rendering/Renderer.h"
#include "RT-PhysicsCore/rendering/systems/RenderSystem.h"
#include "RT-PhysicsCore/rendering/components/MeshComponent.h"
#include "RT-PhysicsCore/utils/DebugDraw.h"

int main()
{
    using namespace RT_PhysicsCore;

    Renderer renderer(1280, 720, "Example: Dzhanibekov Effect");
    if (!renderer.IsValid()) return 1;

    Engine engine(60.0);
    Scene scene;

    scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
    auto phys = std::make_unique<PhysicsSystem>(scene);
    phys->SetGravity({ 0.0f, 0.0f, 0.0f }); // Free space!
    scene.AddSystem(std::move(phys));
    scene.AddSystem(std::make_unique<RenderSystem>(scene, renderer));

    // Common properties for both objects
    glm::vec3 halfExtents = { 1.5f, 0.1f, 0.5f };
    glm::mat3 inertia = ComputeBoxInertia(1.0f, halfExtents);

    // 1. The Tumbler (Intermediate Axis)
    Entity tumbler = scene.CreateEntity();
    TransformComponent ttc; ttc.position = { -3.0f, 0.0f, 0.0f }; ttc.scale = halfExtents * 2.0f;
    scene.AddComponent(tumbler, ttc);
    MeshComponent tmc; tmc.shape = PrimitiveShape::Cube; tmc.color = { 1.0f, 0.5f, 0.0f };
    scene.AddComponent(tumbler, tmc);
    RigidBodyComponent trb = MakeDynamicBody(1.0f, inertia);
    // Spin mostly around Z (intermediate axis) with a tiny perturbation
    trb.angularMomentum = { 0.01f, 0.01f, 3.0f };
    scene.AddComponent(tumbler, trb);

    // 2. The Stable Spinner (Major Axis)
    Entity spinner = scene.CreateEntity();
    TransformComponent stc; stc.position = { 3.0f, 0.0f, 0.0f }; stc.scale = halfExtents * 2.0f;
    scene.AddComponent(spinner, stc);
    MeshComponent smc; smc.shape = PrimitiveShape::Cube; smc.color = { 0.0f, 0.6f, 1.0f };
    scene.AddComponent(spinner, smc);
    RigidBodyComponent srb = MakeDynamicBody(1.0f, inertia);
    // Spin mostly around Y (major axis) with a tiny perturbation
    srb.angularMomentum = { 0.01f, 3.0f, 0.01f };
    scene.AddComponent(spinner, srb);

    bool slowMo = false;

    engine.SetUpdateCallback([&](double dt) {
        scene.SetDeltaTime(dt);
        scene.UpdateSystems();

        if (renderer.GetInput().WasKeyPressed(GLFW_KEY_SPACE)) slowMo = !slowMo;
        if (renderer.ShouldClose()) engine.RequestExit();
        });

    engine.SetFixedUpdateCallback([&](double dt) {
        scene.SetFixedDeltaTime(slowMo ? dt * 0.2 : dt);
        scene.FixedUpdateSystems();
        });

    engine.SetRenderCallback([&](double) {
        renderer.BeginFrame();
        scene.RenderUpdateSystems();

        // Draw vectors for both
        Entity entities[] = { tumbler, spinner };
        for (Entity e : entities)
        {
            auto* tc = scene.GetComponent<TransformComponent>(e);
            auto* rb = scene.GetComponent<RigidBodyComponent>(e);
            glm::mat3 rot = glm::mat3_cast(tc->rotation);

            // Angular Momentum (L) - Magenta (should stay constant!)
            DebugDraw::Line(tc->position, tc->position + rb->angularMomentum * 0.8f, { 1.0f, 0.0f, 1.0f }, false);

            // Angular Velocity (omega) - Yellow (wobbles on tumbler)
            DebugDraw::Line(tc->position, tc->position + rb->angularVelocity * 0.5f, { 1.0f, 1.0f, 0.0f }, false);

            // Body axes
            DebugDraw::Line(tc->position, tc->position + rot[0] * 1.5f, { 1.0f, 0.2f, 0.2f }, false);
            DebugDraw::Line(tc->position, tc->position + rot[1] * 1.5f, { 0.2f, 1.0f, 0.2f }, false);
            DebugDraw::Line(tc->position, tc->position + rot[2] * 1.5f, { 0.2f, 0.2f, 1.0f }, false);
        }

        static float initialETumbler = -1.0f;
        static float initialESpinner = -1.0f;

        auto* trb_ptr = scene.GetComponent<RigidBodyComponent>(tumbler);
        auto* srb_ptr = scene.GetComponent<RigidBodyComponent>(spinner);

        float curETumbler = 0.5f * glm::dot(trb_ptr->angularMomentum, trb_ptr->angularVelocity);
        float curESpinner = 0.5f * glm::dot(srb_ptr->angularMomentum, srb_ptr->angularVelocity);

        if (initialETumbler <= 0.0f && curETumbler > 0.0f) initialETumbler = curETumbler;
        if (initialESpinner <= 0.0f && curESpinner > 0.0f) initialESpinner = curESpinner;

        float driftTumbler = std::abs(curETumbler - initialETumbler) / initialETumbler * 100.0f;
        float driftSpinner = std::abs(curESpinner - initialESpinner) / initialESpinner * 100.0f;

        ImGui::SetNextWindowPos(ImVec2(10, 10));
        ImGui::Begin("Gyroscope Dynamics", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::Text("Space: Toggle Slow-Mo (%s)", slowMo ? "ON" : "OFF");
        ImGui::Separator();

        ImGui::Text("TUMBLER (Intermediate Axis)");
        ImGui::Text("|L|     = %.3f", glm::length(trb_ptr->angularMomentum));
        ImGui::Text("w       = [%.2f, %.2f, %.2f]", trb_ptr->angularVelocity.x, trb_ptr->angularVelocity.y, trb_ptr->angularVelocity.z);
        ImGui::Text("E       = %.4f (drift: %+.3f%%)", curETumbler, driftTumbler);

        ImGui::Separator();

        ImGui::Text("SPINNER (Major Axis)");
        ImGui::Text("|L|     = %.3f", glm::length(srb_ptr->angularMomentum));
        ImGui::Text("w       = [%.2f, %.2f, %.2f]", srb_ptr->angularVelocity.x, srb_ptr->angularVelocity.y, srb_ptr->angularVelocity.z);
        ImGui::Text("E       = %.4f (drift: %+.3f%%)", curESpinner, driftSpinner);
        ImGui::End();

        renderer.FlushDebugDraw();
        renderer.EndFrame();
        });

    engine.Run();
    return 0;
}