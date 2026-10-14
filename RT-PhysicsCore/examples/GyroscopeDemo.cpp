/**
 * @file GyroscopeDemo.cpp
 * @brief Demonstrates the Dzhanibekov effect (Intermediate Axis Theorem).
 */

#include <cmath>
#include <memory>
#include <GLFW/glfw3.h>
#include <imgui/imgui.h>

#include "RT-PhysicsCore/core/Engine.h"
#include "RT-PhysicsCore/core/ecs/core/Scene.h"
#include "RT-PhysicsCore/core/ecs/systems/TransformPropagationSystem.h"
#include "RT-PhysicsCore/core/ecs/components/TransformComponent.h"
#include "RT-PhysicsCore/physics/systems/PhysicsPipelineSystem.h"
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
    if (!renderer.IsValid())
    {
        return 1;
    }

    Engine engine(60.0);
    Scene scene;

    scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
    auto pipeline = std::make_unique<PhysicsPipelineSystem>(scene);
    pipeline->GetPhysicsSystem().SetGravity({ 0.0f, 0.0f, 0.0f }); // Free space!
    scene.AddSystem(std::move(pipeline));
    scene.AddSystem(std::make_unique<RenderSystem>(scene, renderer));

    // Common properties for both objects
    glm::vec3 halfExtents = { 1.5f, 0.1f, 0.5f };
    glm::mat3 inertia = ComputeBoxInertia(1.0f, halfExtents);

    // 1. The Tumbler (Intermediate Axis)
    Entity tumbler = scene.CreateEntity();
    TransformComponent ttc;
    ttc.position = { -3.0f, 0.0f, 0.0f };
    ttc.scale = halfExtents * 2.0f;
    scene.AddComponent(tumbler, ttc);
    MeshComponent tmc;
    tmc.shape = PrimitiveShape::Cube;
    tmc.color = { 1.0f, 0.5f, 0.0f };
    scene.AddComponent(tumbler, tmc);
    RigidBodyComponent trb = MakeDynamicBody(1.0f, inertia);
    // Spin mostly around Z (intermediate axis) with a tiny perturbation
    trb.angularMomentum = { 0.01f, 0.01f, 3.0f };
    scene.AddComponent(tumbler, trb);

    // 2. The Stable Spinner (Major Axis)
    Entity spinner = scene.CreateEntity();
    TransformComponent stc;
    stc.position = { 3.0f, 0.0f, 0.0f };
    stc.scale = halfExtents * 2.0f;
    scene.AddComponent(spinner, stc);
    MeshComponent smc;
    smc.shape = PrimitiveShape::Cube;
    smc.color = { 0.0f, 0.6f, 1.0f };
    scene.AddComponent(spinner, smc);
    RigidBodyComponent srb = MakeDynamicBody(1.0f, inertia);
    // Spin mostly around Y (major axis) with a tiny perturbation
    srb.angularMomentum = { 0.01f, 3.0f, 0.01f };
    scene.AddComponent(spinner, srb);

    bool slowMo = false;
    bool showFps = false;
    float initialETumbler = -1.0f;
    float initialESpinner = -1.0f;

    auto resetSimulation = [&]()
        {
            auto* curTtc = scene.GetComponent<TransformComponent>(tumbler);
            auto* curTrb = scene.GetComponent<RigidBodyComponent>(tumbler);
            if (curTtc && curTrb)
            {
                curTtc->position = { -3.0f, 0.0f, 0.0f };
                curTtc->rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
                *curTrb = MakeDynamicBody(1.0f, inertia);
                curTrb->angularMomentum = { 0.01f, 0.01f, 3.0f };
            }

            auto* curStc = scene.GetComponent<TransformComponent>(spinner);
            auto* curSrb = scene.GetComponent<RigidBodyComponent>(spinner);
            if (curStc && curSrb)
            {
                curStc->position = { 3.0f, 0.0f, 0.0f };
                curStc->rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
                *curSrb = MakeDynamicBody(1.0f, inertia);
                curSrb->angularMomentum = { 0.01f, 3.0f, 0.01f };
            }

            initialETumbler = -1.0f;
            initialESpinner = -1.0f;
        };

    engine.SetUpdateCallback([&](double dt)
        {
            scene.SetDeltaTime(dt);
            scene.UpdateSystems();

            auto& input = renderer.GetInput();
            if (input.WasKeyPressed(GLFW_KEY_F3))
            {
                showFps = !showFps;
            }
            if (input.WasKeyPressed(GLFW_KEY_SPACE))
            {
                slowMo = !slowMo;
            }
            if (input.WasKeyPressed(GLFW_KEY_R))
            {
                resetSimulation();
            }
            if (input.WasKeyPressed(GLFW_KEY_ESCAPE) || renderer.ShouldClose())
            {
                engine.RequestExit();
            }
        });

    engine.SetFixedUpdateCallback([&](double dt)
        {
            scene.SetFixedDeltaTime(slowMo ? dt * 0.2 : dt);
            scene.FixedUpdateSystems();
        });

    engine.SetRenderCallback([&](double /*alpha*/)
        {
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

            auto* trbPtr = scene.GetComponent<RigidBodyComponent>(tumbler);
            auto* srbPtr = scene.GetComponent<RigidBodyComponent>(spinner);

            float curETumbler = 0.5f * glm::dot(trbPtr->angularMomentum, trbPtr->angularVelocity);
            float curESpinner = 0.5f * glm::dot(srbPtr->angularMomentum, srbPtr->angularVelocity);

            if (initialETumbler <= 0.0f && curETumbler > 0.0f)
            {
                initialETumbler = curETumbler;
            }
            if (initialESpinner <= 0.0f && curESpinner > 0.0f)
            {
                initialESpinner = curESpinner;
            }

            float driftTumbler = (initialETumbler > 0.0f) ? std::abs(curETumbler - initialETumbler) / initialETumbler * 100.0f : 0.0f;
            float driftSpinner = (initialESpinner > 0.0f) ? std::abs(curESpinner - initialESpinner) / initialESpinner * 100.0f : 0.0f;

            if (showFps)
            {
                ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f), ImGuiCond_Always);
                ImGui::SetNextWindowBgAlpha(0.65f);
                ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration |
                    ImGuiWindowFlags_AlwaysAutoResize |
                    ImGuiWindowFlags_NoSavedSettings |
                    ImGuiWindowFlags_NoFocusOnAppearing |
                    ImGuiWindowFlags_NoNav |
                    ImGuiWindowFlags_NoMove;
                if (ImGui::Begin("##FPSOverlay", nullptr, flags))
                {
                    ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.4f, 1.0f), "FPS: %.1f (%.2f ms)",
                        ImGui::GetIO().Framerate, 1000.0f / ImGui::GetIO().Framerate);
                }
                ImGui::End();
            }

            ImGui::SetNextWindowPos(ImVec2(10.0f, showFps ? 45.0f : 10.0f), ImGuiCond_Always);
            ImGui::Begin("Gyroscope Dynamics", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
            ImGui::Text("Space: Toggle Slow-Mo (%s) | R: Reset", slowMo ? "ON" : "OFF");
            ImGui::Separator();

            ImGui::Text("TUMBLER (Intermediate Axis)");
            ImGui::Text("|L|     = %.3f", glm::length(trbPtr->angularMomentum));
            ImGui::Text("w       = [%.2f, %.2f, %.2f]", trbPtr->angularVelocity.x, trbPtr->angularVelocity.y, trbPtr->angularVelocity.z);
            ImGui::Text("E       = %.4f (drift: %+.3f%%)", curETumbler, driftTumbler);

            ImGui::Separator();

            ImGui::Text("SPINNER (Major Axis)");
            ImGui::Text("|L|     = %.3f", glm::length(srbPtr->angularMomentum));
            ImGui::Text("w       = [%.2f, %.2f, %.2f]", srbPtr->angularVelocity.x, srbPtr->angularVelocity.y, srbPtr->angularVelocity.z);
            ImGui::Text("E       = %.4f (drift: %+.3f%%)", curESpinner, driftSpinner);
            ImGui::End();

            renderer.FlushDebugDraw();
            renderer.EndFrame();
        });

    engine.Run();
    return 0;
}