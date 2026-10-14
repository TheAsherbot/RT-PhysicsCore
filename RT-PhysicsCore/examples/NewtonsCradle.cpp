/**
 * @file NewtonsCradle.cpp
 * @brief Momentum and energy conservation in elastic 1D collisions.
 */

#include <memory>
#include <vector>
#include <GLFW/glfw3.h>
#include <imgui/imgui.h>

#include "RT-PhysicsCore/core/Engine.h"
#include "RT-PhysicsCore/core/ecs/core/Scene.h"
#include "RT-PhysicsCore/core/ecs/systems/TransformPropagationSystem.h"
#include "RT-PhysicsCore/core/ecs/components/TransformComponent.h"
#include "RT-PhysicsCore/physics/systems/PhysicsPipelineSystem.h"
#include "RT-PhysicsCore/physics/components/RigidBodyComponent.h"
#include "RT-PhysicsCore/physics/MassProperties.h"
#include "RT-PhysicsCore/physics/components/ColliderComponent.h"
#include "RT-PhysicsCore/physics/components/PhysicsMaterialComponent.h"
#include "RT-PhysicsCore/rendering/Renderer.h"
#include "RT-PhysicsCore/rendering/systems/RenderSystem.h"
#include "RT-PhysicsCore/rendering/components/MeshComponent.h"
#include "RT-PhysicsCore/utils/DebugDraw.h"

int main()
{
    using namespace RT_PhysicsCore;

    Renderer renderer(1280, 720, "Example: Newton's Cradle");
    if (!renderer.IsValid())
    {
        return 1;
    }

    Engine engine(60.0);
    Scene scene;

    scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
    auto pipeline = std::make_unique<PhysicsPipelineSystem>(
        scene, ResolutionSystem::SolverMode::Exact, 4);
    pipeline->GetPhysicsSystem().SetGravity({ 0.0f, 0.0f, 0.0f }); // Zero gravity
    scene.AddSystem(std::move(pipeline));
    scene.AddSystem(std::make_unique<RenderSystem>(scene, renderer));

    std::vector<Entity> balls;
    int currentPullCount = 1;
    bool showFps = false;

    auto setup = [&](int pullCount)
        {
            currentPullCount = pullCount;
            for (Entity e : balls)
            {
                scene.DestroyEntity(e);
            }
            balls.clear();

            for (int i = 0; i < 5; ++i)
            {
                Entity e = scene.CreateEntity();
                TransformComponent tc;

                // If it's one of the pulled balls, move it back on X and give it initial velocity
                if (i < pullCount)
                {
                    tc.position = { -3.0f - static_cast<float>(pullCount - 1 - i) * 1.0f, 0.0f, 0.0f };
                }
                else
                {
                    tc.position = { static_cast<float>(i) * 1.0f, 0.0f, 0.0f };
                }
                tc.scale = { 1.0f, 1.0f, 1.0f };
                scene.AddComponent(e, tc);

                RigidBodyComponent rb = MakeDynamicBody(1.0f, ComputeSphereInertia(1.0f, 0.5f));
                if (i < pullCount)
                {
                    rb.velocity = { 5.0f, 0.0f, 0.0f };
                }
                scene.AddComponent(e, rb);

                ColliderComponent cc;
                cc.shape = ColliderShape::Sphere;
                cc.size = { 0.5f, 0.0f, 0.0f };
                scene.AddComponent(e, cc);

                MeshComponent mc;
                mc.shape = PrimitiveShape::Sphere;
                mc.color = { 0.7f, 0.8f, 0.9f };
                scene.AddComponent(e, mc);

                PhysicsMaterialComponent pm;
                pm.material = MaterialId::BouncyIce; // e=1.0, mu=0
                scene.AddComponent(e, pm);

                balls.push_back(e);
            }
        };
    setup(1);

    engine.SetUpdateCallback([&](double dt)
        {
            scene.SetDeltaTime(dt);
            scene.UpdateSystems();

            auto& input = renderer.GetInput();
            if (input.WasKeyPressed(GLFW_KEY_F3))
            {
                showFps = !showFps;
            }
            if (input.WasKeyPressed(GLFW_KEY_R))
            {
                setup(currentPullCount);
            }
            if (input.WasKeyPressed(GLFW_KEY_1))
            {
                setup(1);
            }
            if (input.WasKeyPressed(GLFW_KEY_2))
            {
                setup(2);
            }
            if (input.WasKeyPressed(GLFW_KEY_3))
            {
                setup(3);
            }

            if (input.WasKeyPressed(GLFW_KEY_ESCAPE) || renderer.ShouldClose())
            {
                engine.RequestExit();
            }
        });

    engine.SetFixedUpdateCallback([&](double dt)
        {
            scene.SetFixedDeltaTime(dt);
            scene.FixedUpdateSystems();

            // Constrain to 1D motion to emulate the suspension cables
            for (Entity e : balls)
            {
                auto* rb = scene.GetComponent<RigidBodyComponent>(e);
                auto* tc = scene.GetComponent<TransformComponent>(e);
                rb->velocity.y = 0.0f;
                rb->velocity.z = 0.0f;
                rb->angularVelocity = { 0.0f, 0.0f, 0.0f };
                tc->position.y = 0.0f;
                tc->position.z = 0.0f;
            }
        });

    engine.SetRenderCallback([&](double /*alpha*/)
        {
            renderer.BeginFrame();
            scene.RenderUpdateSystems();

            DebugDraw::Line({ -10.0f, 0.0f, 0.0f }, { 10.0f, 0.0f, 0.0f }, { 0.3f, 0.3f, 0.3f });

            for (Entity e : balls)
            {
                auto* tc = scene.GetComponent<TransformComponent>(e);
                auto* rb = scene.GetComponent<RigidBodyComponent>(e);
                if (glm::length(rb->velocity) > 0.1f)
                {
                    DebugDraw::Line(tc->position + glm::vec3(0.0f, 0.6f, 0.0f),
                        tc->position + glm::vec3(0.0f, 0.6f, 0.0f) + rb->velocity * 0.2f,
                        { 0.0f, 1.0f, 1.0f }, false);
                }
            }

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

            renderer.FlushDebugDraw();
            renderer.EndFrame();
        });

    engine.Run();
    return 0;
}