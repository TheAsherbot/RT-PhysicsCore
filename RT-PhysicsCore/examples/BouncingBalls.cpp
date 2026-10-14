/**
 * @file BouncingBalls.cpp
 * @brief Restitution and material interaction demo with robust apex bounce tracking.
 */

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
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

namespace
{
    using namespace RT_PhysicsCore;

    /**
     * @struct BallData
     * @brief Holds visualization and material properties for demo sphere bodies.
     */
    struct BallData
    {
        MaterialId mat;
        glm::vec3 color;
        std::string name;
    };
}

int main()
{
    using namespace RT_PhysicsCore;

    Renderer renderer(1280, 720, "Example: Bouncing Balls");
    if (!renderer.IsValid())
    {
        return 1;
    }

    Engine engine(60.0);
    Scene scene;

    scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
    scene.AddSystem(std::make_unique<PhysicsPipelineSystem>(
        scene, ResolutionSystem::SolverMode::SequentialImpulses, 4));
    scene.AddSystem(std::make_unique<RenderSystem>(scene, renderer));

    // Ground
    Entity ground = scene.CreateEntity();
    TransformComponent gtc;
    gtc.position = { 0.0f, -0.5f, 0.0f };
    gtc.scale = { 20.0f, 1.0f, 10.0f };
    scene.AddComponent(ground, gtc);
    scene.AddComponent(ground, MakeStaticBody());
    MeshComponent gm;
    gm.shape = PrimitiveShape::Cube;
    gm.color = { 0.3f, 0.3f, 0.3f };
    scene.AddComponent(ground, gm);
    ColliderComponent gcc;
    gcc.shape = ColliderShape::Box;
    gcc.size = { 10.0f, 0.5f, 5.0f };
    scene.AddComponent(ground, gcc);

    std::vector<BallData> balls = {
        { MaterialId::Clay, { 0.6f, 0.1f, 0.1f }, "Clay" },
        { MaterialId::Wood, { 0.8f, 0.5f, 0.1f }, "Wood" },
        { MaterialId::Rubber, { 1.0f, 0.9f, 0.1f }, "Rubber" },
        { MaterialId::HardRubber, { 0.5f, 1.0f, 0.2f }, "HardRubber" },
        { MaterialId::SuperBall, { 0.1f, 1.0f, 0.3f }, "SuperBall" }
    };

    std::vector<Entity> ballEntities;
    std::vector<std::vector<float>> peakHeights(5);
    std::vector<glm::vec3> peakMarkers;

    // Ascent state and peak tracking per ball
    std::vector<bool> wasAscending(5, false);
    std::vector<float> maxAscentHeight(5, 0.0f);

    bool showFps = false;

    auto spawnBalls = [&]()
        {
            for (Entity e : ballEntities)
            {
                scene.DestroyEntity(e);
            }
            ballEntities.clear();
            peakHeights.assign(5, std::vector<float>());
            peakMarkers.clear();
            wasAscending.assign(5, false);
            maxAscentHeight.assign(5, 0.0f);

            for (size_t i = 0; i < balls.size(); ++i)
            {
                Entity e = scene.CreateEntity();
                TransformComponent tc;
                tc.position = { -6.0f + static_cast<float>(i) * 3.0f, 8.0f, 0.0f };
                tc.scale = { 0.8f, 0.8f, 0.8f };
                scene.AddComponent(e, tc);

                scene.AddComponent(e, MakeDynamicBody(1.0f, ComputeSphereInertia(1.0f, 0.4f)));
                ColliderComponent cc;
                cc.shape = ColliderShape::Sphere;
                cc.size = { 0.4f, 0.0f, 0.0f };
                scene.AddComponent(e, cc);

                MeshComponent mc;
                mc.shape = PrimitiveShape::Sphere;
                mc.color = balls[i].color;
                scene.AddComponent(e, mc);

                PhysicsMaterialComponent pm;
                pm.material = balls[i].mat;
                scene.AddComponent(e, pm);

                ballEntities.push_back(e);
            }
        };
    spawnBalls();

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
                spawnBalls();
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

            // Track peaks via velocity sign reversal and ascent tracking
            for (size_t i = 0; i < ballEntities.size(); ++i)
            {
                auto* tc = scene.GetComponent<TransformComponent>(ballEntities[i]);
                auto* rb = scene.GetComponent<RigidBodyComponent>(ballEntities[i]);

                if (!tc || !rb)
                {
                    continue;
                }

                float vy = static_cast<float>(rb->velocity.y);
                float y = static_cast<float>(tc->position.y);

                // Rebounding upward: track the ascent
                if (vy > 0.05f)
                {
                    wasAscending[i] = true;
                    if (y > maxAscentHeight[i])
                    {
                        maxAscentHeight[i] = y;
                    }
                }
                // Reached apex: velocity has transitioned from positive to negative
                else if (wasAscending[i] && vy <= 0.0f)
                {
                    wasAscending[i] = false;
                    float peakY = std::max(maxAscentHeight[i], y);

                    // Ensure this is an actual rebound above resting contact height (radius = 0.4m)
                    if (peakY > 0.45f)
                    {
                        // Prevent duplicates if velocity lingers near zero for consecutive steps
                        if (peakHeights[i].empty() || std::abs(peakHeights[i].back() - peakY) > 0.05f)
                        {
                            peakHeights[i].push_back(peakY);
                            peakMarkers.push_back({ tc->position.x, peakY, tc->position.z });
                        }
                    }

                    maxAscentHeight[i] = 0.0f;
                }
            }
        });

    engine.SetRenderCallback([&](double /*alpha*/)
        {
            renderer.BeginFrame();
            scene.RenderUpdateSystems();

            for (const auto& marker : peakMarkers)
            {
                DebugDraw::Sphere(marker, 0.1f, { 1.0f, 1.0f, 1.0f }, 8, false);
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

            ImGui::SetNextWindowPos(ImVec2(10.0f, showFps ? 45.0f : 10.0f), ImGuiCond_Always);
            ImGui::Begin("Material Bounciness", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
            ImGui::Text("Press 'R' to reset | 'F3' to toggle FPS");
            ImGui::Separator();
            for (size_t i = 0; i < balls.size(); ++i)
            {
                ImGui::Text("%-10s | ", balls[i].name.c_str());
                for (float h : peakHeights[i])
                {
                    ImGui::SameLine();
                    ImGui::Text("%.2fm | ", h);
                }
            }
            ImGui::End();

            renderer.FlushDebugDraw();
            renderer.EndFrame();
        });

    engine.Run();
    return 0;
}