/**
 * @file BouncingBalls.cpp
 * @brief Restitution and material interaction demo.
 */

#include <memory>
#include <vector>
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

int main()
{
    using namespace RT_PhysicsCore;

    Renderer renderer(1280, 720, "Example: Bouncing Balls");
    if (!renderer.IsValid()) return 1;

    Engine engine(60.0);
    Scene scene;

    scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
    scene.AddSystem(std::make_unique<PhysicsSystem>(scene));
    auto colSys = std::make_unique<CollisionSystem>(scene);
    scene.AddSystem(std::make_unique<ResolutionSystem>(scene, *colSys, ResolutionSystem::SolverMode::SequentialImpulses));
    scene.AddSystem(std::move(colSys));
    scene.AddSystem(std::make_unique<RenderSystem>(scene, renderer));

    // Ground
    Entity ground = scene.CreateEntity();
    TransformComponent gtc; gtc.position = { 0.0f, -0.5f, 0.0f }; gtc.scale = { 20.0f, 1.0f, 10.0f };
    scene.AddComponent(ground, gtc);
    scene.AddComponent(ground, MakeStaticBody());
    MeshComponent gm; gm.shape = PrimitiveShape::Cube; gm.color = { 0.3f, 0.3f, 0.3f };
    scene.AddComponent(ground, gm);
    ColliderComponent gcc; gcc.shape = ColliderShape::Box; gcc.size = { 10.0f, 0.5f, 5.0f };
    scene.AddComponent(ground, gcc);

    struct BallData { MaterialId mat; glm::vec3 color; std::string name; };
    std::vector<BallData> balls = {
        {MaterialId::Clay, {0.6f, 0.1f, 0.1f}, "Clay"},
        {MaterialId::Wood, {0.8f, 0.5f, 0.1f}, "Wood"},
        {MaterialId::Rubber, {1.0f, 0.9f, 0.1f}, "Rubber"},
        {MaterialId::HardRubber, {0.5f, 1.0f, 0.2f}, "HardRubber"},
        {MaterialId::SuperBall, {0.1f, 1.0f, 0.3f}, "SuperBall"}
    };

    std::vector<Entity> ballEntities;
    std::vector<std::vector<float>> peakHeights(5);
    std::vector<glm::vec3> peakMarkers;

    auto spawnBalls = [&]() {
        for (Entity e : ballEntities) scene.DestroyEntity(e);
        ballEntities.clear();
        peakHeights.assign(5, std::vector<float>());
        peakMarkers.clear();

        for (size_t i = 0; i < balls.size(); ++i)
        {
            Entity e = scene.CreateEntity();
            TransformComponent tc;
            tc.position = { -6.0f + i * 3.0f, 8.0f, 0.0f };
            tc.scale = { 0.8f, 0.8f, 0.8f };
            scene.AddComponent(e, tc);

            scene.AddComponent(e, MakeDynamicBody(1.0f, ComputeSphereInertia(1.0f, 0.4f)));
            ColliderComponent cc; cc.shape = ColliderShape::Sphere; cc.size = { 0.4f, 0, 0 };
            scene.AddComponent(e, cc);

            MeshComponent mc; mc.shape = PrimitiveShape::Sphere; mc.color = balls[i].color;
            scene.AddComponent(e, mc);

            PhysicsMaterialComponent pm; pm.material = balls[i].mat;
            scene.AddComponent(e, pm);

            ballEntities.push_back(e);
        }
        };
    spawnBalls();

    engine.SetUpdateCallback([&](double dt) {
        scene.SetDeltaTime(dt);
        scene.UpdateSystems();
        if (renderer.GetInput().WasKeyPressed(GLFW_KEY_R)) spawnBalls();
        if (renderer.ShouldClose()) engine.RequestExit();
        });

    engine.SetFixedUpdateCallback([&](double dt) {
        scene.SetFixedDeltaTime(dt);
        scene.FixedUpdateSystems();

        // Track peaks
        for (size_t i = 0; i < ballEntities.size(); ++i)
        {
            auto* tc = scene.GetComponent<TransformComponent>(ballEntities[i]);
            auto* rb = scene.GetComponent<RigidBodyComponent>(ballEntities[i]);

            if (rb->velocity.y <= 0.0f && rb->velocity.y > -0.1f && tc->position.y > 0.5f)
            {
                // Found an apex
                if (peakHeights[i].empty() || std::abs(peakHeights[i].back() - tc->position.y) > 0.1f)
                {
                    peakHeights[i].push_back(tc->position.y);
                    peakMarkers.push_back(tc->position);
                }
            }
        }
        });

    engine.SetRenderCallback([&](double) {
        renderer.BeginFrame();
        scene.RenderUpdateSystems();

        for (const auto& marker : peakMarkers)
        {
            DebugDraw::Sphere(marker, 0.1f, { 1.0f, 1.0f, 1.0f }, 8, false);
        }

        ImGui::SetNextWindowPos(ImVec2(10, 10));
        ImGui::Begin("Material Bounciness", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
        ImGui::Text("Press 'R' to reset");
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