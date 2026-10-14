/**
 * @file ShapeSoup.cpp
 * @brief Stress test in an enclosed arena.
 */

#include <memory>
#include <random>
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

int main()
{
    using namespace RT_PhysicsCore;

    Renderer renderer(1280, 720, "Example: Shape Soup");
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

    // Arena walls
    auto makeWall = [&](glm::vec3 pos, glm::vec3 extents)
        {
            Entity w = scene.CreateEntity();
            TransformComponent tc;
            tc.position = pos;
            tc.scale = extents * 2.0f;
            scene.AddComponent(w, tc);
            scene.AddComponent(w, MakeStaticBody());
            MeshComponent mc;
            mc.shape = PrimitiveShape::Cube;
            mc.color = { 0.2f, 0.2f, 0.25f };
            scene.AddComponent(w, mc);
            ColliderComponent cc;
            cc.shape = ColliderShape::Box;
            cc.size = extents;
            scene.AddComponent(w, cc);
            PhysicsMaterialComponent pm;
            pm.material = MaterialId::Rubber;
            scene.AddComponent(w, pm);
        };

    makeWall({ 0.0f, -0.5f, 0.0f }, { 8.0f, 0.5f, 8.0f });  // floor
    makeWall({ 0.0f, 4.0f, -8.5f }, { 8.0f, 4.0f, 0.5f });  // back
    makeWall({ 0.0f, 4.0f, 8.5f }, { 8.0f, 4.0f, 0.5f });   // front
    makeWall({ -8.5f, 4.0f, 0.0f }, { 0.5f, 4.0f, 8.0f });  // left
    makeWall({ 8.5f, 4.0f, 0.0f }, { 0.5f, 4.0f, 8.0f });   // right

    std::vector<Entity> soupEntities;

    auto spawnSoup = [&]()
        {
            for (Entity e : soupEntities)
            {
                scene.DestroyEntity(e);
            }
            soupEntities.clear();

            std::mt19937 rng(1337);
            std::uniform_real_distribution<float> posDist(-6.0f, 6.0f);
            std::uniform_real_distribution<float> velDist(-5.0f, 5.0f);
            std::uniform_real_distribution<float> sizeDist(0.3f, 0.6f);
            std::uniform_real_distribution<float> colDist(0.3f, 1.0f);

            for (int i = 0; i < 150; ++i)
            {
                Entity e = scene.CreateEntity();
                TransformComponent tc;
                tc.position = { posDist(rng), 2.0f + static_cast<float>(i) * 0.1f, posDist(rng) };
                float s = sizeDist(rng);
                tc.scale = { s * 2.0f, s * 2.0f, s * 2.0f };
                scene.AddComponent(e, tc);

                bool isBox = posDist(rng) > 0.0f;
                MeshComponent mc;
                mc.shape = isBox ? PrimitiveShape::Cube : PrimitiveShape::Sphere;
                mc.color = { colDist(rng), colDist(rng), colDist(rng) };
                scene.AddComponent(e, mc);

                float mass = s * s * s * 10.0f;
                RigidBodyComponent rb = MakeDynamicBody(mass,
                    isBox ? ComputeBoxInertia(mass, glm::vec3(s)) : ComputeSphereInertia(mass, s));
                rb.velocity = { velDist(rng), velDist(rng), velDist(rng) };
                scene.AddComponent(e, rb);

                ColliderComponent cc;
                cc.shape = isBox ? ColliderShape::Box : ColliderShape::Sphere;
                cc.size = { s, s, s };
                scene.AddComponent(e, cc);

                PhysicsMaterialComponent pm;
                pm.material = MaterialId::HardRubber;
                scene.AddComponent(e, pm);

                soupEntities.push_back(e);
            }
        };

    spawnSoup();

    bool showFps = false;

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
                spawnSoup();
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
        });

    engine.SetRenderCallback([&](double /*alpha*/)
        {
            renderer.BeginFrame();
            scene.RenderUpdateSystems();

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

            renderer.EndFrame();
        });

    engine.Run();
    return 0;
}