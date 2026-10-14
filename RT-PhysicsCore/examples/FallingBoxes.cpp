/**
 * @file FallingBoxes.cpp
 * @brief Continuous rigid body stress test with lifetime recycling.
 */

#include <memory>
#include <random>
#include <vector>
#include <GLFW/glfw3.h>
#include <imgui/imgui.h>

#include "RT-PhysicsCore/core/Engine.h"
#include "RT-PhysicsCore/utils/Log.h"
#include "RT-PhysicsCore/utils/DebugDraw.h"

#include "RT-PhysicsCore/core/ecs/core/Scene.h"
#include "RT-PhysicsCore/core/ecs/systems/TransformPropagationSystem.h"
#include "RT-PhysicsCore/core/ecs/components/TransformComponent.h"
#include "RT-PhysicsCore/physics/systems/PhysicsPipelineSystem.h"
#include "RT-PhysicsCore/physics/components/RigidBodyComponent.h"
#include "RT-PhysicsCore/physics/MassProperties.h"
#include "RT-PhysicsCore/physics/components/ColliderComponent.h"
#include "RT-PhysicsCore/rendering/Renderer.h"
#include "RT-PhysicsCore/rendering/systems/RenderSystem.h"
#include "RT-PhysicsCore/rendering/components/MeshComponent.h"

#include <glm/gtc/quaternion.hpp>

namespace
{
    using namespace RT_PhysicsCore;

    /**
     * @struct ActiveBox
     * @brief Tracks an active body and its spawn timestamp for automatic lifetime deletion.
     */
    struct ActiveBox
    {
        Entity entity;
        double spawnTime;
    };
}

int main()
{
    using namespace RT_PhysicsCore;

    Renderer renderer(1280, 720, "Example: Falling Boxes");
    if (!renderer.IsValid())
    {
        return 1;
    }

    Engine engine(60.0);
    Scene scene;

    scene.AddSystem(std::make_unique<TransformPropagationSystem>(scene));
    auto pipeline = std::make_unique<PhysicsPipelineSystem>(
        scene, ResolutionSystem::SolverMode::SequentialImpulses, 4);
    PhysicsPipelineSystem* pipelinePtr = pipeline.get();
    scene.AddSystem(std::move(pipeline));
    scene.AddSystem(std::make_unique<RenderSystem>(scene, renderer));

    // Ground
    Entity ground = scene.CreateEntity();
    TransformComponent gtc;
    gtc.position = { 0.0f, -0.5f, 0.0f };
    gtc.scale = { 20.0f, 1.0f, 20.0f };
    scene.AddComponent(ground, gtc);
    scene.AddComponent(ground, MakeStaticBody());
    MeshComponent gm;
    gm.shape = PrimitiveShape::Cube;
    gm.color = { 0.3f, 0.4f, 0.3f };
    scene.AddComponent(ground, gm);
    ColliderComponent gcc;
    gcc.shape = ColliderShape::Box;
    gcc.size = { 10.0f, 0.5f, 10.0f };
    scene.AddComponent(ground, gcc);

    std::mt19937 rng(42);
    std::uniform_real_distribution<float> posDist(-4.0f, 4.0f);
    std::uniform_real_distribution<float> sizeDist(0.4f, 0.8f);
    std::uniform_real_distribution<float> colorDist(0.4f, 1.0f);
    std::uniform_real_distribution<float> rotDist(-3.14f, 3.14f);

    std::vector<ActiveBox> activeBoxes;
    double totalTime = 0.0;
    double spawnTimer = 0.0;

    bool drawContacts = false;
    bool drawNormals = false;
    bool showFps = false;

    auto resetSimulation = [&]()
        {
            for (const auto& box : activeBoxes)
            {
                scene.DestroyEntity(box.entity);
            }
            activeBoxes.clear();
            totalTime = 0.0;
            spawnTimer = 0.0;
        };

    engine.SetUpdateCallback([&](double dt)
        {
            scene.SetDeltaTime(dt);
            scene.UpdateSystems();

            auto& input = renderer.GetInput();
            if (input.WasKeyPressed(GLFW_KEY_F1))
            {
                drawContacts = !drawContacts;
            }
            if (input.WasKeyPressed(GLFW_KEY_F2))
            {
                drawNormals = !drawNormals;
            }
            if (input.WasKeyPressed(GLFW_KEY_F3))
            {
                showFps = !showFps;
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
            totalTime += dt;
            spawnTimer += dt;

            scene.SetFixedDeltaTime(dt);
            scene.FixedUpdateSystems();

            // 1. Recycle entities older than 20.0 seconds
            constexpr double maxLifetime = 20.0;
            size_t writeIdx = 0;
            for (size_t readIdx = 0; readIdx < activeBoxes.size(); ++readIdx)
            {
                if (totalTime - activeBoxes[readIdx].spawnTime >= maxLifetime)
                {
                    scene.DestroyEntity(activeBoxes[readIdx].entity);
                }
                else
                {
                    activeBoxes[writeIdx++] = activeBoxes[readIdx];
                }
            }
            activeBoxes.resize(writeIdx);

            // 2. Spawn a new body every 0.2s
            if (spawnTimer >= 0.2)
            {
                spawnTimer = 0.0;
                Entity e = scene.CreateEntity();
                float size = sizeDist(rng);
                bool isBox = posDist(rng) > 0.0f;

                TransformComponent tc;
                tc.position = { posDist(rng), 12.0f, posDist(rng) };
                tc.scale = { size * 2.0f, size * 2.0f, size * 2.0f };
                tc.rotation = glm::quat(glm::vec3(rotDist(rng), rotDist(rng), rotDist(rng)));
                scene.AddComponent(e, tc);

                MeshComponent mc;
                mc.shape = isBox ? PrimitiveShape::Cube : PrimitiveShape::Sphere;
                mc.color = { colorDist(rng), colorDist(rng), colorDist(rng) };
                scene.AddComponent(e, mc);

                float mass = size * size * size * 10.0f;
                scene.AddComponent(e, MakeDynamicBody(mass,
                    isBox ? ComputeBoxInertia(mass, glm::vec3(size)) : ComputeSphereInertia(mass, size)));

                ColliderComponent cc;
                cc.shape = isBox ? ColliderShape::Box : ColliderShape::Sphere;
                cc.size = { size, size, size };
                scene.AddComponent(e, cc);

                activeBoxes.push_back({ e, totalTime });
            }
        });

    engine.SetRenderCallback([&](double /*alpha*/)
        {
            renderer.BeginFrame();
            scene.RenderUpdateSystems();

            if (drawContacts || drawNormals)
            {
                for (const auto& c : pipelinePtr->GetCollisionSystem().GetContacts())
                {
                    for (int i = 0; i < c.pointCount; ++i)
                    {
                        if (drawContacts)
                        {
                            DebugDraw::Sphere(c.points[i].position, 0.05f, { 1.0f, 1.0f, 0.0f });
                        }
                        if (drawNormals)
                        {
                            DebugDraw::Line(c.points[i].position, c.points[i].position + c.normal * 0.4f, { 1.0f, 0.0f, 0.0f });
                        }
                    }
                }
            }

            renderer.FlushDebugDraw();

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